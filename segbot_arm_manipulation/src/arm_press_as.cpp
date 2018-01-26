//includes
#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <stdlib.h>
#include <std_msgs/String.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/point_cloud_conversion.h>

#include <sensor_msgs/JointState.h>
#include <kinova_msgs/PoseVelocity.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/WrenchStamped.h>

//get table scene and color histogram
#include "segbot_arm_perception/TabletopPerception.h"
#include <segbot_arm_perception/segbot_arm_perception.h>

//actions
#include <actionlib/server/simple_action_server.h>
#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"
#include "kinova_msgs/ArmJointAnglesAction.h"
#include <segbot_arm_manipulation/PressAction.h>
#include <segbot_arm_manipulation/arm_utils.h>

//pcl includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//tf stuff
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <moveit_msgs/GetPositionIK.h>
//defines
#define FINGER_FULLY_OPENED 6
#define FINGER_FULLY_CLOSED 7300

#define NUM_JOINTS_ARMONLY 6
#define NUM_JOINTS 8 //6+2 for the arm

#define MIN_DISTANCE_TO_PLANE 0.05

class PressActionServer
{
protected:

  ros::NodeHandle nh_;
  
  actionlib::SimpleActionServer<segbot_arm_manipulation::PressAction> as_; 
  
  std::string action_name_;
  
  //messages to publish feedback and result of action
  segbot_arm_manipulation::PressFeedback feedback_;
  segbot_arm_manipulation::PressResult result_;
  
  ros::Publisher arm_vel;
  
  ros::Subscriber sub_angles;
  ros::Subscriber sub_torques;
  ros::Subscriber sub_tool;
  ros::Subscriber sub_finger;
  ros::Subscriber sub_wrench;
  
  ros::Publisher debug_pub;
  
  sensor_msgs::JointState current_state;
  
  sensor_msgs::JointState home_state;
  
  kinova_msgs::FingerPosition current_finger;
  geometry_msgs::PoseStamped current_pose;
  geometry_msgs::WrenchStamped current_wrench; 
  
  bool heardPose;
  bool heardJoinstState;
  bool heardWrench;
  
  tf::TransformListener listener;

 
public:

  PressActionServer(std::string name) :
    as_(nh_, name, boost::bind(&PressActionServer::executeCB, this, _1), false),
    action_name_(name)
  {
	heardPose = false;
	heardJoinstState = false;
	heardWrench = false;
	
	//publisher to move arm down
	arm_vel= nh_.advertise<kinova_msgs::PoseVelocity>("/m1n6s200_driver/in/cartesian_velocity", 2);

	//create subscriber to joint angles
	sub_angles = nh_.subscribe ("/m1n6s200_driver/out/joint_state", 1, &PressActionServer::joint_state_cb, this);

	//create subscriber to tool position topic
	sub_tool = nh_.subscribe("/m1n6s200_driver/out/tool_pose", 1, &PressActionServer::toolpos_cb, this);

	//subscriber for fingers
	sub_finger = nh_.subscribe("/m1n6s200_driver/out/finger_position", 1, &PressActionServer::fingers_cb, this);
	  
	//subscriber for wrench
	sub_wrench = nh_.subscribe("/m1n6s200_driver/out/tool_wrench", 1, &PressActionServer::wrench_cb, this);
	
	//advertise the goal pose for debugging
	debug_pub = nh_.advertise<geometry_msgs::PoseStamped>("/m1n6s200_driver/in/debug_pose", 2);
	
	ROS_INFO("Press action has started");
	
    as_.start();
  }

  ~PressActionServer(void)
  {
  }
  
  //Joint state cb
	void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
		if (input->position.size() == NUM_JOINTS){
			current_state = *input;
			heardJoinstState = true;
		}
	}

	//tool position cb
	void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
	  current_pose = msg;
	  heardPose = true;
	}

	//fingers state cb
	void fingers_cb (const kinova_msgs::FingerPosition msg) {
	  current_finger = msg;
	}

	//Callback for toolwrench 
	void wrench_cb(const geometry_msgs::WrenchStamped &msg){ 
		current_wrench = msg;
		heardWrench = true;
	}
	
	//wait for updated data	
	void listenForArmData(float rate){
		heardPose = false;
		heardJoinstState = false;
		heardWrench = false;
		ros::Rate r(rate);
		
		while (ros::ok()){
			ros::spinOnce();
			
			if (heardPose && heardJoinstState && heardWrench)
				return;
			
			r.sleep();
		}
	}
	
	bool plane_distance(geometry_msgs::Pose start, Eigen::Vector4f plane_c){
		//filter 1: if too close to the plane
		pcl::PointXYZ p_a;
		p_a.x=start.position.x;
		p_a.y=start.position.y;
		p_a.z=start.position.z;
	
		return pcl::pointToPlaneDistance(p_a, plane_c);
	}

	moveit_msgs::GetPositionIK::Response computeIK(ros::NodeHandle n, geometry_msgs::PoseStamped p){
		ros::ServiceClient ikine_client = n.serviceClient<moveit_msgs::GetPositionIK> ("/compute_ik");
	
		moveit_msgs::GetPositionIK::Request ikine_request;
		moveit_msgs::GetPositionIK::Response ikine_response;
		ikine_request.ik_request.group_name = "arm";
		ikine_request.ik_request.pose_stamped = p;
	
		/* Call the service */
		if(ikine_client.call(ikine_request, ikine_response)){
			ROS_INFO("IK service call success:");
			//ROS_INFO_STREAM(ikine_response);
		} else {
			ROS_INFO("IK service call FAILED. Exiting");
		}
	
		return ikine_response;
	}
	
	//method using cartesian velocities to press down on the object

	void pushButton() {
		double timeoutSeconds = 2.0;
		int rateHertz = 40;
		geometry_msgs::TwistStamped velocityMsg;
		ros::Rate r(rateHertz);
		
		//push down
		for(int i = 0; i < (int)timeoutSeconds * rateHertz; i++) {
			velocityMsg.twist.linear.x = 0;
			velocityMsg.twist.linear.y = 0.0;
			velocityMsg.twist.linear.z = -0.125;
		
			velocityMsg.twist.angular.x = 0.0;
			velocityMsg.twist.angular.y = 0.0;
			velocityMsg.twist.angular.z = 0.0;
		
			ros::spinOnce();
			arm_vel.publish(velocityMsg);
		
			r.sleep();
		}
	
		//release
		for(int i = 0; i < (int)3.0 * rateHertz; i++) {
			velocityMsg.twist.linear.x = 0.0;
			velocityMsg.twist.linear.y = -0.125;
			velocityMsg.twist.linear.z = 0.2;
		
			velocityMsg.twist.angular.x = 0.0;
			velocityMsg.twist.angular.y = 0.0;
			velocityMsg.twist.angular.z = 0.0;
		
		
		}
		
	}
	
	//generates stamedPose over center of object
	geometry_msgs::PoseStamped pclToPoseStamped(sensor_msgs::PointCloud2 pc2){
		//transform to PCL
		pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
		pcl::fromROSMsg(pc2, pcl_cloud);
		
		//find center of point cloud
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(pcl_cloud, centroid);
		
		//find min and max of point cloud
		pcl::PointXYZ min;
		pcl::PointXYZ max;
		pcl::getMinMax3D(pcl_cloud, min, max);
		
		geometry_msgs::PoseStamped pose_i;
		pose_i.header.frame_id = pc2.header.frame_id;
		pose_i.header.stamp = ros::Time(0);
		pose_i.pose.position.x=centroid(0);
		pose_i.pose.position.y=centroid(1)+0.1;
		pose_i.pose.position.z=max.z;
		pose_i.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
		return pose_i;
	}

	//create stamped pose from point cloud
	std::vector<geometry_msgs::Pose> generate_poses(sensor_msgs::PointCloud2 pc2){
		//transform to PCL
		pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
		pcl::fromROSMsg(pc2, pcl_cloud);
		
		//create a pose with x y z set to the center of point cloud
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(pcl_cloud, centroid);
	
		pcl::PointXYZ min;
		pcl::PointXYZ max;
		pcl::getMinMax3D(pcl_cloud, min, max);
		
		std::vector<geometry_msgs::Pose> start_poses;
		geometry_msgs::Pose pose_i;
		pose_i.position.x=centroid(0);
		pose_i.position.y=centroid(1);
		pose_i.position.z=max.z;
		//orientation 1 
		pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,3.14/2,3.14/2);
		start_poses.push_back(pose_i);
		//orientation 2 
		pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,-3.14/2,3.14/2);
		start_poses.push_back(pose_i);
		//orientation 3
		pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,3.14/2,0);
		start_poses.push_back(pose_i);
		
		return start_poses;
	}


	void executeCB(const segbot_arm_manipulation::PressGoalConstPtr  &goal){
		listenForArmData(30.0);
		
		if(goal -> tgt_cloud.data.size() == 0){
			result_.success = false;
			ROS_INFO("[arm_press_as.cpp] No object point clouds received...aborting");
			as_.setAborted(result_);
			return;
		}
		
		if (as_.isPreemptRequested() || !ros::ok()){
			ROS_INFO("Press action: Preempted");
			// set the action state to preempted
			as_.setPreempted();
			result_.success = false;
			as_.setSucceeded(result_);
			return;
        }
        
		Eigen::Vector4f plane_coef_vector;
		for (int i = 0; i < 4; i ++)
			plane_coef_vector(i)=goal->cloud_plane_coef[i];
		ROS_INFO("[push_as.cpp] Received action request...proceeding.");
		listenForArmData(40.0);
		
		segbot_arm_manipulation::closeHand();
		
		//wait for transform and perform it
		listener.waitForTransform(goal->tgt_cloud.header.frame_id,"m1n6s200_link_base",ros::Time(0), ros::Duration(3.0)); 
		
		//transform to base link frame of reference
	    sensor_msgs::PointCloud2 obj_cloud = goal->tgt_cloud;
		pcl_ros::transformPointCloud ("m1n6s200_link_base", obj_cloud, obj_cloud, listener);
		
		//generate array of poses
		std::vector<geometry_msgs::Pose> app_pos = generate_poses(obj_cloud);
		
		geometry_msgs::PoseStamped stampedPose;
		stampedPose.header.frame_id = obj_cloud.header.frame_id;
		stampedPose.header.stamp = ros::Time(0);
		
		//determine which pose can be reached
		for(int i = 0; i < app_pos.size(); i++){
			int dist = plane_distance(app_pos.at(i), plane_coef_vector);
			if(dist < MIN_DISTANCE_TO_PLANE){
				int threshold = MIN_DISTANCE_TO_PLANE - dist;
				app_pos.at(i).position.z += threshold;
			}
				stampedPose.pose = app_pos.at(i);
				moveit_msgs::GetPositionIK::Response ik_response = computeIK(nh_,stampedPose);
				if (ik_response.error_code.val == 1){
					break;
				}
		}
		
		//publish pose
		debug_pub.publish(stampedPose);
		ros::spinOnce();

		//move to pose
		segbot_arm_manipulation::moveToPoseMoveIt(nh_, stampedPose);
		segbot_arm_manipulation::moveToPoseMoveIt(nh_, stampedPose);
		segbot_arm_manipulation::moveToPoseMoveIt(nh_, stampedPose);
		pushButton();

		//home arm
		segbot_arm_manipulation::homeArm(nh_);
		
		//set result of action
		result_.success = true;
		as_.setSucceeded(result_);
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_press_as");

  PressActionServer as(ros::this_node::getName());
  ros::spin();

  return 0;
}
