#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */


#include <sensor_msgs/JointState.h>
#include <kinova_msgs/PoseVelocity.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/grasp_utils.h>


//actions
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"
#include "kinova_msgs/ArmJointAnglesAction.h"
#include <segbot_arm_manipulation/PushAction.h>


//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"

#include <moveit_msgs/DisplayRobotState.h>
// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>


#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>

//the action definition
#include "segbot_arm_manipulation/PushAction.h"

//tf stuff
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#define FINGER_FULLY_OPENED 6
#define FINGER_FULLY_CLOSED 7300

#define NUM_JOINTS_ARMONLY 6
#define NUM_JOINTS 8 //6+2 for the arm

#define MIN_DISTANCE_TO_PLANE 0.05

class PushActionServer
{
protected:
	
	ros::NodeHandle nh_;
	// NodeHandle instance must be created before this line. Otherwise strange error may occur.
	actionlib::SimpleActionServer<segbot_arm_manipulation::PushAction> as_; 
	std::string action_name_;
	// create messages that are used to published feedback/result
	segbot_arm_manipulation::PushFeedback feedback_;
	segbot_arm_manipulation::PushResult result_;

  
	
	sensor_msgs::JointState current_state;
	kinova_msgs::FingerPosition current_finger;
	geometry_msgs::PoseStamped current_pose;
	bool heardPose;
	bool heardJoinstState; 
	
	bool heardGrasps;
	agile_grasp::Grasps current_grasps;

	ros::Publisher pub_velocity;
	ros::Publisher debug_pub;
	ros::Publisher cloud_pub;
	ros::Publisher cloud_grasp_pub;
	ros::Publisher pose_array_pub;
	ros::Publisher pose_pub;
	ros::Publisher pose_fk_pub;
	
	//used to compute transforms
	tf::TransformListener listener;
	
	bool heard_string;
	ros::Subscriber sub_string_;
	
	
	//subscribers -- in an action server, these have to be class members
	ros::Subscriber sub_angles;
	ros::Subscriber sub_torques;
	ros::Subscriber sub_tool;
	ros::Subscriber sub_finger;
	ros::Subscriber sub_grasps;
	
public:

  PushActionServer(std::string name) :
	as_(nh_, name, boost::bind(&PushActionServer::executeCB, this, _1), false),
    action_name_(name)
  {
	
	
	
	heardPose = false;
	heardJoinstState = false;
	
	heardGrasps = false;

	//create subscriber to joint angles
	sub_angles = nh_.subscribe ("/m1n6s200_driver/out/joint_state", 1, &PushActionServer::joint_state_cb, this);

	//create subscriber to tool position topic
	sub_tool = nh_.subscribe("/m1n6s200_driver/out/tool_pose", 1, &PushActionServer::toolpos_cb, this);

	//publisher for velocities
	pub_velocity = nh_.advertise<geometry_msgs::TwistStamped>("/m1n6s200_driver/in/cartesian_velocity", 10);

	//advertise the goal pose for debugging
	debug_pub = nh_.advertise<geometry_msgs::PoseStamped>("/m1n6s200_driver/in/debug_pose", 2);

    ROS_INFO("Starting push action server...");
    
    as_.start();
  }

  ~PushActionServer(void)
  {
  }
  
	void test_string_cb(const std_msgs::String& msg){
		ROS_INFO("Received string!");
		heard_string = true;
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
	
	void listenForArmData(float rate){
		heardPose = false;
		heardJoinstState = false;
		ros::Rate r(rate);
		
		while (ros::ok()){
			ros::spinOnce();
			
			if (heardPose && heardJoinstState)
				return;
			
			r.sleep();
		}
	}	
		
	void spinSleep(double duration){
		int rateHertz = 40;	
		ros::Rate r(rateHertz);
		for(int i = 0; i < (int)duration * rateHertz; i++) {
			ros::spinOnce();
			r.sleep();
		}
	}
	
	// Blocking call for user input
	void pressEnter(std::string message){
		std::cout << message;
		while (true){
			char c = std::cin.get();
			if (c == '\n')
				break;
			else if (c == 'q'){
				ros::shutdown();
				exit(1);
			}
			else {
				std::cout <<  message;
			}
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

	//create stamped pose from point cloud
	std::vector<geometry_msgs::Pose> generate_poses(sensor_msgs::PointCloud2 pc2){
		//transform to PCL
		pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
		pcl::fromROSMsg(pc2, pcl_cloud);
		
		//find center, max, and min of point cloud
		pcl::PointXYZ min;
		pcl::PointXYZ max;
		pcl::getMinMax3D(pcl_cloud, min, max);
		
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(pcl_cloud, centroid);
	
		std::vector<geometry_msgs::Pose> start_poses;
	
		//right	
		//orientation 1 \/
		geometry_msgs::Pose pose_i;
		pose_i.position.x = centroid(0);
		pose_i.position.y = min.y - 0.03;
		pose_i.position.z = centroid(2);
		pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,3.14,3.14/2);
		start_poses.push_back(pose_i);
	
		//orientation 2
		pose_i.position.z = centroid(2) - 0.02;
		pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,3.14/2,0);
		start_poses.push_back(pose_i);
	
		//left
		//orientation 2
		pose_i.position.y = max.y + 0.03;
		start_poses.push_back(pose_i);
	
		//orientation 1
		pose_i.position.z = centroid(2);
		pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,3.14,3.14/2);
		start_poses.push_back(pose_i);
	
		//front
		//orientation 1
		pose_i.position.y = centroid(1);
		pose_i.position.x = min.x - 0.05;
		start_poses.push_back(pose_i);
	
		//orientation 3 <
		pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,3.14/2,3.14/2);
		start_poses.push_back(pose_i);
	
		//orientation 4 >
		pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,-3.14/2,3.14/2);
		start_poses.push_back(pose_i);

		return start_poses;
	}



	void push(int index) {
		double timeoutSeconds = 3.0;
		int rateHertz = 40;
		geometry_msgs::TwistStamped velocityMsg;
		ros::Rate r(rateHertz);
	
		if(index==0 || index==1){
			for(int i = 0; i < (int)timeoutSeconds * rateHertz; i++) {
				velocityMsg.twist.linear.x = 0.0;
				velocityMsg.twist.linear.y = 0.125;
				velocityMsg.twist.linear.z = 0.0;
		
				velocityMsg.twist.angular.x = 0.0;
				velocityMsg.twist.angular.y = 0.0;
				velocityMsg.twist.angular.z = 0.0;
		
				ros::spinOnce();
				pub_velocity.publish(velocityMsg);
		
				r.sleep();
			}
		}
		else if(index==2 || index==3){
			for(int i = 0; i < (int)timeoutSeconds * rateHertz; i++) {
				velocityMsg.twist.linear.x = 0.0;
				velocityMsg.twist.linear.y = -0.125;
				velocityMsg.twist.linear.z = 0.0;
		
				velocityMsg.twist.angular.x = 0.0;
				velocityMsg.twist.angular.y = 0.0;
				velocityMsg.twist.angular.z = 0.0;
		
				ros::spinOnce();
				pub_velocity.publish(velocityMsg);
		
				r.sleep();
			}
		}
		else{
			for(int i = 0; i < (int)timeoutSeconds * rateHertz; i++) {
				velocityMsg.twist.linear.x = 0.125;
				velocityMsg.twist.linear.y = 0.0;
				velocityMsg.twist.linear.z = 0.0;
		
				velocityMsg.twist.angular.x = 0.0;
				velocityMsg.twist.angular.y = 0.0;
				velocityMsg.twist.angular.z = 0.0;
		
				ros::spinOnce();
				pub_velocity.publish(velocityMsg);
		
				r.sleep();
			}
		}
	
	}

	void executeCB(const segbot_arm_manipulation::PushGoalConstPtr  &goal)
	{
		
		if (goal->tgt_cloud.data.size() == 0){
			ROS_INFO("[push_as.cpp] No object point clouds received...aborting");
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
	
		//get the data out of the goal
	    Eigen::Vector4f plane_coef_vector;
		for (int i = 0; i < 4; i ++)
			plane_coef_vector(i)=goal->cloud_plane_coef[i];
		ROS_INFO("[push_as.cpp] Received action request...proceeding.");
		listenForArmData(40.0);
		
		segbot_arm_manipulation::closeHand();
		
		//wait for transform and perform it
		listener.waitForTransform(goal->tgt_cloud.header.frame_id,"m1n6s200_link_base",ros::Time::now(), ros::Duration(3.0)); 

		//transform to base link frame of reference
		sensor_msgs::PointCloud2 obj_cloud = goal->tgt_cloud;
		pcl_ros::transformPointCloud ("m1n6s200_link_base", obj_cloud, obj_cloud, listener);
		
		//find possible start poses
		std::vector<geometry_msgs::Pose> app_pos = generate_poses(obj_cloud);
		
		geometry_msgs::PoseStamped stampedPose;
		stampedPose.header.frame_id = obj_cloud.header.frame_id;
		stampedPose.header.stamp = ros::Time(0);
	
		//determine which poses can be reached
		int result_i; 
		for(int i = 0; i < app_pos.size(); i++){
			int dist = plane_distance(app_pos.at(i), plane_coef_vector);
			if(dist < MIN_DISTANCE_TO_PLANE){
				int threshold = MIN_DISTANCE_TO_PLANE - dist;
				app_pos.at(i).position.z += threshold;
			}
				stampedPose.pose = app_pos.at(i);
				result_i= i;
				moveit_msgs::GetPositionIK::Response ik_response = computeIK(nh_,stampedPose);
				if (ik_response.error_code.val == 1){
					break;
				}
		}
		
		debug_pub.publish(stampedPose);
		ros::spinOnce();

		//move to pose and push
		segbot_arm_manipulation::moveToPoseMoveIt(nh_, stampedPose);
		segbot_arm_manipulation::moveToPoseMoveIt(nh_, stampedPose);
		segbot_arm_manipulation::moveToPoseMoveIt(nh_, stampedPose);
		push(result_i);
		
		//move arm out of view for redetection of object
		segbot_arm_manipulation::arm_side_view(nh_);
		
		
		//set result of action
		result_.success = true;
		as_.setSucceeded(result_);
		
	}
			
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_push_as");
  
  PushActionServer as(ros::this_node::getName());
  ros::spin();

  return 0;
}
