//includes
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/point_cloud_conversion.h>

#include <sensor_msgs/JointState.h>
#include <kinova_msgs/PoseVelocity.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/WrenchStamped.h>

//get table scene and color histogram
#include "bwi_perception/TabletopPerception.h"
#include <bwi_perception/bwi_perception.h>

//actions
#include <actionlib/server/simple_action_server.h>
#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"
#include "kinova_msgs/ArmJointAnglesAction.h"
#include <segbot_arm_manipulation/PressAction.h>
#include <segbot_arm_manipulation/arm_utils.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>


#include <segbot_arm_manipulation/MicoManager.h>

#define MIN_DISTANCE_TO_PLANE 0.05


class PressActionServer {
protected:

  ros::NodeHandle nh_;
  
  actionlib::SimpleActionServer<segbot_arm_manipulation::PressAction> as_; 
  
  std::string action_name_;
  
  //messages to publish feedback and result of action
  segbot_arm_manipulation::PressFeedback feedback_;
  segbot_arm_manipulation::PressResult result_;
  
  ros::Publisher debug_pub;
  
  sensor_msgs::JointState current_state;
  sensor_msgs::JointState home_state;
  
  tf::TransformListener listener;

	MicoManager mico;

 
public:

  PressActionServer(std::string name) :
    as_(nh_, name, boost::bind(&PressActionServer::executeCB, this, _1), false),
    action_name_(name), mico(nh_)
  {
	
	//advertise the goal pose for debugging
	debug_pub = nh_.advertise<geometry_msgs::PoseStamped>("/m1n6s200_driver/in/debug_pose", 2);
	
	ROS_INFO("Press action has started");
	
    as_.start();
  }

  ~PressActionServer(void)
  {
  }

	
	bool plane_distance(geometry_msgs::Pose start, Eigen::Vector4f plane_c){
		//filter 1: if too close to the plane
		pcl::PointXYZ p_a;
		p_a.x=start.position.x;
		p_a.y=start.position.y;
		p_a.z=start.position.z;
	
		return pcl::pointToPlaneDistance(p_a, plane_c);
	}


	//method using cartesian velocities to press down on the object

	void pushButton() {
		double timeoutSeconds = 2.0;
		kinova_msgs::PoseVelocity velocityMsg;

        velocityMsg.twist_linear_z = -0.125;
		mico.move_with_cartesian_velocities(velocityMsg, timeoutSeconds);

		//release
        velocityMsg.twist_linear_y = -0.125;
        velocityMsg.twist_linear_z = 0.2;
		mico.move_with_cartesian_velocities(velocityMsg, timeoutSeconds);

		
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
        mico.wait_for_data();
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
		mico.wait_for_data();
		
		mico.close_hand();
		
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
				moveit_msgs::GetPositionIK::Response ik_response = mico.compute_ik(stampedPose);
				if (ik_response.error_code.val == 1){
					break;
				}
		}
		
		//publish pose
		debug_pub.publish(stampedPose);
		ros::spinOnce();

		//move to pose
		mico.move_to_pose_moveit(stampedPose);
		mico.move_to_pose_moveit(stampedPose);
		mico.move_to_pose_moveit(stampedPose);
		pushButton();

		//home arm
		mico.move_home();
		
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
