//includes
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

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

#include <segbot_arm_manipulation/ShakeAction.h>
#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/Mico.h>



typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class ShakeActionServer
{
protected:

  ros::NodeHandle nh_;
  
  actionlib::SimpleActionServer<segbot_arm_manipulation::ShakeAction> as_; 
  
  std::string action_name_;
  
  //messages for publishing feedback and result of action
  segbot_arm_manipulation::ShakeFeedback feedback_;
  segbot_arm_manipulation::ShakeResult result_;
  
  tf::TransformListener listener;

    segbot_arm_manipulation::Mico mico;

 
public:

  ShakeActionServer(std::string name) :
    as_(nh_, name, boost::bind(&ShakeActionServer::executeCB, this, _1), false),
    action_name_(name), mico(nh_)
  {
	ROS_INFO("Shake action has started");
	
    as_.start();
  }

  ~ShakeActionServer(void)
  {
  }

	//method to shake object, use planner
	void lift(ros::NodeHandle n, double x){
		mico.wait_for_data();

		geometry_msgs::PoseStamped p_target = mico.current_pose;

		p_target.pose.position.z += x;
		mico.move_to_pose_moveit(p_target);
	}	
	
	void shake_down(float duration){
		kinova_msgs::PoseVelocity v;
		
		v.twist_linear_x = 0;
		v.twist_linear_y = 0.0;
		v.twist_linear_z = 0.125;
		
		v.twist_angular_x = 0.0;
		v.twist_angular_y = 0.0;
		v.twist_angular_z = 0.0;
		
		float elapsed_time = 0.0;
		
		mico.wait_for_data();

		v.twist_linear_z = 0.125;
		mico.move_with_cartesian_velocities(v, duration);
	}
	
	
	void shake_up(float duration){
		kinova_msgs::PoseVelocity v;
		v.twist_linear_x = 0;
		v.twist_linear_y = 0.0;
		v.twist_linear_z = -0.125;

		v.twist_angular_x = 0.0;
		v.twist_angular_y = 0.0;
		v.twist_angular_z = 0.0;

		float elapsed_time = 0.0;

		mico.wait_for_data();

		mico.move_with_cartesian_velocities(v, duration);
		
	}

	
	void executeCB(const segbot_arm_manipulation::ShakeGoalConstPtr  &goal){
		
		mico.wait_for_data();
		
		if(goal -> tgt_cloud.data.size() == 0){
			result_.success = false;
			ROS_INFO("[arm_shake_as.cpp] No object point clouds received...aborting");
			as_.setAborted(result_);
			return;
		}
		
		if (as_.isPreemptRequested() || !ros::ok()){
			ROS_INFO("Shake action: Preempted");
			// set the action state to preempted
			as_.setPreempted();
			result_.success = false;
			as_.setSucceeded(result_);
			return;
        }
        //step 1: transform to mico link space
        std::string sensor_frame_id = goal -> tgt_cloud.header.frame_id;
			
		listener.waitForTransform(sensor_frame_id, "m1n6s200_link_base", ros::Time(0), ros::Duration(3.0));
		
		PointCloudT pcl_cloud;
		pcl::fromROSMsg(goal -> tgt_cloud, pcl_cloud);
		
		mico.wait_for_data();
				
		//step 2 : determine if object in in hand
		if(goal -> verified){
			//step 3: shake the object
			shake_up(4); //raise above table
			shake_down(1.5);
			shake_up(1.5);
			mico.open_hand();

		}else{
			//for now if object is not in hand, abort
			ROS_WARN("object must already be in hand... aborting");
			result_.success = false;
			as_.setAborted(result_);
			return;
		}
			
		//step 4: move arm home	
		mico.move_to_side_view();
		mico.move_to_side_view();

		//step 5: set result of action
		result_.success = true;
		as_.setSucceeded(result_);
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_shake_as");

  ShakeActionServer as(ros::this_node::getName());
  ros::spin();

  return 0;
}
