#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>

#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/grasp_utils.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"
#include "kinova_msgs/ArmJointAnglesAction.h"

//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"
#include "segbot_arm_manipulation/LiftVerifyAction.h"
#include "segbot_arm_manipulation/TabletopGraspAction.h"

#include <moveit_msgs/DisplayRobotState.h>
// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

sensor_msgs::JointState current_state;
bool heardState;

geometry_msgs::PoseStamped current_pose;
bool heardPose;

#define NUM_JOINTS 8 //6+2 for the arm
#define FINGER_FULLY_CLOSED 7300

/*Blocking call for user input to ensure safety*/
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

void joint_state_cb(const sensor_msgs::JointStateConstPtr& input){
	if(input->position.size() == NUM_JOINTS){
		current_state = *input;
		heardState = true;
	}
}

void pose_stamped_cb(const geometry_msgs::PoseStamped &input){
	current_pose = input;
	heardPose = true;
}

/*Wait for up to date arm data */
void listen_for_arm_data(float rate){
	heardState = false;
	heardPose = false;
	ros::Rate r(rate);
		
	while (ros::ok()){
		ros::spinOnce();
		
		if (heardPose && heardState)
			return;
		
		r.sleep();
	}
}

int find_largest_cloud(segbot_arm_perception::TabletopPerception::Response table_scene){
	//ensure there are clouds found
	if ((int)table_scene.cloud_clusters.size() == 0){
			ROS_WARN("No objects found on table. The end...");
			exit(1);
	}
	//find the largest cloud
	int largest_pc_index = -1;
	int largest_num_points = -1;
	
	for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
		
		int num_points_i = table_scene.cloud_clusters[i].height* table_scene.cloud_clusters[i].width;
	
		if (num_points_i > largest_num_points){
			largest_num_points = num_points_i;
			largest_pc_index = i;
		}
	}
	return largest_pc_index;
}

int main(int argc, char** argv){
	
	//initialize ros
	ros::init(argc, argv, "test_grasp_replacement");
	ros::NodeHandle nh;
	
	//initialize booleans
	heardState = false;
	heardPose = false;
	
	//subscribers
	ros::Subscriber sub_angles = nh.subscribe ("/joint_states", 1, joint_state_cb);

	//publishers
	ros::Subscriber sub_tool = nh.subscribe("/m1n6s200_driver/out/tool_position", 1, pose_stamped_cb);

	listen_for_arm_data(10);
	
	//joint state for use in sending to replacement
	sensor_msgs::JointState grasped_state = current_state;
	
	/*Test the action*/
	
	//get table scene
	segbot_arm_perception::TabletopPerception::Response table_scene = segbot_arm_manipulation::getTabletopScene(nh);
				
	int largest_index = find_largest_cloud(table_scene);
				
	/*create action client for the grasping action*/ 
	actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopGraspAction> ac("segbot_tabletop_grasp_as",true);
	ac.waitForServer();
	ROS_INFO("action server made...");
				
	//create and fill goal
	segbot_arm_manipulation::TabletopGraspGoal grasp_goal;
	grasp_goal.action_name = segbot_arm_manipulation::TabletopGraspGoal::GRASP;
				
				
	//for that action, we have to specify the method used for picking the target grasp out of the candidates
	grasp_goal.grasp_selection_method=segbot_arm_manipulation::TabletopGraspGoal::CLOSEST_ORIENTATION_SELECTION;
				
	//finally, we fill in the table scene
	grasp_goal.cloud_plane = table_scene.cloud_plane;
	grasp_goal.cloud_plane_coef = table_scene.cloud_plane_coef;
				
	for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
		grasp_goal.cloud_clusters.push_back(table_scene.cloud_clusters[i]);
	}
	grasp_goal.target_object_cluster_index = largest_index;
						
	//send the goal
	ROS_INFO("Sending goal to action server...");
	ac.sendGoal(grasp_goal);
				
	//Wait for result
	ROS_INFO("Waiting for result...");
	ac.waitForResult();
	ROS_INFO("Action Finished...");

	listen_for_arm_data(10);
	grasped_state = current_state;

	/*Create action client for lift verification*/
	actionlib::SimpleActionClient<segbot_arm_manipulation::LiftVerifyAction> lift_ac("arm_lift_verify_as", true);
	lift_ac.waitForServer();
	ROS_INFO("lift and verify action server made...");
				
	//make goals to send to action
	segbot_arm_manipulation::LiftVerifyGoal lift_verify_goal;
	lift_verify_goal.tgt_cloud = table_scene.cloud_clusters[largest_index];
	lift_verify_goal.bins = 8;
				
	//wait for result
	ROS_INFO("sending goal to lift and verify action server...");
	lift_ac.sendGoal(lift_verify_goal);
	ROS_INFO("waiting for lift and verify action server result....");
	lift_ac.waitForResult();
				
	//get response from action
	ROS_INFO("lift and verify action finished.");
	segbot_arm_manipulation::LiftVerifyResult result = *lift_ac.getResult();
				
	//display success of action to user
	bool verified = result.success;
	if(verified){
		ROS_INFO("Verification succeeded.");
	}else{
		ROS_WARN("Verification failed");
	}
				
	segbot_arm_manipulation::homeArm(nh);
				
	/*create action for replacement*/
	actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopGraspAction> replacement_ac("segbot_tabletop_grasp_as",true);
	replacement_ac.waitForServer();
	ROS_INFO("action server made...");
				
	//create goal for the replacement
	segbot_arm_manipulation::TabletopGraspGoal replacement_goal;
	replacement_goal.action_name = segbot_arm_manipulation::TabletopGraspGoal::REPLACEMENT;
	replacement_goal.grasped_joint_state = grasped_state;
			
	ROS_INFO("Sending goal to action server...");
	ac.sendGoal(replacement_goal);
				
	//Wait for result
	ROS_INFO("Waiting for result...");
	ac.waitForResult();
	ROS_INFO("Action Finished...");
		
		
	
}
