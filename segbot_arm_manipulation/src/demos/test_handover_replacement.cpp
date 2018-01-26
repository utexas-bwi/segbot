#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>
#include <iostream>
#include <cstdlib>

#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/grasp_utils.h>
#include <segbot_arm_manipulation/arm_positions_db.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"
#include "kinova_msgs/ArmJointAnglesAction.h"

#include "segbot_arm_manipulation/TabletopGraspAction.h"
#include "segbot_arm_manipulation/ObjReplacementAction.h"

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>


sensor_msgs::JointState current_state;
bool heardState;

geometry_msgs::PoseStamped current_pose;
bool heardPose;

#define NUM_JOINTS 8 //6+2 for the arm
#define FINGER_FULLY_CLOSED 7300

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

int main(int argc, char** argv){
	
	//initialize ros
	ros::init(argc, argv, "test_handover_replacement");
	ros::NodeHandle nh;
	
	//initialize booleans
	heardState = false;
	heardPose = false;
	
	//subscribers
	ros::Subscriber sub_angles = nh.subscribe ("/joint_states", 1, joint_state_cb);

	//publishers
	ros::Subscriber sub_tool = nh.subscribe("/m1n6s200_driver/out/tool_position", 1, pose_stamped_cb);

	listen_for_arm_data(10);
	
	//action clients
	actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopGraspAction> ac_grasp("segbot_tabletop_grasp_as",true);
	ac_grasp.waitForServer();

	actionlib::SimpleActionClient<segbot_arm_manipulation::ObjReplacementAction> ac_replace("segbot_obj_replacement_as",true);
	ac_replace.waitForServer();
	
	//load database of joint- and tool-space positions
	std::string j_pos_filename = ros::package::getPath("segbot_arm_manipulation")+"/data/jointspace_position_db.txt";
	std::string c_pos_filename = ros::package::getPath("segbot_arm_manipulation")+"/data/toolspace_position_db.txt";
	
	ArmPositionDB positionDB(j_pos_filename, c_pos_filename);
	positionDB.print();
	
	listen_for_arm_data(10);
	
	//Open hand and move to home position
	segbot_arm_manipulation::homeArm(nh);
	segbot_arm_manipulation::openHand();
	
	// Go to handover position 
	if (positionDB.hasCarteseanPosition("handover_front")){		
		geometry_msgs::PoseStamped handover_pose = positionDB.getToolPositionStamped("handover_front","m1n6s200_link_base");
		
		ROS_INFO("Moving to handover position"); 
		
		segbot_arm_manipulation::moveToPoseMoveIt(nh, handover_pose);
		
		pressEnter("Press [Enter] to proceed");
	}
	else {
		ROS_ERROR("handover_front position does not exist! Aborting...");
		return 1;
	}

	//now receive object
	std::cout << "Please place an object in robot's hand\n"; 
	segbot_arm_manipulation::TabletopGraspGoal receive_goal;
	receive_goal.action_name = segbot_arm_manipulation::TabletopGraspGoal::HANDOVER_FROM_HUMAN;
	receive_goal.timeout_seconds = -1.0;
	
	ac_grasp.sendGoal(receive_goal);
	ac_grasp.waitForResult();
	
	if(ac_grasp.getResult()->success == false) {
		ROS_ERROR("HANDOVER_FROM_HUMAN grasp failed:"); 
		return 1; 
	}
	
	segbot_arm_manipulation::homeArm(nh);
	if (positionDB.hasCarteseanPosition("side_view")){		
		geometry_msgs::PoseStamped side_pose = positionDB.getToolPositionStamped("side_view","m1n6s200_link_base");
		
		ROS_INFO("Moving to side position"); 
		
		segbot_arm_manipulation::moveToPoseMoveIt(nh, side_pose);
	}
	else {
		ROS_ERROR("side_view position does not exist! Aborting...");
		return 1;
	}
	
	pressEnter("Press [Enter] to proceed");


	//test object replacement
	segbot_arm_manipulation::ObjReplacementGoal replacement_goal; 
	ROS_INFO("Sending goal to replacement...");
	ac_replace.sendGoal(replacement_goal); 
	ac_replace.waitForResult(); 
	ROS_INFO("replacement action finished");

	if(!ac_replace.getResult()->success) {
		ROS_ERROR("Object replacement failed:"); 
		return 1; 
	}
	
}
