#include <ros/ros.h>
#include <ros/package.h>

#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/grasp_utils.h>

#include <actionlib/server/simple_action_server.h>

#include "segbot_arm_manipulation/TabletopGraspAction.h"
#include "segbot_arm_manipulation/ObjReplacementAction.h"

#include <geometry_msgs/PoseArray.h>
#include <segbot_arm_manipulation/MicoManager.h>


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

int main(int argc, char** argv){
	
	//initialize ros
	ros::init(argc, argv, "test_handover_replacement");
	ros::NodeHandle nh;

	MicoManager mico(nh);


	mico.wait_for_data();
	
	//action clients
	actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopGraspAction> ac_grasp("segbot_tabletop_grasp_as",true);
	ac_grasp.waitForServer();

	actionlib::SimpleActionClient<segbot_arm_manipulation::ObjReplacementAction> ac_replace("segbot_obj_replacement_as",true);
	ac_replace.waitForServer();
	

	mico.wait_for_data();
	
	//Open hand and move to home position
	mico.move_home();
	mico.open_hand();
	
	// Go to handover position 
	mico.move_to_handover();
		
    pressEnter("Press [Enter] to proceed");

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
	
	mico.move_home();
	mico.move_to_side_view();
	
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
