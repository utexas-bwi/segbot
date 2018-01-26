#include <ros/ros.h>
#include <ros/package.h>

#include <signal.h>

#include <sensor_msgs/JointState.h>

//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"

//action for grasping
#include "segbot_arm_manipulation/TabletopGraspAction.h"
#include "segbot_arm_manipulation/TabletopApproachAction.h"

#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/arm_positions_db.h>

#include <segbot_arm_perception/segbot_arm_perception.h>


#include "bwi_kr_execution/ExecutePlanAction.h"

#include <move_base_msgs/MoveBaseAction.h>


#include <moveit_utils/MicoNavSafety.h>

#include <iostream>
#include <string>

//audio service
#include "bwi_services/SpeakMessage.h"

#include <bwi_msgs/QuestionDialog.h>

#define NUM_JOINTS 8 //6+2 for the arm


//global variables for storing data
sensor_msgs::JointState current_state;
bool heardJoinstState;

geometry_msgs::PoseStamped current_pose;
bool heardPose;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

//Joint state cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
	
	if (input->position.size() == NUM_JOINTS){
		current_state = *input;
		heardJoinstState = true;
	}
}

//Joint state cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
  current_pose = msg;
  heardPose = true;
}

//blocking call to listen for arm data (in this case, joint states)
void listenForArmData(){
	
	heardJoinstState = false;
	heardPose = false;
	ros::Rate r(10.0);
	
	while (ros::ok()){
		ros::spinOnce();	
		
		if (heardJoinstState && heardPose)
			return;
		
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

// Asks the user what door the robot should deliver the object to 
std::string askDoor(std::string default_door) {
	std::cout << "Default door:" + default_door + "\n";
	std::cout << "Change door (y or n)? "; 
	while(true) {
		char c = std::cin.get(); 
		if (c == 'n') {
			return default_door; 
		}
		else if (c == 'y') {
			break;
		}
		else {
			std::cout << "Change door (y or n)? "; 
		}
	}  
	while (true) {
		std::string input; 
		std::getline(std::cin,input); 
		std::cout << "Please enter a door to deliver object to: "; 
		std::getline(std::cin, input);
		if(input.size() < 6 || (input.at(0) != 'd' && input.at(2) != '_')) {
			std::cout << "Door must be in following form: d<floor number>_<room number/door number> (Ex. d3_414b2)\n"; 
		} 
		else {
			return input;  
		}
	}
}

// Asks the user if they want the robot to return to original position
// If yes, sends robot to original position
void returnToHome(){
	std::string input;
	std::getline(std::cin, input); 
	std::string message = "Return to room 3.414a? Enter y or n)";  
	std::cout << message; 
	while (true) {
		char c = std::cin.get(); 
		if (c == 'y') {
			break; 
		}
		else if (c == 'n') {
			return; 
		}
		else {
			std::cout << message; 
		}
	}
	
	bwi_kr_execution::ExecutePlanGoal goal_asp;
    bwi_kr_execution::AspRule rule;
    bwi_kr_execution::AspFluent fluent;
    fluent.variables.push_back("l3_414a");
    
    actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> client_asp("/action_executor/execute_plan", true);
	client_asp.waitForServer();

	rule.body.push_back(fluent);
    goal_asp.aspGoal.push_back(rule);
    
    ROS_INFO("sending goal");
    client_asp.sendGoalAndWait(goal_asp);
	
}

std::vector<std::string> getRooms() {
	std::vector<std::string> rooms;
	rooms.push_back("Robot Arm Lab"); 
	rooms.push_back("BWI Lab"); 
	rooms.push_back("Conference Room"); 
	rooms.push_back("Seminar Room"); 
	rooms.push_back("Robo Soccer Lab"); 
	rooms.push_back("Peter Stone's Office"); 
	rooms.push_back("Jivko Sinapov's Office"); 
	return rooms; 
}

std::vector<std::string> getDoors() {
	std::vector<std::string> doors;
	doors.push_back("d3_414a2"); 
	doors.push_back("d3_414b2");
	doors.push_back("d3_416"); 
	doors.push_back("d3_516"); 
	doors.push_back("d3_436");
	doors.push_back("d3_508"); 
	doors.push_back("d3_432"); 
	return doors; 
}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "object_handover_delivery_task");
	
	ros::NodeHandle n;

	//make vectors for rooms and doors
	std::vector<std::string> rooms = getRooms();
	std::vector<std::string> doors = getDoors(); 

	//create subscribers for arm topics
	ros::Subscriber sub_angles = n.subscribe ("/m1n6s200_driver/out/joint_state", 1, joint_state_cb);
	ros::Subscriber sub_tool = n.subscribe("/m1n6s200_driver/out/tool_pose", 1, toolpos_cb);

	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	//action clients
	actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopGraspAction> ac_grasp("segbot_tabletop_grasp_as",true);
	ac_grasp.waitForServer();
	
	//move arm into the handover view
	segbot_arm_manipulation::arm_handover_view(n);

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
	
	//now make safe
	segbot_arm_manipulation::homeArm(n);
	ROS_INFO("Making arm safe for travel"); 
	bool safe = segbot_arm_manipulation::makeSafeForTravel(n);
	pressEnter("Press [Enter] to proceed with navigation");
	
		
	//give robot goal

	//get goal from text input
	//std::string delivery_door = askDoor("d3_414b2");

	//get goal from gui input 
	int delivery_index = 0; 

	ros::ServiceClient client_gui = n.serviceClient<bwi_msgs::QuestionDialog>("/question_dialog");

	bwi_msgs::QuestionDialog question;
	question.request.type = question.request.CHOICE_QUESTION;
	
	question.request.message = "Please select a location to deliver object to:";
	for (unsigned int k = 0; k < rooms.size(); k++){
		question.request.options.push_back(rooms.at(k));
	}

	question.request.timeout = 30.0;
	
	if (client_gui.call(question))
    {
		if (question.response.index >= 0){
			delivery_index = question.response.index;
		}
		else {
			ROS_INFO("No response detected, defaulting to Robot Arm Lab");
			delivery_index = 0;
		}
	}
	else
	{
		ROS_ERROR("Failed to call service /question_dialog");
		return 1;
	}

	std::string delivery_door = doors.at(delivery_index); 
	ROS_INFO_STREAM("Going to " << delivery_door);
	
	//now travel to goal 
	bwi_kr_execution::ExecutePlanGoal goal_asp;
    bwi_kr_execution::AspRule rule;
    bwi_kr_execution::AspFluent fluent;
    fluent.name = "not facing";
    fluent.variables.push_back(delivery_door);
	
	actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> client_asp("/action_executor/execute_plan", true);
	client_asp.waitForServer();

	rule.body.push_back(fluent);
    goal_asp.aspGoal.push_back(rule);

    ROS_INFO("sending goal");
    client_asp.sendGoalAndWait(goal_asp);	
	
	//now home and let go of object 
	segbot_arm_manipulation::homeArm(n);
	
	//move arm to handover view
	segbot_arm_manipulation::arm_handover_view(n);
	
	//play audio message 
	ros::ServiceClient speakMessageClient = n.serviceClient<bwi_services::SpeakMessage>("/speak_message_service/speak_message");  
	bwi_services::SpeakMessage speakSrv;
	speakSrv.request.message = "Special delivery"; 
	speakMessageClient.call(speakSrv);
	
	std::cout << "Please take the object from the robot's hand\n"; 
	
	segbot_arm_manipulation::TabletopGraspGoal handover_goal;
	handover_goal.action_name = segbot_arm_manipulation::TabletopGraspGoal::HANDOVER;
	handover_goal.timeout_seconds = -1.0;
	
	ac_grasp.sendGoal(handover_goal);
	ac_grasp.waitForResult();
	
	if(ac_grasp.getResult()->success == false) {
		ROS_ERROR("HANDOVER grasp failed:"); 
		return 1; 
	}

	// make ready for travel
	segbot_arm_manipulation::homeArm(n);
	segbot_arm_manipulation::closeHand();
	segbot_arm_manipulation::makeSafeForTravel(n);
	
	returnToHome(); 
}
