#include <ros/ros.h>
#include <ros/package.h>

#include <signal.h>

#include <sensor_msgs/JointState.h>

//srv for talking to table_object_detection_node.cpp
#include "bwi_perception/bwi_perception.h"

//action for grasping
#include "segbot_arm_manipulation/TabletopGraspAction.h"
#include "segbot_arm_manipulation/TabletopApproachAction.h"
#include "segbot_arm_manipulation/HandoverAction.h"

#include "plan_execution/ExecutePlanAction.h"

#include <move_base_msgs/MoveBaseAction.h>

//audio service
#include "bwi_services/SpeakMessage.h"

#include <bwi_msgs/QuestionDialog.h>
#include <segbot_arm_manipulation/Mico.h>


segbot_arm_manipulation::Mico *mico;


//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};


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
	
	plan_execution::ExecutePlanGoal goal_asp;
    plan_execution::AspRule rule;
    plan_execution::AspFluent fluent;
    fluent.variables.push_back("l3_414a");
    
    actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> client_asp("/action_executor/execute_plan", true);
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

    mico = new segbot_arm_manipulation::Mico(n);

	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	//action clients
	actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopGraspAction> ac_grasp("segbot_tabletop_grasp_as",true);
	ac_grasp.waitForServer();

	actionlib::SimpleActionClient<segbot_arm_manipulation::HandoverAction> ac_handover("segbot_handover_as",true);
	ac_grasp.waitForServer();


	//move arm into the handover view
	mico->move_to_handover();

	//now receive object
	std::cout << "Please place an object in robot's hand\n"; 
	segbot_arm_manipulation::HandoverGoal receive_goal;
	receive_goal.type = segbot_arm_manipulation::HandoverGoal::RECEIVE;
	receive_goal.timeout_seconds = -1.0;
	
	ac_handover.sendGoal(receive_goal);
	ac_handover.waitForResult();
	
	if(ac_handover.getResult()->success == false) {
		ROS_ERROR("HANDOVER_FROM_HUMAN grasp failed:"); 
		return 1; 
	}
	
	//now make safe
	mico->move_home();
	ROS_INFO("Making arm safe for travel"); 
	bool safe = mico->make_safe_for_travel();
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
	plan_execution::ExecutePlanGoal goal_asp;
    plan_execution::AspRule rule;
    plan_execution::AspFluent fluent;
    fluent.name = "not facing";
    fluent.variables.push_back(delivery_door);
	
	actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> client_asp("/action_executor/execute_plan", true);
	client_asp.waitForServer();

	rule.body.push_back(fluent);
    goal_asp.aspGoal.push_back(rule);

    ROS_INFO("sending goal");
    client_asp.sendGoalAndWait(goal_asp);	
	
	//now home and let go of object 
	mico->move_home();
	
	//move arm to handover view
	mico->move_to_handover();
	
	//play audio message 
	ros::ServiceClient speakMessageClient = n.serviceClient<bwi_services::SpeakMessage>("/speak_message_service/speak_message");  
	bwi_services::SpeakMessage speakSrv;
	speakSrv.request.message = "Special delivery"; 
	speakMessageClient.call(speakSrv);
	
	std::cout << "Please take the object from the robot's hand\n"; 
	
	segbot_arm_manipulation::HandoverGoal handover_goal;
	handover_goal.type = segbot_arm_manipulation::HandoverGoal::GIVE;
	handover_goal.timeout_seconds = -1.0;
	
	ac_handover.sendGoal(handover_goal);
	ac_handover.waitForResult();
	
	if(ac_handover.getResult()->success == false) {
		ROS_ERROR("HANDOVER grasp failed:"); 
		return 1; 
	}

	// make ready for travel
	mico->move_home();
	mico->close_hand();
	mico->make_safe_for_travel();
	
	returnToHome(); 
}
