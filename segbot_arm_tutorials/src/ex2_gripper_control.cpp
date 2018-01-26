#include <ros/ros.h>
#include <ros/package.h>

//used for registering Ctrl-C events
#include <signal.h>

//ROS messages
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

//ROS action client used for calling actions
#include <actionlib/client/simple_action_client.h>

//JACO messages and actions
#include <kinova_msgs/FingerPosition.h>
#include <kinova_msgs/SetFingersPositionAction.h>

//our own arm library
#include <segbot_arm_manipulation/MicoManager.h>

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


int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "ex2_gripper_control");
	
	ros::NodeHandle n;

	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	//open the hand using an action call
	
	actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> ac("/m1n6s200_driver/fingers_action/finger_positions", true);
	ac.waitForServer();

	//construction the action request
	kinova_msgs::SetFingersPositionGoal goalFinger;
	goalFinger.fingers.finger1 = 100; //100 is open, 7500 is close
	goalFinger.fingers.finger2 = 100;
		
	//send the goal and block until completed
	pressEnter("Press [Enter] to open the hand.");
	ac.sendGoal(goalFinger);
	ac.waitForResult();

	// MicoManager is a class that facilitates many common arm motion activities
	MicoManager mico(n);
	//now, close the hand using an API call from segbot_arm_manipulation
	pressEnter("Press [Enter] to close the hand.");
	mico.close_hand();
	
	//open again
	pressEnter("Press [Enter] to open the hand again.");
	mico.open_hand();
	
	//close just one finger
	pressEnter("Press [Enter] to close one finger.");
	mico.move_fingers(7500,100);
	
	//close again
	pressEnter("Press [Enter] to close the gripper and finish.");
	mico.close_hand();
	
	ros::shutdown();
}

