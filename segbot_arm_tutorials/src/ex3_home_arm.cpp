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
#include <kinova_msgs/HomeArm.h>

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
	ros::init(argc, argv, "ex1_subscribing_to_topics");
	
	ros::NodeHandle n;
	
	//create subscribers for arm topics

	//register ctrl-c
	signal(SIGINT, sig_handler);

    MicoManager mico(n);

	//close fingers and "home" the arm
	pressEnter("Press [Enter] to close the gripper and home the arm");
	
	//close hand
	mico.close_hand();
	
	//home arm using service call to arm driver
	ros::ServiceClient home_client = n.serviceClient<kinova_msgs::HomeArm>("/m1n6s200_driver/in/home_arm");
	
	kinova_msgs::HomeArm srv;
	if(home_client.call(srv))
		ROS_INFO("Homing arm");
	else
		ROS_INFO("Cannot contact homing service. Is it running?");
	
	pressEnter("Press [Enter] to home the arm using the API call from segbot_arm_manipulation");
	
	//does the same thing but with less code
	mico.move_home();

	//the end
	ros::shutdown();
}
