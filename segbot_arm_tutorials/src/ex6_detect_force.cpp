#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinova_msgs/FingerPosition.h>


//our own arm library 
#include <segbot_arm_manipulation/arm_utils.h>
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
	ros::init(argc, argv, "ex6_detect_force");
	
	ros::NodeHandle n;

	MicoManager mico(n);
	 
	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	//open the hand
	mico.open_hand();
	int hand_state = 0; //0 for open, 1 for closed
	
	double force_threshold = 3.0;
	double timeout = 30.0;
	
	while (ros::ok()){
		ROS_INFO("Waiting for force...");
		bool force_detected = mico.wait_for_force(force_threshold, timeout);

		if (force_detected){
			if (hand_state == 0){
				mico.close_hand();
				hand_state = 1;
			}
			else {
				mico.open_hand();
				hand_state = 0;
			}	
		}	
	}
	
	ros::shutdown();
}
