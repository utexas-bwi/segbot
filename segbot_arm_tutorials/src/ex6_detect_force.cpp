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


/*
 * blocks until force of sufficient amount is detected or timeout is exceeded
 * returns true of force degected, false if timeout
 */
bool waitForForce(MicoManager &mico, const double force_threshold, const double timeout){
	double rate = 40.0;
	ros::Rate r(rate);

	double total_delta;

	mico.wait_for_data();
	//make a copy and store it as the previous effort state	
	sensor_msgs::JointState prev_effort_state = mico.current_state;

	double elapsed_time = 0;

	while (ros::ok()){
		
		ros::spinOnce();
				
		total_delta=0.0;
		for (int i = 0; i < 6; i ++){
			total_delta += fabs(mico.current_state.effort[i]-prev_effort_state.effort[i]);
		}
		
		//ROS_INFO("Total delta=%f",total_delta);
				
		if (total_delta > fabs(force_threshold)){
			ROS_INFO("Force detected");
			return true;	
		}
				
		r.sleep();
		elapsed_time+=(1.0)/rate;
				
		if (elapsed_time > timeout){		
			ROS_WARN("Wait for force function timed out");
			return false;
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
		bool force_detected = waitForForce(mico, force_threshold, timeout);
		
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
