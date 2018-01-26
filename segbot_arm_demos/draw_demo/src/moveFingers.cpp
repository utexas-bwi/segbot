#include <ros/ros.h>

#include <signal.h> 

#include <iostream>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include "kinova_msgs/SetFingersPositionAction.h"
#include <sensor_msgs/JointState.h>


using namespace std;

bool heard_efforts = false;
sensor_msgs::JointState current_efforts;
sensor_msgs::JointState last_efforts;
double total_grav_free_effort = 0;
double delta_effort[6];

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

bool recording = false;
int state = 1;
int max_num_points = 100;


void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};


int openFull(){
	actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> ac("/m1n6s200_driver/fingers/finger_positions/", true);
	kinova_msgs::SetFingersPositionGoal goal;
	goal.fingers.finger1 = 6;
	goal.fingers.finger2 = 6;
	goal.fingers.finger3 = 0;
	ac.waitForServer();
	ac.sendGoal(goal);
	ac.waitForResult();
 
}

int closeComplt(){
	actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> ac("/m1n6s200_driver/fingers/finger_positions/", true);
	kinova_msgs::SetFingersPositionGoal goal;
 	goal.fingers.finger1 = 7000;
	goal.fingers.finger2 = 7000;
	goal.fingers.finger3 = 0;
	ac.waitForServer();
	ac.sendGoal(goal);
	ac.waitForResult();
}

//method reads torque readings and will close grippers once something has been placed in the hand
int openAndClose(){
	
	ros::Time start = ros::Time::now();
	ros::Duration timeout = ros::Duration(10.0);
	ros::Rate r(10);
	
	//Open fingers.
	ROS_INFO("Opening fingers");
	openFull();
	
	while( (ros::Time::now() - start) < timeout){
		
		ros::spinOnce();
		if (heard_efforts){
			ROS_INFO("Total effort: %f",total_grav_free_effort);
			ROS_INFO("%f, %f, %f, %f, %f, %f",current_efforts.effort[0],current_efforts.effort[1],current_efforts.effort[2],
											current_efforts.effort[3],current_efforts.effort[4],current_efforts.effort[5]);
			
			ROS_INFO("%f, %f, %f, %f, %f, %f",delta_effort[0],delta_effort[1],delta_effort[2],
											delta_effort[3],delta_effort[4],delta_effort[5]);
											
			double total_delta = delta_effort[0]+delta_effort[1]+delta_effort[2]+delta_effort[3]+delta_effort[4]+delta_effort[5];
		
			ROS_INFO("%f",total_delta);
			
			/*if (total_grav_free_effort > 2.0){
				break;
			}*/
			
			if (total_delta > 0.1){
				ROS_INFO("Forces above 0.1N detected. Closing fingers.");
				closeComplt();
				total_delta = 0;
				heard_efforts = false;
				return 0;
			}
		} 
		r.sleep();
	}
}

//joint effort callback function
void joint_state_cb(const sensor_msgs::JointStateConstPtr& input){
	
	//compute the change in efforts if we had already heard the last one
	if (heard_efforts){
		for (int i = 0; i < 6; i ++){
			delta_effort[i] = input->effort[i]-current_efforts.effort[i];
		}
	}
	
	//store the current effort
	current_efforts = *input;
	
	total_grav_free_effort = 0.0;
	for (int i = 0; i < 6; i ++){
		if (current_efforts.effort[i] < 0.0)
			total_grav_free_effort -= (current_efforts.effort[i]);
		else 
			total_grav_free_effort += (current_efforts.effort[i]);
	}
	
	heard_efforts=true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "moveFingers");
	
	ros::NodeHandle n;
	char input;
	
	//create subscriber to joint torques
	ros::Subscriber sub_torques = n.subscribe ("/m1n6s200_driver/out/joint_state", 1, joint_state_cb);

	//register ctrl-c
	signal(SIGINT, sig_handler);	
	
 	
	ROS_INFO("I will open my fingers and wait for a user to put something between them.");
	while(ros::ok()){
		openAndClose();
		ROS_INFO("System now hangning on input. Input of a single char will cause another round of finger workout.");
		cin >> input;
	}


	return 0;
	
}
