#include <ros/ros.h>
#include <signal.h> 

#include <iostream>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include "kinova_msgs/SetFingersPositionAction.h"
#include <sensor_msgs/JointState.h>
#include "kinova_msgs/JointAngles.h"
#include "kinova_msgs/JointVelocity.h"
#include "kinova_msgs/PoseVelocity.h"
//subscriber msgs
#include <sensor_msgs/JointState.h>
#include <kinova_msgs/PoseVelocity.h>
#include <std_msgs/Float32.h>

//actions
#include <actionlib/client/simple_action_client.h>
#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"

ros::Publisher j_vel_pub;
ros::Publisher joint_state_pub;
ros::Publisher c_vel_pub_;

sensor_msgs::JointState current_jpos;

//efforts and force detection
bool heard_efforts = false;
sensor_msgs::JointState current_efforts;
sensor_msgs::JointState last_efforts;
double total_grav_free_effort = 0;
double total_delta;
double delta_effort[6];

int f1,f2;

void sig_handler(int sig){
	ROS_INFO("caugt sigint, init shutdown seq...");
	ros::shutdown();
	exit(1);
};

void clearMsgs(){
	ros::Time start = ros::Time::now();
	ros::Duration timeout = ros::Duration(.7);
	ros::Rate r2(5);
	//clears out old effort msgs
	while( (ros::Time::now() - start) < timeout){
		ros::spinOnce();
		r2.sleep();
	}
}

//checks fingers position - used for object holding assurance
void fingers_cb(const kinova_msgs::FingerPosition input){
	f1 = input.finger1;
	f2 = input.finger2;
}


//Joint state cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input)
{
	current_jpos = *input;
	
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
	
	//calc total change in efforts
	total_delta = delta_effort[0]+delta_effort[1]+delta_effort[2]+delta_effort[3]+delta_effort[4]+delta_effort[5];
	
	heard_efforts=true;
}

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
int openFingersAndWait(){
	
	ros::Time start = ros::Time::now();
	ros::Duration timeout = ros::Duration(60.0);
	ros::Rate r(20);
	
	//Open fingers.
	openFull();
	clearMsgs();
	while(true){
		ros::spinOnce();
		if (total_delta > 0.29){
			ROS_INFO("Forces above 0.29N detected. Closing fingers.");
			ROS_INFO("Efforts: %f", total_delta);
			closeComplt();
			
			break;
		}
		r.sleep();
	}
	
	//This checks if the fingers missed their target
	ros::spinOnce();
	std::cout << "Finger 1: " << f1 << " Finger 2: " << f2 << std::endl;
	
	return 0;
}

/*
 * Send a cartesian velocity command in xyz, no angular 
 */
void sendCartVelocity(double xin, double yin, double zin){
	double duration = 1.0;
	int rate_hz = 100;
	ros::Rate r(rate_hz);
	
	for (int i = 0; i < (int)(duration*rate_hz); i++){
		ros::spinOnce();
		
		kinova_msgs::PoseVelocity T; 
		T.twist_linear_x = xin;
		T.twist_linear_y = yin;
		T.twist_linear_z = zin;
		
		T.twist_angular_x=0.0;
		T.twist_angular_y=0.0;
		T.twist_angular_z=0.0;
		
		c_vel_pub_.publish(T);

		r.sleep();
	}
}

void sendJointVelocity(double xin){
	kinova_msgs::JointVelocity goal;
	goal.joint1 = -xin;
	goal.joint2 = xin;
	goal.joint3 = -xin;
	goal.joint4 = -xin;
	goal.joint5 = -xin;
	goal.joint6 = -xin;
	
	j_vel_pub.publish(goal);
}

// Blocking call for user input
void pressEnter(){
	std::cout << "Press the ENTER key to continue";
	while (std::cin.get() != '\n')
		std::cout << "Please press ENTER\n";
}

int main(int argc, char **argv)
{
	ros::init (argc, argv, "mico_movement_demo");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle n;
	signal(SIGINT, sig_handler);

	//publisher for joint velocity
    j_vel_pub = n.advertise<kinova_msgs::JointVelocity>("/m1n6s200_driver/in/joint_velocity", 1);

	//publisher for cartesian velocity
	c_vel_pub_ = n.advertise<kinova_msgs::PoseVelocity>("/m1n6s200_driver/in/cartesian_velocity", 2);

	//create subscriber to joint angles
	ros::Subscriber sub_angles = n.subscribe ("/m1n6s200_driver/out/joint_state", 1, joint_state_cb);
	
	//subscriber for fingers
  	ros::Subscriber sub_finger = n.subscribe("/m1n6s200_driver/out/finger_position", 1, fingers_cb);
  	
  	char in;
  	while(true){
		std::cout << "1 - Finger's Force Sensing" << std::endl;
		std::cout << "2 - Cartesian Vel. Cmd" << std::endl;
		std::cout << "3 - Joint Vel. Control" << std::endl;
		std::cout << "Enter a choice: ";
		std::cin >> in;
		
		if(in == '1'){
			openFingersAndWait();
		}
		else if(in == '2'){
			double x,y,z;
			std::cout << "Enter x: ";
			std::cin >> x;
			std::cout << "Enter y: ";
			std::cin >> y;
			std::cout << "Enter z: ";
			std:: cin >> z;
			sendCartVelocity(x,y,z);
		}
		else if(in == '3'){
			double x;
			std::cout << "Enter a joint vel. This will be applied to all joints: ";
			std::cin >> x;
			sendJointVelocity(x);
		}
		else
			std::cout << "Invalid, try again." << std::endl;
	}
  	
	ros::shutdown();
	return 0;
}
