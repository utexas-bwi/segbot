#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>

#include <sensor_msgs/JointState.h>
#include <kinova_msgs/PoseVelocity.h>
#include <std_msgs/Float32.h>

//actions
#include <actionlib/client/simple_action_client.h>
#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"

#include <tf/tf.h>

//moveit interface service
#include "moveit_utils/MicoMoveitCartesianPose.h"

#define PI 3.14159265

using namespace std;

#define FINGER_FULLY_OPENED 6
#define FINGER_FULLY_CLOSED 7300


sensor_msgs::JointState current_state;
kinova_msgs::FingerPosition current_finger;
geometry_msgs::PoseStamped current_pose;

ros::Publisher pub_velocity;
ros::ServiceClient moveit_client;

bool button_pose_received = false;
geometry_msgs::PoseStamped current_button_pose;
 
//Joint state cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
  current_state = *input;
  //ROS_INFO_STREAM(current_state);
}

//Joint state cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
  current_pose = msg;
  //  ROS_INFO_STREAM(current_pose);
}

//Joint state cb
void fingers_cb (const kinova_msgs::FingerPosition msg) {
  current_finger = msg;
}


void movePose(float d_z) {
  actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ac("/m1n6s200_driver/pose_action/tool_pose", true);

  kinova_msgs::ArmPoseGoal goalPose;

  // Set goal pose coordinates

  goalPose.pose.header.frame_id = "m1n6s200_link_base";
  
  ROS_INFO_STREAM(current_pose);

  goalPose.pose.pose.position.x = current_pose.pose.position.x;
  goalPose.pose.pose.position.y = current_pose.pose.position.y;
  goalPose.pose.pose.position.z = current_pose.pose.position.z + d_z;
  goalPose.pose.pose.orientation.x = current_pose.pose.orientation.x;
  goalPose.pose.pose.orientation.y = current_pose.pose.orientation.y;
  goalPose.pose.pose.orientation.z = current_pose.pose.orientation.z;
  goalPose.pose.pose.orientation.w = current_pose.pose.orientation.w;

	ROS_INFO_STREAM(goalPose);

  ac.waitForServer();
  ROS_INFO("Waiting for server.");
  //finally, send goal and wait
  ROS_INFO("Sending goal.");
  ac.sendGoal(goalPose);
  ac.waitForResult();

}

// Range = [6, 7300] ([open, close])
void moveFinger(int finger_value) {
    actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> ac("/m1n6s200_driver/fingers_action/finger_positions", true);

    kinova_msgs::SetFingersPositionGoal goalFinger;

    goalFinger.fingers.finger1 = finger_value;
    goalFinger.fingers.finger2 = finger_value;
    // Not used for our arm
    goalFinger.fingers.finger3 = 0;
    
    ac.waitForServer();

    ac.sendGoal(goalFinger);

    ac.waitForResult();
}

void graspObject() {
	double timeoutSeconds = 1.75;
	int rateHertz = 100;
	kinova_msgs::PoseVelocity velocityMsg;
	
	double linearAngleX = 0;
	double linearVelX;
	double linearAngleZ = 0;
	double linearVelZ;
	double magnitude = 0.2;
	
	ros::Rate r(rateHertz);
	for(int i = 0; i < (int)timeoutSeconds * rateHertz; i++) {
		
		
		velocityMsg.twist_linear_x = 0;
		velocityMsg.twist_linear_y = 0.0;
		velocityMsg.twist_linear_z = -0.125;
		
		velocityMsg.twist_angular_x = 0.0;
		velocityMsg.twist_angular_y = 0.0;
		velocityMsg.twist_angular_z = 0.0;
		
		
		pub_velocity.publish(velocityMsg);
		
		r.sleep();
	}
	

	for(int i = 0; i < (int)3.0 * rateHertz; i++) {
		
		
		velocityMsg.twist_linear_x = -0.13;
		velocityMsg.twist_linear_y = 0.0;
		velocityMsg.twist_linear_z = 0.1;
		
		velocityMsg.twist_angular_x = 0.0;
		velocityMsg.twist_angular_y = 0.0;
		velocityMsg.twist_angular_z = 0.0;
		
		
		pub_velocity.publish(velocityMsg);
		
		r.sleep();
	}
}


void moveToSideOfObject(){
	
	//actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ac("/m1n6s200_driver/pose_action/tool_pose", true);
	geometry_msgs::PoseStamped goalPose;
	
	// Set goal pose coordinates

	goalPose.header.frame_id = "m1n6s200_link_base";
     
     // Points for the side grasp
    goalPose.pose.position.x = 0.319440335035;
	goalPose.pose.position.y = -0.332632094622;
	goalPose.pose.position.z = -0.0221896357834;
	
	goalPose.pose.orientation.x = 0.464970425525;
	goalPose.pose.orientation.y = 0.521655124743;
	goalPose.pose.orientation.z = 0.501337492214;
	goalPose.pose.orientation.w = 0.51023440997;
	
 	/*goalPose.pose.position.x = current_button_pose.pose.position.x+0.09;
	goalPose.pose.position.y = current_button_pose.pose.position.y-0.03;
	goalPose.pose.position.z = current_button_pose.pose.position.z+0.125;*/
	
	//goalPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14,3.14/1.9,0);
	
	moveit_utils::MicoMoveitCartesianPose srv;
	srv.request.target = goalPose;
	if(moveit_client.call(srv))
		ROS_INFO("Called IK interface service.");
	else
		ROS_INFO("Service call to IK interface failed, is it running?");
	
	ROS_INFO_STREAM(goalPose);	
	
  /*ac.waitForServer();
  ROS_DEBUG("Waiting for server.");
  //finally, send goal and wait
  ROS_INFO("Sending goal.");
  ac.sendGoal(goalPose);
  ac.waitForResult();*/
}

//button state cb
void button_pose_cb (const geometry_msgs::PoseStamped &msg) {
  current_button_pose = msg;
  button_pose_received = true;
}

void waitForButtonPose(){
	ros::Rate r(10);
	while (!button_pose_received){
		r.sleep();
		ros::spinOnce();
	}
}

int main(int argc, char **argv) {
  // Intialize ROS with this node name
  ros::init(argc, argv, "mimic_motion");

  ros::NodeHandle n;

  //create subscriber to joint angles
  ros::Subscriber sub_angles = n.subscribe ("/m1n6s200_driver/out/joint_state", 1, joint_state_cb);

  //create subscriber to tool position topic
  ros::Subscriber sub_tool = n.subscribe("/m1n6s200_driver/out/tool_pose", 1, toolpos_cb);

  //subscriber for fingers
  ros::Subscriber sub_finger = n.subscribe("/m1n6s200_driver/out/finger_position", 1, fingers_cb);
  
  //create subscriber for button position
  ros::Subscriber sub_button = n.subscribe("/press_object_specific_position/pose", 1, button_pose_cb);

  //publish velocities
  pub_velocity = n.advertise<kinova_msgs::PoseVelocity>("/m1n6s200_driver/in/cartesian_velocity", 10);
	
	moveit_client = n.serviceClient<moveit_utils::MicoMoveitCartesianPose>("mico_cartesianpose_service");
	//Step 1: listen to the button pose
	waitForButtonPose();
	
	ROS_INFO("Closest Object detected at: %f, %f, %f",current_button_pose.pose.position.x,current_button_pose.pose.position.y,current_button_pose.pose.position.z);

	//Step 2: open fingers
	moveFinger(6);
	
	//Step 3: move above the button
	moveToSideOfObject();
	
	//graspObject();

  /*unsigned int finger_open_close_toggle = 0;
  while (ros::ok()) {
      ros::spinOnce();
      ros::Duration(5).sleep();
      ROS_INFO("Iteration = %d", finger_open_close_toggle);
      if (finger_open_close_toggle % 2 == 0) {
          moveFinger(FINGER_FULLY_OPENED);
          ROS_INFO("Opening...");
      } else {
          moveFinger(FINGER_FULLY_CLOSED);
          ROS_INFO("Closing...");
      }
      finger_open_close_toggle++;
  }*/
  
  /*for (int i = 0; i < 20; i ++){
	  ros::spinOnce();
      ros::Duration(0.05).sleep();
  }*/
  
  //movePose(0.05);
  
   /*for (int i = 0; i < 20; i ++){
	  ros::spinOnce();
      ros::Duration(0.05).sleep();
  }*/
  
  //moveArmVelocity();
  //movePose(-0.05);
//  moveFinger(int finger_value);
  return 0;
}
