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
}


//Joint state cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
  current_pose = msg;
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
    ROS_INFO("In moveFinger: %d", finger_value);
    actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> ac("/m1n6s200_driver/fingers_action/finger_positions", true);

    kinova_msgs::SetFingersPositionGoal goalFinger;

    goalFinger.fingers.finger1 = finger_value;
    goalFinger.fingers.finger2 = finger_value;
    // Not used for our arm
    goalFinger.fingers.finger3 = 0;
    
    ac.waitForServer();

	ac.sendGoal(goalFinger);
    //ac.sendGoal(goalFinger,ros::Duration(10.0),ros::Duration(10.0));

    ac.waitForResult(ros::Duration(10.0));
    
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
}

/*void graspObject(){
	actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> ac("/m1n6s200_driver/fingers_action/finger_positions", true);
	kinova_msgs::SetFingersPositionGoal goalFinger;
	float before_finger1 = 0;
	float before_finger2 = 0;	
	float before1_finger1, before1_finger2;	
	
	while(true){
			
			before1_finger1  = before_finger1;
			before1_finger2 = before_finger2;
			ros::spinOnce();
			before_finger1 = current_finger.finger1;
			before_finger2 = current_finger.finger2;
			
			ROS_INFO("before_finger1: %f", before_finger1);
			ROS_INFO("before_finger2: %f", before_finger2);
			ROS_INFO("before1_finger1: %f", before1_finger1);
			ROS_INFO("before1_finger2: %f", before1_finger2);
	
			if((before_finger1 - before1_finger1 > 2) && (before_finger2 - before1_finger2 > 2)){
				
				goalFinger.fingers.finger1 = (int)(before_finger1+40.0);
				goalFinger.fingers.finger2 = (int)(before_finger2+40.0);
				// Not used for our arm
				goalFinger.fingers.finger3 = 0;
				ac.waitForServer();

				ac.sendGoal(goalFinger);

				ac.waitForResult();
		}
		else
			break;
	}
}
*/
void liftObject() {
	ROS_INFO("In liftObject");
	double timeoutSeconds = 2.00;
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
		velocityMsg.twist_linear_z = 0.125;
		
		velocityMsg.twist_angular_x = 0.0;
		velocityMsg.twist_angular_y = 0.0;
		velocityMsg.twist_angular_z = 0.0;
		
		pub_velocity.publish(velocityMsg);
		
		r.sleep();
	}
	

	for(int i = 0; i < (int)timeoutSeconds * rateHertz; i++) {
		velocityMsg.twist_linear_x = 0.0;
		velocityMsg.twist_linear_y = 0.0;
		velocityMsg.twist_linear_z = -0.125;
		
		velocityMsg.twist_angular_x = 0.0;
		velocityMsg.twist_angular_y = 0.0;
		velocityMsg.twist_angular_z = 0.0;
		
		pub_velocity.publish(velocityMsg);
		
		r.sleep();
	}
}

void goBackToRestPosition(){
	geometry_msgs::PoseStamped goalPose;
	goalPose.header.frame_id = "m1n6s200_link_base";
	
	goalPose.pose.position.x = 0.210980892181;
	goalPose.pose.position.y = -0.262371391058;
	goalPose.pose.position.z = 0.478665828705;
	
	goalPose.pose.orientation.x =  0.582317142899;
	goalPose.pose.orientation.y = 0.392099700782;
	goalPose.pose.orientation.z =  0.374113192129;
	goalPose.pose.orientation.w =  0.605973505368;

	moveit_utils::MicoMoveitCartesianPose srv;
	srv.request.target = goalPose;
	if(moveit_client.call(srv))
		ROS_INFO("Called IK interface service.");
	else
		ROS_INFO("Service call to IK interface failed, is it running?");
	
	ROS_INFO_STREAM(goalPose);
}

void moveToTopOfObject(){
	// Specific object position: (0.263455903, -0.365234429, -0.043787225)
	//actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ac("/m1n6s200_driver/pose_action/tool_pose", true);
	geometry_msgs::PoseStamped goalPose;
	
	// Set goal pose coordinates

	goalPose.header.frame_id = "m1n6s200_link_base";
     
	// points for the top grasp
	/*goalPose.pose.position.x = 0.28510850668;
	goalPose.pose.position.y = -0.349182128906;
	goalPose.pose.position.z = -0.0154058495536;*/
	
	goalPose.pose.position.x = current_button_pose.pose.position.x+0.04252604;
	goalPose.pose.position.y = current_button_pose.pose.position.y+0.0060523;
	goalPose.pose.position.z = current_button_pose.pose.position.z+0.028381375;
	
	goalPose.pose.orientation.x = 0.707466353222;
	goalPose.pose.orientation.y = 0.7055341541675;
	goalPose.pose.orientation.z = -0.0316366919845;
	goalPose.pose.orientation.w = 0.026684004027;
	
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
  ros::Subscriber sub_button = n.subscribe("/top_grasp_node/pose", 1, button_pose_cb);

  //publish velocities
  pub_velocity = n.advertise<kinova_msgs::PoseVelocity>("/m1n6s200_driver/in/cartesian_velocity", 10);
	
  moveit_client = n.serviceClient<moveit_utils::MicoMoveitCartesianPose>("mico_cartesianpose_service");
  //Step 1: listen to the button pose
  waitForButtonPose();
	
  ROS_INFO("Closest Object detected at: %f, %f, %f",current_button_pose.pose.position.x,current_button_pose.pose.position.y,current_button_pose.pose.position.z);

  //Step 2: open fingers
  moveFinger(6);
	
  //Step 3: move above the button
  moveToTopOfObject();

  //Step 4: Close fingers
  moveFinger(7300);
  moveFinger(7300);
  
  int rateHertz = 1;
  ros::Rate r(rateHertz);
  r.sleep();
	
  //Step 5: Lift object
  liftObject();
	
  //Step 6: Release object
  moveFinger(6);

  //Step 7: Go back to the rest position
  goBackToRestPosition();

  return 0;
}
