#include <ros/ros.h>
#include <ros/package.h>

#include <signal.h>

#include <sensor_msgs/JointState.h>

#include <geometry_msgs/PoseStamped.h>

#include <kinova_msgs/FingerPosition.h>

#define NUM_JOINTS 8 //6+2 for the arm

//global variables for storing sensory data
sensor_msgs::JointState current_state;
geometry_msgs::PoseStamped current_pose;
kinova_msgs::FingerPosition current_finger;


bool heardJointState;
bool heardPose;
bool heardFingers;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
	g_caught_sigint = true;
	ROS_INFO("caught sigint, init shutdown sequence...");
	ros::shutdown();
	exit(1);
};

//Joint positions cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& msg) {
	
	if (msg->position.size() == NUM_JOINTS){
		current_state = *msg;
		heardJointState = true;
        ROS_INFO("Heard arm pose");
	}
}

//tool pose cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
	current_pose = msg;
	heardPose = true;
    ROS_INFO("Heard tool pose");
}

//fingers state cb
void fingers_cb (const kinova_msgs::FingerPositionConstPtr& msg) {
	current_finger = *msg;
	heardFingers = true;
	ROS_INFO("Heard finger state");
}

//blocking call to listen for arm data (in this case, joint states)
void listenForArmData(){
	
	heardJointState = false;
	heardPose = false;
	heardFingers = false;
	
	ros::Rate r(40.0);
	
	while (ros::ok()){
		ros::spinOnce();	
		
		if (heardJointState && heardPose && heardFingers)
			return;
		
		r.sleep();
	}
}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "ex1_subscribing_to_topics");
	
	ros::NodeHandle n;
	
	//create subscribers for arm topics
	
	//joint positions and efforts (AKA haptics)
	ros::Subscriber sub_angles = n.subscribe ("/m1n6s200_driver/out/joint_state", 1, joint_state_cb);
	
	//cartesean tool position and orientation
	ros::Subscriber sub_tool = n.subscribe("/m1n6s200_driver/out/tool_pose", 1, toolpos_cb);

	//finger positions
	ros::Subscriber sub_finger = n.subscribe("/m1n6s200_driver/out/finger_position", 1, fingers_cb);
	 
	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	//listen for arm data
	listenForArmData();
	
	ROS_INFO("Current joint positions and efforts:");
	ROS_INFO_STREAM(current_state);
	
	ROS_INFO("Current finger positions:");
	ROS_INFO_STREAM(current_finger);
	
	ROS_INFO("Current tool pose:");
	ROS_INFO_STREAM(current_pose);
	
	
	ros::shutdown();
}
