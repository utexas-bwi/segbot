#include <ros/ros.h>

#include <signal.h>

#include <sensor_msgs/JointState.h>

//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"

//srvs for ispy
#include "segbot_arm_manipulation/iSpyTouch.h"
#include "segbot_arm_manipulation/iSpyDetectTouch.h"
#include "segbot_arm_manipulation/iSpyFaceTable.h"

#include <segbot_arm_manipulation/arm_utils.h>

#define NUM_JOINTS 8 //6+2 for the arm

#define NUM_TABLE_OBJECTS 4

//global variables for storing data
sensor_msgs::JointState current_state;
bool heardJoinstState;

geometry_msgs::PoseStamped current_pose;
bool heardPose;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;


/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

//Joint state cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
	
	if (input->position.size() == NUM_JOINTS){
		current_state = *input;
		heardJoinstState = true;
	}
}


//Joint state cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
  current_pose = msg;
  heardPose = true;
}

//blocking call to listen for arm data (in this case, joint states)
void listenForArmData(){
	
	heardJoinstState = false;
	heardPose = false;
	ros::Rate r(10.0);
	
	while (ros::ok()){
		ros::spinOnce();
		
		if (heardJoinstState && heardPose)
			return;
		
		r.sleep();
	}
}


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


void lift(ros::NodeHandle n, double x){
	listenForArmData();
	
	geometry_msgs::PoseStamped p_target = current_pose;
	
	p_target.pose.position.z += x;
	segbot_arm_manipulation::moveToPoseMoveIt(n,p_target);
}

void goToSafePose(ros::NodeHandle n){
	geometry_msgs::PoseStamped pose_st;
	pose_st.header.stamp = ros::Time(0);
	pose_st.header.frame_id = "m1n6s200_link_base";
	
	
}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "test_ispy");
	
	ros::NodeHandle n;

	//create subscriber to joint angles
	ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);
	
	//create subscriber to tool position topic
	ros::Subscriber sub_tool = n.subscribe("/m1n6s200_driver/out/tool_position", 1, toolpos_cb);

	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	//store out-of-view position here
	sensor_msgs::JointState joint_state_outofview;
	geometry_msgs::PoseStamped pose_outofview;

	pressEnter("Demo starting...move the arm to a position where it is not occluding the table.");
	
	//store out of table joint position
	//listenForArmData();
	//joint_state_outofview = current_state;
	//pose_outofview = current_pose;
	
	//create clients
	ros::ServiceClient client_touch_object = n.serviceClient<segbot_arm_manipulation::iSpyTouch>("ispy/touch_object_service");
	
	while (ros::ok()){
	
		//get the table scene
		segbot_arm_perception::TabletopPerception::Response table_scene;
		bool perception_success = false;
		
		
		while (!perception_success){
			
			ROS_INFO("Waiting for service...");
			ros::service::waitForService("tabletop_object_detection_service");
			ROS_INFO("...done!");
			ROS_INFO("Waiting for service existg...");
			bool result = ros::service::exists("tabletop_object_detection_service",true);
			ROS_INFO("...done!");
			std::cout << result << "\n";
			
			ros::ServiceClient client_tabletop_perception = n.serviceClient<segbot_arm_perception::TabletopPerception>("tabletop_object_detection_service");
				
				
			segbot_arm_perception::TabletopPerception srv_perception; 
				
			//to make sure we only see objects on the table in front of us
			srv_perception.request.apply_x_box_filter = true;
			srv_perception.request.x_min = -0.3;
			srv_perception.request.x_max = 0.8;
				
			ROS_INFO("Calling perception!");
			if (client_tabletop_perception.call(srv_perception))
			{
				table_scene = srv_perception.response;
			}
			else
			{
				ROS_ERROR("Failed to call service tabletop_object_detection_service");
				exit(1);
			}
				
				
				
			if ((int)table_scene.cloud_clusters.size() == 0){
				ROS_WARN("No objects found on table. The end...");
				exit(1);
			}
			else if ((int)table_scene.cloud_clusters.size() == NUM_TABLE_OBJECTS){
				ROS_INFO("Correct number of objects seen!");
				perception_success = true;
			}
		}	
	
		
		//test pointing to all objects on the table
		for (int i = 0; i < (int)table_scene.cloud_clusters.size(); i++){
			
			//pressEnter("Press 'Enter' to touch next object.");
			
			//create srv with scene and target object
			segbot_arm_manipulation::iSpyTouch srv;
			srv.request.objects = table_scene.cloud_clusters;
			srv.request.cloud_plane_coef = table_scene.cloud_plane_coef;
			srv.request.cloud_plane = table_scene.cloud_plane;
			
			//target object
			srv.request.touch_index = i;
			
			//touch object
			if (client_touch_object.call(srv))
			{
				ROS_INFO("Touch completed.");
			}
			else
			{
				ROS_ERROR("Failed to call service ");
				return 0;
			}
			
			//retract
			/*	return 0;
			}*/
		}
		
		
		//create srv with scene and target object
		segbot_arm_manipulation::iSpyTouch srv;
		srv.request.objects = table_scene.cloud_clusters;
		srv.request.cloud_plane_coef = table_scene.cloud_plane_coef;
		srv.request.cloud_plane = table_scene.cloud_plane;
		srv.request.touch_index = -1;
		if (client_touch_object.call(srv))
		{
			ROS_INFO("Retract completed.");
		}
		else
		{
			ROS_ERROR("Failed to call service ");
		}
		
		
		pressEnter("Press 'Enter' to grasp again or 'q' to quit.");
	}
	
	
}
