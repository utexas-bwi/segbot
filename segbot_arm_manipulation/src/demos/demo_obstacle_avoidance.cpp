#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/JointState.h>
#include "segbot_arm_perception/TabletopPerception.h"
#include <segbot_arm_manipulation/arm_utils.h>
#include <geometry_msgs/PoseStamped.h>

#define NUM_JOINTS 8 //6+2 for the arm

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
	while (std::cin.get() != '\n')
		std::cout << "Please press ENTER\n";
}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "demo_obstacle_avoidance");
	
	ros::NodeHandle n;

	//create subscriber to joint angles
	ros::Subscriber sub_angles = n.subscribe ("/m1n6s200_driver/out/joint_state", 1, joint_state_cb);
	
	//create subscriber to tool position topic
	ros::Subscriber sub_tool = n.subscribe("/m1n6s200_driver/out/tool_pose", 1, toolpos_cb);

	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	//Step 1: move the arm to the goal position
	pressEnter("Move the arm to the goal position and press 'Enter'");
	
	//update the arm's position
	listenForArmData();
	
	//store the position as the goal
	geometry_msgs::PoseStamped goal_pose = current_pose;
	
	//Step 2: move the arm to the start position
	pressEnter("Move the arm to the start position and press 'Enter'");
	//update the arm's position
	listenForArmData();
	geometry_msgs::PoseStamped start_pose = current_pose;
	
	
	while (ros::ok()){
	
		//Step 3: ask user to place an obstacle on the table between the two positions
		pressEnter("Place an obstacle and press 'Enter'");
		
		//Step 4: call tabletop perception service to detect the object and table
		segbot_arm_perception::TabletopPerception::Response tabletop_response = segbot_arm_manipulation::getTabletopScene(n);
		
		//check if plane was not found
		/*if (tabletop_response.is_plane_found == false){
			ROS_ERROR("[demo_obstalce_avoidance.cpp] Table not found...aborting.");
			ros::shutdown();
			exit(1);
		}*/
		
		ROS_INFO("Found %i objects on the table.",(int)tabletop_response.cloud_clusters.size());
		
		//Step 5: compute the vector of clouds to be used as obstacles and send them to the obstacle manager node
		std::vector<sensor_msgs::PointCloud2> obstacle_clouds;
		obstacle_clouds.push_back(tabletop_response.cloud_plane);
		for (unsigned int i = 0; i < tabletop_response.cloud_clusters.size(); i++){
			obstacle_clouds.push_back(tabletop_response.cloud_clusters.at(i));
		}
		
		segbot_arm_manipulation::setArmObstacles(n,obstacle_clouds);
		
		//now, move the arm to the goal -- it should avoid the obstacles
		moveit_utils::MicoMoveitCartesianPose::Response resp = segbot_arm_manipulation::moveToPoseMoveIt(n,goal_pose);
		
		//Step 3: ask user to place an obstacle on the table between the two positions
		pressEnter("Place or remove an obstacle and press 'Enter'");
		
		//Step 4: call tabletop perception service to detect the object and table
		tabletop_response = segbot_arm_manipulation::getTabletopScene(n);
		
		//check if plane was not found
		/*if (tabletop_response.is_plane_found == false){
			ROS_ERROR("[demo_obstalce_avoidance.cpp] Table not found...aborting.");
			ros::shutdown();
			exit(1);
		}*/
		
		ROS_INFO("Found %i objects on the table.",(int)tabletop_response.cloud_clusters.size());
		
		//Step 5: compute the vector of clouds to be used as obstacles and send them to the obstacle manager node
		obstacle_clouds.clear();
		obstacle_clouds.push_back(tabletop_response.cloud_plane);
		for (unsigned int i = 0; i < tabletop_response.cloud_clusters.size(); i++){
			obstacle_clouds.push_back(tabletop_response.cloud_clusters.at(i));
		}
		
		//now, move the arm to the goal -- it should avoid the obstacles
		moveit_utils::MicoMoveitCartesianPose::Response resp2 = segbot_arm_manipulation::moveToPoseMoveIt(n,start_pose);
		
	
	
	}
}
