#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/JointState.h>
#include "bwi_perception/TabletopPerception.h"
#include <bwi_perception/bwi_perception.h>
#include <segbot_arm_manipulation/arm_utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <segbot_arm_manipulation/Mico.h>

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
	while (std::cin.get() != '\n')
		std::cout << "Please press ENTER\n";
}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "demo_obstacle_avoidance");
	
	ros::NodeHandle n;
    segbot_arm_manipulation::Mico mico(n);
	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	//Step 1: move the arm to the goal position
	pressEnter("Move the arm to the goal position and press 'Enter'");
	
	//update the arm's position
	mico.wait_for_data();
	
	//store the position as the goal
	geometry_msgs::PoseStamped goal_pose = mico.current_pose;
	
	//Step 2: move the arm to the start position
	pressEnter("Move the arm to the start position and press 'Enter'");
	//update the arm's position
	mico.wait_for_data();
	geometry_msgs::PoseStamped start_pose = mico.current_pose;
	
	
	while (ros::ok()){
	
		//Step 3: ask user to place an obstacle on the table between the two positions
		pressEnter("Place an obstacle and press 'Enter'");
		
		//Step 4: call tabletop perception service to detect the object and table
		bwi_perception::TabletopPerception::Response tabletop_response = bwi_perception::getTabletopScene(n);
		
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

		//now, move the arm to the goal -- it should avoid the obstacles
		bool resp = mico.move_to_pose_moveit(goal_pose, obstacle_clouds);
		
		//Step 3: ask user to place an obstacle on the table between the two positions
		pressEnter("Place or remove an obstacle and press 'Enter'");
		
		//Step 4: call tabletop perception service to detect the object and table
		tabletop_response = bwi_perception::getTabletopScene(n);
		
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
		bool resp2 = mico.move_to_pose_moveit(start_pose, obstacle_clouds);
	
	}
}
