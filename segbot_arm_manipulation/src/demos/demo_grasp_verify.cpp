#include <ros/ros.h>

#include <signal.h>

#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <segbot_arm_manipulation/Mico.h>

//srv for talking to table_object_detection_node.cpp
#include "bwi_perception/bwi_perception.h"

//actions
#include "segbot_arm_manipulation/TabletopGraspAction.h"
#include "segbot_arm_manipulation/LiftVerifyAction.h"


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
	ros::init(argc, argv, "demo_grasp_action_client");
	
	ros::NodeHandle n;

    segbot_arm_manipulation::Mico mico(n);
	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	//store out-of-view position here
	sensor_msgs::JointState joint_state_outofview;
	geometry_msgs::PoseStamped pose_outofview;

	pressEnter("Demo starting...move the arm to a position where it is not occluding the table.");
	
	//store out of table joint position
	mico.wait_for_data();
	joint_state_outofview = mico.current_state;
	pose_outofview = mico.current_pose;
	

	while (ros::ok()){
	
		//get the table scene
		bwi_perception::PerceiveTabletopScene::Response table_scene = bwi_perception::getTabletopScene(n);
		
		if ((int)table_scene.cloud_clusters.size() == 0){
			ROS_WARN("No objects found on table. The end...");
			exit(1);
		}
		
		//select the object with most points as the target object
		int largest_pc_index = -1;
		int largest_num_points = -1;
		for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
			
			int num_points_i = table_scene.cloud_clusters[i].height* table_scene.cloud_clusters[i].width;
		
			if (num_points_i > largest_num_points){
				largest_num_points = num_points_i;
				largest_pc_index = i;
			}
		}
		
		//create the action client
		actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopGraspAction> ac("segbot_tabletop_grasp_as",true);
		ac.waitForServer();
		ROS_INFO("action server made...");
		
		//create and fill goal
		segbot_arm_manipulation::TabletopGraspGoal grasp_goal;
		
		//for that action, we have to specify the method used for picking the target grasp out of the candidates
		grasp_goal.grasp_selection_method=segbot_arm_manipulation::TabletopGraspGoal::CLOSEST_ORIENTATION_SELECTION;
		
		//finally, we fill in the table scene
		grasp_goal.cloud_plane = table_scene.cloud_plane;
		grasp_goal.cloud_plane_coef = table_scene.cloud_plane_coef;
		for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
			grasp_goal.cloud_clusters.push_back(table_scene.cloud_clusters[i]);
		}
		grasp_goal.target_object_cluster_index = largest_pc_index;
				
		//send the goal
		ROS_INFO("Sending goal to action server...");
		ac.sendGoal(grasp_goal);
		
		//block until the action is completed
		ROS_INFO("Waiting for result...");
		
		ac.waitForResult();
		ROS_INFO("Action Finished...");


		//action to lift and verify
		actionlib::SimpleActionClient<segbot_arm_manipulation::LiftVerifyAction> lift_ac("arm_lift_verify_as", true);
		lift_ac.waitForServer();
		ROS_INFO("lift and verify action server made...");
		
		//make goals to send to action
		segbot_arm_manipulation::LiftVerifyGoal lift_verify_goal;
		lift_verify_goal.tgt_cloud = table_scene.cloud_clusters[largest_pc_index];
		lift_verify_goal.bins = 8;
		
		ROS_INFO("sending goal to lift and verify action server...");
		lift_ac.sendGoal(lift_verify_goal);
		
		ROS_INFO("waiting for lift and verify action server result....");
		lift_ac.waitForResult();
		
		ROS_INFO("lift and verify action finished.");
		segbot_arm_manipulation::LiftVerifyResult result = *lift_ac.getResult();
		
		//display success of action to user
		bool verified = result.success;
		if(verified){
			ROS_INFO("Verification succeeded.");
		}else{
			ROS_WARN("Verification failed");
		}
		
	
		pressEnter("Press 'Enter' to grasp again or 'q' to quit.");
	}
}
