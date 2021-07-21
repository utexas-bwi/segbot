#include <ros/ros.h>
#include <ros/package.h>

#include <signal.h>

#include <sensor_msgs/JointState.h>

//srv for talking to table_object_detection_node.cpp
#include "bwi_perception/PerceiveTabletopScene.h"

//action for grasping
#include "segbot_arm_manipulation/TabletopGraspAction.h"
#include "segbot_arm_manipulation/TabletopApproachAction.h"

#include <segbot_arm_manipulation/arm_positions_db.h>

#include <bwi_perception/bwi_perception.h>

//includes related to bwi_common
#include <move_base_msgs/MoveBaseAction.h>


#include <bwi_moveit_utils/MicoNavSafety.h>

#define NUM_JOINTS 8 //6+2 for the arm


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


void lift(ros::NodeHandle n, double x){
	mico->wait_for_data();
	
	geometry_msgs::PoseStamped p_target = current_pose;
	
	p_target.pose.position.z += x;
	mico->move_to_pose_moveit(p_target);
}


int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "object_to_office_task");
	
	ros::NodeHandle n;

	//create subscribers for arm topics
	ros::Subscriber sub_angles = n.subscribe ("/m1n6s200_driver/out/joint_state", 1, joint_state_cb);
	ros::Subscriber sub_tool = n.subscribe("/m1n6s200_driver/out/tool_pose", 1, toolpos_cb);

	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	//load database of joint- and tool-space positions
	std::string j_pos_filename = ros::package::getPath("segbot_arm_manipulation")+"/data/jointspace_position_db.txt";
	std::string c_pos_filename = ros::package::getPath("segbot_arm_manipulation")+"/data/toolspace_position_db.txt";
	
	ArmPositionDB positionDB(j_pos_filename, c_pos_filename);
	positionDB.print();
	
	//Step 1: store out-of-view position here
	sensor_msgs::JointState joint_state_outofview;
	geometry_msgs::PoseStamped pose_outofview;

	pressEnter("Please move the arm to out of view position...");
	
	//store out of table view joint position -- this is the position in which the arm is not occluding objects on the table
	mico->wait_for_data();
	joint_state_outofview = current_state;
	pose_outofview = current_pose;
	
	//Step 2: call safety service to make the arm safe for base movement -- TO DO
	bool safe = mico->make_safe_for_travel();
	if (!safe)
		return 1;
	
	//Step 3: issue a goal to move to the table in the pod -- for now, this is a hardcoded position in the map
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
	ac.waitForServer();
	
	//create the goal - currently hardcoded until go to object bug is fixed
	move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.pose.position.x = -31.3875554082;
    goal.target_pose.pose.position.y =  -3.73833188096;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.71660551689;
    goal.target_pose.pose.orientation.w = 0.697478697282;
    
	//send the goal and wait for result;
    ac.sendGoal(goal);
    ac.waitForResult();

	//Step 4: approach the table using visual servoing
	actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopApproachAction> ac_approach("segbot_table_approach_as",true);
		
	segbot_arm_manipulation::TabletopApproachGoal approach_goal;
	approach_goal.command = "approach";
	
	//send the goal
	ac_approach.sendGoal(approach_goal);
	ac_approach.waitForResult();
	
	
	//Step 6: move the arm out of the way
	mico->move_home();
	mico->move_to_joint_state(,joint_state_outofview);
	
	
	//Step 7: get the table scene and select object to grasp
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
		
	//Step 8: call the grasp action
	
	//create the action client
	actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopGraspAction> ac_grasp("segbot_tabletop_grasp_as",true);
	ac_grasp.waitForServer();
		
	//create and fill goal
	segbot_arm_manipulation::TabletopGraspGoal grasp_goal;
	grasp_goal.cloud_plane = table_scene.cloud_plane;
	grasp_goal.cloud_plane_coef = table_scene.cloud_plane_coef;
	for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
		grasp_goal.cloud_clusters.push_back(table_scene.cloud_clusters[i]);
	}
	grasp_goal.target_object_cluster_index = largest_pc_index;
	grasp_goal.action_name = segbot_arm_manipulation::TabletopGraspGoal::GRASP;
				
	//send the goal
	ROS_INFO("Sending goal to action server...");
	ac_grasp.sendGoal(grasp_goal);
	ac_grasp.waitForResult();
	ROS_INFO("Action Finished...");

	//next, lift and make arm safe again
	lift(n,0.05);
	safe = mico->make_safe_for_travel();
	if (!safe)
		return 1;
		
	//next, back out
	segbot_arm_manipulation::TabletopApproachGoal back_out_goal;
	back_out_goal.command = "back_out";
	ac_approach.sendGoal(back_out_goal);
	ac_approach.waitForResult();
	
	//next go to an office - TO DO
	
	
	//hand over object
	mico->move_to_pose_moveit(pose_outofview);
	
	segbot_arm_manipulation::TabletopGraspGoal handover_goal;
	handover_goal.action_name = segbot_arm_manipulation::TabletopGraspGoal::HANDOVER;
	handover_goal.timeout_seconds = 30.0;
	
	ac_grasp.sendGoal(grasp_goal);
	ac_grasp.waitForResult();
	

	
}
