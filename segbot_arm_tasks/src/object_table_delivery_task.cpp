#include <ros/ros.h>
#include <ros/package.h>

#include <signal.h>

#include <sensor_msgs/JointState.h>

//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"

//action for grasping
#include "segbot_arm_manipulation/TabletopGraspAction.h"
#include "segbot_arm_manipulation/TabletopApproachAction.h"
#include "segbot_arm_manipulation/LiftVerifyAction.h"

#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/arm_positions_db.h>

#include <segbot_arm_perception/segbot_arm_perception.h>

#include "bwi_kr_execution/ExecutePlanAction.h"

#include <move_base_msgs/MoveBaseAction.h>


#include <moveit_utils/MicoNavSafety.h>

#include "segbot_arm_manipulation/ObjReplacementAction.h"

#define NUM_JOINTS 8 //6+2 for the arm

//global variables for storing data
sensor_msgs::JointState current_state;
bool heardJointState;

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
		heardJointState = true;
	}
}

//Joint state cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
  current_pose = msg;
  heardPose = true;
}

void go_to_place(std::string table){
	bwi_kr_execution::ExecutePlanGoal table_goal;
    bwi_kr_execution::AspRule table_rule;
    bwi_kr_execution::AspFluent table_fluent;
    
    table_fluent.name = "not facing";
    table_fluent.variables.push_back(table);
    
    table_rule.body.push_back(table_fluent);
    
    table_goal.aspGoal.push_back(table_rule);
	
	actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> table_asp("/action_executor/execute_plan", true);
	table_asp.waitForServer();
	
	ROS_INFO("finished waiting for server");
	
	//send the goal and wait
    ROS_INFO("sending goal");
    table_asp.sendGoalAndWait(table_goal);
    
    ROS_INFO("finished waiting for goal");
    
    //check for success
    if(table_asp.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_WARN("could not reach destination");
		exit(1);
	}
}

int find_largest_obj(segbot_arm_perception::TabletopPerception::Response table_scene){
	int largest_pc_index = -1;
	int largest_num_points = -1;
	for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
			
		int num_points_i = table_scene.cloud_clusters[i].height* table_scene.cloud_clusters[i].width;
		
		if (num_points_i > largest_num_points){
			largest_num_points = num_points_i;
			largest_pc_index = i;
		}
	}
	return largest_pc_index;
}

void call_approach(std::string command){
	actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopApproachAction> ac_approach("segbot_table_approach_as",true);
	ac_approach.waitForServer();
	
	segbot_arm_manipulation::TabletopApproachGoal approach_goal;
	approach_goal.command = command;
	
	//send goal and wait
	ac_approach.sendGoal(approach_goal);
	ac_approach.waitForResult();
	ROS_INFO("finished tabletop approach action");
	
	//get result and check for failure
	segbot_arm_manipulation::TabletopApproachResult approach_result = *ac_approach.getResult();
	bool approach_success = approach_result.success; 
	if(!approach_success){
		ROS_WARN("tabletop approach action failed");
		ROS_INFO_STREAM(approach_result.error_msg);
		exit(1);
	}
}

void grasp_largest_object(ros::NodeHandle n, segbot_arm_perception::TabletopPerception::Response table_scene, int largest_pc_index){

	
	actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopGraspAction> ac_grasp("segbot_tabletop_grasp_as",true);
	ac_grasp.waitForServer();
		
	//create and fill goal for grasping
	segbot_arm_manipulation::TabletopGraspGoal grasp_goal;
	grasp_goal.cloud_plane = table_scene.cloud_plane;
	grasp_goal.cloud_plane_coef = table_scene.cloud_plane_coef;
	for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
		grasp_goal.cloud_clusters.push_back(table_scene.cloud_clusters[i]);
	}
	grasp_goal.target_object_cluster_index = largest_pc_index;
	grasp_goal.action_name = segbot_arm_manipulation::TabletopGraspGoal::GRASP;
	grasp_goal.grasp_selection_method=segbot_arm_manipulation::TabletopGraspGoal::CLOSEST_ORIENTATION_SELECTION;
			
	//send the goal and wait
	ROS_INFO("Sending goal to grasp action server...");
	ac_grasp.sendGoal(grasp_goal);
	ac_grasp.waitForResult();
	ROS_INFO("Grasp action Finished...");
}

void lift_object(ros::NodeHandle n, segbot_arm_perception::TabletopPerception::Response table_scene, int largest_pc_index){
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
	
	//check success of lift
	bool verified = result.success;
	if(verified){
		ROS_INFO("Verification succeeded.");
	}else{
		ROS_WARN("Verification failed");
		segbot_arm_manipulation::homeArm(n);
		exit(1);
	}
	
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "object_table_delivery_task"); 
	
	ros::NodeHandle n; 
	
	//create subscribers for arm topics
	ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);
	ros::Subscriber sub_tool = n.subscribe("/m1n6s200_driver/out/tool_position", 1, toolpos_cb);
	
	//register ctrl-c
	signal(SIGINT, sig_handler);

	//Step 1: make arm safe and go to table location
	segbot_arm_manipulation::closeHand();
	segbot_arm_manipulation::homeArm(n);
	bool safe = segbot_arm_manipulation::makeSafeForTravel(n);
	if (!safe){
		ROS_WARN("the robot and arm cannot be made safe for travel");
		exit(1);
	}
	
	//move to the starting table with desired object
	go_to_place("o3_514_tablea");

	//Step 2: approach table
	call_approach("approach");

	//Step 3: get the table scene and target object
	segbot_arm_manipulation::homeArm(n);
	segbot_arm_manipulation::arm_side_view(n);
	
	segbot_arm_perception::TabletopPerception::Response table_scene = segbot_arm_manipulation::getTabletopScene(n);
	if (!table_scene.is_plane_found){
		ROS_WARN("No plane found. Exiting...");
		exit(1);
	}else if ((int)table_scene.cloud_clusters.size() == 0){
		ROS_WARN("No objects found on table. The end...");
		exit(1);
	}
	int largest_pc_index = find_largest_obj(table_scene);
	
	
	//Step 4: grasp the target object
	grasp_largest_object(n, table_scene, largest_pc_index);

	//Step 5: lift and verify object
	lift_object(n, table_scene, largest_pc_index);

	//Step 6: make arm safe and go to goal table location
	segbot_arm_manipulation::closeHand();
	segbot_arm_manipulation::homeArm(n);
	safe = segbot_arm_manipulation::makeSafeForTravel(n);
	if (!safe){
		ROS_WARN("the robot and arm cannot be made safe for travel");
		exit(1);
	}
	
	call_approach("back_out");
	go_to_place("o3_514_tableb");

	//Step 7: approach table
	call_approach("approach");

	//Step 8: replace object on the new table
    actionlib::SimpleActionClient<segbot_arm_manipulation::ObjReplacementAction> replacement_ac("segbot_obj_replacement_as",true);
	replacement_ac.waitForServer();

    segbot_arm_manipulation::ObjReplacementGoal replace_goal;

    ROS_INFO("Sending replacement goal"); 
    replacement_ac.sendGoal(replace_goal);
	replacement_ac.waitForResult();
	ROS_INFO("finished replacement");

    segbot_arm_manipulation::ObjReplacementResult replace_result = *replacement_ac.getResult();
	bool verified = replace_result.success;
	if(verified){
		ROS_INFO("Replacement succeeded.");
	}else{
		ROS_WARN("Replacement failed");
		segbot_arm_manipulation::homeArm(n);
		exit(1);
	}

	segbot_arm_manipulation::homeArm(n);
	safe = segbot_arm_manipulation::makeSafeForTravel(n);
	if (!safe){
		ROS_WARN("the robot and arm cannot be made safe for travel");
		exit(1);
	}
	call_approach("back_out");
	 
}
