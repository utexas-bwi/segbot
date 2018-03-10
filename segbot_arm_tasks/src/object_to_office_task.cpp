#include <ros/ros.h>
#include <ros/package.h>

#include <signal.h>

#include <sensor_msgs/JointState.h>

//srv for talking to table_object_detection_node.cpp
#include <bwi_perception/PerceiveTabletopScene.h>

//actions
#include "segbot_arm_manipulation/TabletopGraspAction.h"
#include "segbot_arm_manipulation/TabletopApproachAction.h"
#include "segbot_arm_manipulation/LiftVerifyAction.h"

#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/Mico.h>

#include <bwi_perception/bwi_perception.h>


#include "plan_execution/ExecutePlanAction.h"

#include <move_base_msgs/MoveBaseAction.h>


#include <segbot_arm_manipulation/HandoverAction.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

#define NUM_JOINTS 8 //6+2 for the arm


//true if Ctrl-C is pressed
bool g_caught_sigint=false;

segbot_arm_manipulation::Mico *mico;


/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};


/* Function to find the largest object on a table*/
int find_largest_obj(bwi_perception::PerceiveTabletopScene::Response table_scene) {
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

void go_to_place(std::string table){
	plan_execution::ExecutePlanGoal table_goal;
    plan_execution::AspRule table_rule;
    plan_execution::AspFluent table_fluent;
    
    table_fluent.name = "not facing";
    table_fluent.variables.push_back(table);
    
    table_rule.body.push_back(table_fluent);
    
    table_goal.aspGoal.push_back(table_rule);
	
	actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> table_asp("/action_executor/execute_plan", true);
	table_asp.waitForServer();
	
	ROS_INFO("finished waiting for server");
	
	//send the goal and wait
    ROS_INFO("sending goal");
    table_asp.sendGoalAndWait(table_goal);
    
    ROS_INFO("finished waiting for goal");
    
    //check for success
    if(table_asp.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_WARN("could not reach destination");
		exit(-1);
	}
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


void grasp_largest_object(ros::NodeHandle n, bwi_perception::PerceiveTabletopScene::Response table_scene,
                          int largest_pc_index) {

	
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
	grasp_goal.grasp_selection_method=segbot_arm_manipulation::TabletopGraspGoal::CLOSEST_ORIENTATION_SELECTION;
			
	//send the goal and wait
	ROS_INFO("Sending goal to grasp action server...");
	ac_grasp.sendGoal(grasp_goal);
	ac_grasp.waitForResult();
	ROS_INFO("Grasp action Finished...");
}

void handover_object(){
	actionlib::SimpleActionClient<segbot_arm_manipulation::HandoverAction> ac("segbot_handover_as",true);
	ac.waitForServer();
	ROS_INFO("handing the object over");
	
	segbot_arm_manipulation::HandoverGoal handover_goal;
	handover_goal.type = segbot_arm_manipulation::HandoverGoal::GIVE;
	handover_goal.timeout_seconds = 30.0;
	
	//send goal and wait for response
	ac.sendGoal(handover_goal);
	ac.waitForResult();
}

void lift_object(ros::NodeHandle n, bwi_perception::PerceiveTabletopScene::Response table_scene, int largest_pc_index) {
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
		mico->move_home();
		exit(1);
	}
	
}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "object_to_office_task");
	
	ros::NodeHandle n;

    mico = new segbot_arm_manipulation::Mico(n);
	//register ctrl-c
	signal(SIGINT, sig_handler);


	pressEnter("Press [ENTER] to proceed");
    
	mico->close_hand();
	mico->move_home();
	
	//Step 1: call safety service to make the arm safe for base movement
	bool safe = mico->make_safe_for_travel();
	if (!safe){
		ROS_WARN("the robot and arm cannot be made safe for travel");
		return 1;
	}
	ROS_INFO("safe for travel");
		
	pressEnter("Press [ENTER] to proceed");
    
    //Step 2: Create and send goal to move the robot to the table
    std::string table = "o3_414a_table";
	go_to_place(table);
	
    pressEnter("Press [ENTER] to approach table");

	//Step 3: approach the table using visual servoing
	call_approach("approach");

	pressEnter("Press [ENTER] to proceed");

	//Step 4: move the arm out of the way
	mico->move_home();
	mico->move_to_side_view();
	
	pressEnter("Press [ENTER] to proceed");
	
	//Step 5: get the table scene and select object to grasp
    bwi_perception::PerceiveTabletopScene::Response table_scene = bwi_perception::getTabletopScene(n);
	
	//ensure the plane is found and there are clusters on the table	
	if (!table_scene.is_plane_found){
		ROS_WARN("No plane found. Exiting...");
		exit(1);
	}else if ((int)table_scene.cloud_clusters.size() == 0){
		ROS_WARN("No objects found on table. The end...");
		exit(1);
	}
		
	//select the object with most points as the target object
	int largest_pc_index = find_largest_obj(table_scene);
		
	//Step 6: create and call the grasp action
	grasp_largest_object(n, table_scene, largest_pc_index);

	pressEnter("Press [ENTER] to proceed");

	//step 7: create and call action to lift and verify
	lift_object(n,table_scene, largest_pc_index);
	
	pressEnter("Press [ENTER] to proceed");

	mico->move_home();
	safe = mico->make_safe_for_travel();
	
	if (!safe){
		ROS_WARN("the robot and arm cannot be made safe for travel");
		return 1;
	}
	
	//Step 8: back out from the table with the object
	call_approach("back_out");


	//Step 9: bring the object to the office
	std::string location = "d3_516";
	
	go_to_place(location);

	pressEnter("Press [ENTER] to proceed to HANDOVER");
	
	mico->move_to_side_view();
		
	//Step 9: handover the object
	handover_object();
	
	mico->move_home();
	
	return 0;
}
