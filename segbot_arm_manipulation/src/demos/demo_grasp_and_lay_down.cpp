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
#include <bwi_manipulation/GraspCartesianCommand.h>

#include <move_base_msgs/MoveBaseAction.h>


#include <segbot_arm_manipulation/HandoverAction.h>
#include <bwi_perception/convenience.h>


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

#define NUM_JOINTS 8 //6+2 for the arm

using namespace std;

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

/* Function to find the smallest object on a table*/
int find_smallest_obj(bwi_perception::PerceiveTabletopScene::Response table_scene) {
	int smallest_pc_index = -1;
	int smallest_num_points = numeric_limits<int>::max();
	for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){

		int num_points_i = table_scene.cloud_clusters[i].height* table_scene.cloud_clusters[i].width;

		if (num_points_i < smallest_num_points){
			smallest_num_points = num_points_i;
			smallest_pc_index = i;
		}
	}
	return smallest_pc_index;
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


void call_approach(std::string command){	actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopApproachAction> ac_approach("segbot_table_approach_as",true);
	ac_approach.waitForServer();

	segbot_arm_manipulation::TabletopApproachGoal approach_goal;
	approach_goal.command = command;
	approach_goal.table_type = "circular";

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



bwi_manipulation::GraspCartesianCommand grasp_object_index(ros::NodeHandle n,
														   bwi_perception::PerceiveTabletopScene::Response table_scene,
														   int largest_pc_index){


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
	grasp_goal.grasp_selection_method=segbot_arm_manipulation::TabletopGraspGoal::CLOSEST_TO_CENTROID_SELECTION;
	grasp_goal.grasp_generation_method = segbot_arm_manipulation::TabletopGraspGoal::HEURISTIC;

	//send the goal and wait
	ROS_INFO("Sending goal to grasp action server...");
	ac_grasp.sendGoal(grasp_goal);
	ac_grasp.waitForResult();
	auto result=ac_grasp.getResult();




	bwi_manipulation::GraspCartesianCommand laydown_pose;
	laydown_pose.approach_pose=result->approach_pose;
	laydown_pose.approach_joint_state=result->approach_joint_state;
	laydown_pose.grasp_pose=result->grasp_pose;
	laydown_pose.grasp_joint_state=result->grasp_joint_state;

	ROS_INFO("Grasp action Finished...");
	return laydown_pose;
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


//void lay_down_object(){
//	actionlib::SimpleActionClient<segbot_arm_manipulation::LayDownAction> ac("segbot_laydown_as",true);
//	ac.waitForServer();
//	ROS_INFO("Laying the object down on the table");
//
//	segbot_arm_manipulation::LaydownGoal laydown_goal;
//	laydown_goal.timeout_seconds = 30.0;
//
//	//send goal and wait for response
//	ac.sendGoal(laydown_goal);
//	ac.waitForResult();
//}

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
	ros::init(argc, argv, "grasp_and_lay_down");

	ros::NodeHandle n;

	mico = new segbot_arm_manipulation::Mico(n);
	//register ctrl-c
	signal(SIGINT, sig_handler);


	ROS_INFO("Starting grasp and lay down task");
	ROS_INFO("Moving arm out of view");
	mico->open_hand();
	mico->close_hand();

//    mico->move_home();
//	mico->position_db->print();
//    mico->move_to_side_view_approach();
//    mico->move_to_side_view_approach();

	mico->move_to_side_view();
	mico->move_to_side_view();
	//   mico->move_home();

//	pressEnter("Press [ENTER] when safe to proceed");

//This is where we should get the input of the task description:
// What to take, from where , to where

//Step 2: Create and send goal to move the robot to the table
//testing index delete later:
    int item_index=0;

//	//Step 5: get the table scene and select object to grasp
	ROS_INFO("Perceiving the table and objects");
	bwi_perception::PerceiveTabletopScene::Response table_scene = bwi_perception::getTabletopScene(n);
//
//	//ensure the plane is found and there are clusters on the table
	if (!table_scene.is_plane_found){
		ROS_WARN("No plane found. Exiting...");
		exit(1);
	}
	else if ((int)table_scene.cloud_clusters.size() == 0){
		ROS_WARN("No objects found on table. The end...");
		exit(1);
	}
	ROS_INFO("Found %d objects on the table",(int)table_scene.cloud_clusters.size());
//	//select the object with  as the target object
	ROS_INFO("Selecting object index %d on the table", item_index);
//	int selected_pc_index = find_largest_obj(table_scene);
	//int selected_pc_index = find_smallest_obj(table_scene);

	// Convert objects to PCL type
	vector<PointCloudT::Ptr> as_pcl_clouds;
	/*transform(table_scene.cloud_clusters.begin(), table_scene.cloud_clusters.end(), as_pcl_clouds.begin(), [](sensor_msgs::PointCloud2 as_msg){
        PointCloudT::Ptr converted(new PointCloudT);
        pcl::fromROSMsg(as_msg, *converted);
        return converted;
    });*/

	tf::TransformListener listener;
	listener.waitForTransform("arm_link", table_scene.cloud_clusters.at(0).header.frame_id, ros::Time::now(), ros::Duration(130));
	for (const auto &as_msg: table_scene.cloud_clusters) {
		PointCloudT::Ptr converted(new PointCloudT);
		pcl::fromROSMsg(as_msg, *converted);

		pcl_ros::transformPointCloud("arm_link", *converted, *converted, listener);
		as_pcl_clouds.emplace_back(converted);
	}

	bwi_perception::order_clouds<PointT>(as_pcl_clouds, bwi_perception::Axis::Y);

	/*transform(as_pcl_clouds.begin(), as_pcl_clouds.end(), table_scene.cloud_clusters.begin(), [](PointCloudT::Ptr as_pcl){
        sensor_msgs::PointCloud2 converted;
        pcl::toROSMsg(*as_pcl, converted);
        return converted;
    });*/

	table_scene.cloud_clusters.clear();
	for (const auto &as_pcl: as_pcl_clouds) {
		sensor_msgs::PointCloud2 converted;
		pcl::toROSMsg(*as_pcl, converted);
		table_scene.cloud_clusters.emplace_back(converted);
	}



//
//	//Step 6: create and call the grasp action
	ROS_INFO("grasping object %d", item_index);

	bwi_manipulation::GraspCartesianCommand laydown_pose= grasp_object_index(n, table_scene, item_index);

//	pressEnter("Press [ENTER] to proceed");
//
//	//step 7: create and call action to lift and verify
//	lift_object(n,table_scene, largest_pc_index);

	//check if the object was taken by comparing wrench


	ROS_INFO("Moving arm out of view");
	mico->move_to_side_view_approach();
	mico->move_to_side_view();
	mico->move_to_side_view();
	mico->move_to_side_view_approach();



	//Step 9: handover/ the object
// pressEnter("Press [ENTER] when safe to proceed");
	ROS_INFO("Laying the object down on the table");
    mico->move_to_joint_state_moveit(laydown_pose.approach_joint_state);
    mico->move_to_joint_state_moveit(laydown_pose.approach_joint_state);
	mico->move_to_joint_state_moveit(laydown_pose.grasp_joint_state);
//	mico->move_to_joint_state_moveit(laydown_pose.grasp_joint_state);
	mico->open_hand();
	mico->move_to_joint_state_moveit(laydown_pose.approach_joint_state);

//	ROS_INFO("Handing over the object");
//	mico->move_to_named_joint_position("side_view_approach");
//	handover_object();
	mico->move_to_side_view_approach();
	mico->close_hand();

	mico->move_to_side_view();

//	mico->move_home();

	return 0;
}
