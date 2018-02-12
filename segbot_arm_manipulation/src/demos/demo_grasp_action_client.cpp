#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <segbot_arm_manipulation/Mico.h>
#include "bwi_perception/TabletopPerception.h"
#include <bwi_perception/bwi_perception.h>
#include <segbot_arm_manipulation/HandoverAction.h>
#include "segbot_arm_manipulation/TabletopGraspAction.h"

segbot_arm_manipulation::Mico *mico;

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
	while (ros::ok()){
		int c = std::cin.get();
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


void lift(double x){
	mico->wait_for_data();
	
	geometry_msgs::PoseStamped p_target = mico->current_pose;
	p_target.pose.position.z += x;
	mico->move_to_pose_moveit(p_target);
}


int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "demo_grasp_action_client");

	ros::NodeHandle n;

    mico = new segbot_arm_manipulation::Mico(n);

	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	//store out-of-view position here
	sensor_msgs::JointState joint_state_outofview;
	geometry_msgs::PoseStamped pose_outofview;

	pressEnter("Demo starting...move the arm to a position where it is not occluding the table.");
	
	//store out of table joint position
	mico->wait_for_data();
	joint_state_outofview = mico->current_state;
	pose_outofview = mico->current_pose;

	while (ros::ok()){
	
		//get the table scene
		bwi_perception::TabletopPerception::Response table_scene = bwi_perception::getTabletopScene(n);
		
		if ((int)table_scene.cloud_clusters.size() == 0){
			ROS_WARN("No objects found on table.");
            pressEnter("Press 'Enter' to try again or 'q' to quit.");
            break;
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
        actionlib::SimpleActionClient<segbot_arm_manipulation::HandoverAction> ac_handover("segbot_handover_as", true);
        ac.waitForServer();
		
		//create and fill goal
		segbot_arm_manipulation::TabletopGraspGoal grasp_goal;

		
		//for that action, we have to specify the method used for picking the target grasp out of the candidates
		//grasp_goal.grasp_selection_method=segbot_arm_manipulation::TabletopGraspGoal::CLOSEST_ORIENTATION_SELECTION;
		grasp_goal.grasp_selection_method=segbot_arm_manipulation::TabletopGraspGoal::CLOSEST_JOINTSPACE_SELECTION;
		
		
		//grasp_goal.grasp_filter_method=segbot_arm_manipulation::TabletopGraspGoal::TOPDOWN_GRASP_FILTER;
		//grasp_goal.grasp_filter_method=segbot_arm_manipulation::TabletopGraspGoal::SIDEWAY_GRASP_FILTER;
		
		
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
		
		bool result = ac.waitForResult();

		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
		if (state.state_ == actionlib::SimpleClientGoalState::ABORTED){\
			ROS_WARN("Grasping server aborted action...");
		}
		else {
		
			//lift and lower the object a bit, let it go and move back
			lift(0.07);
			
			//home the arm
			mico->move_home();
			
			//make safe to travel -- generally, from home position, this works
			bool safe = mico->make_safe_for_travel();
			
			//now home the arm again
			mico->move_home();
			
			//now wait for human to pull on object
            segbot_arm_manipulation::HandoverGoal handover_goal;
            handover_goal.type = segbot_arm_manipulation::HandoverGoal::GIVE;
            handover_goal.timeout_seconds = -1.0;


            ac_handover.waitForServer();
			ROS_INFO("Sending goal to action server...");
            ac_handover.sendGoal(handover_goal);
			
			//block until the action is completed
			ROS_INFO("Waiting for result...");

            ac_handover.waitForResult();
			ROS_INFO("Action Finished...");
			
			//move out of view and try again
			mico->move_home();
			mico->move_to_joint_state_moveit(joint_state_outofview);
		}
		
		/*sleep(2.0);
		lift(n,-0.07);
		mico->open_hand();
		lift(n,0.07);
		
		mico->move_home();
		mico->move_to_joint_state(,joint_state_outofview);
		*/
	
		//return 1;
		pressEnter("Press 'Enter' to grasp again or 'q' to quit.");
	}
}
