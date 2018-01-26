#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include "segbot_arm_perception/TabletopPerception.h"
#include "segbot_arm_manipulation/TabletopGraspAction.h"
#include <segbot_arm_manipulation/arm_utils.h>

#define NUM_JOINTS 8 //6+2 for the arm

//mico joint state safe
//-2.3321322971114142, -1.6372086401627464, -0.28393691436045176, -2.164605083475533, 0.7496982226688764, 4.682638807847723

/* tool pose side
	position: 
		x: 0.117240786552
		y: -0.301719456911
		z: 0.239926770329
	  orientation: 
		x: 0.51289595084
		y: 0.484664185494
		z: 0.517808228151
		w: 0.483645541456

	tool pose safe
		
	x: -0.157769784331
    y: -0.136029005051
    z: 0.376786500216
  orientation: 
    x: 0.994340247286
    y: 0.0977247708014
    z: 0.005313327657
    w: 0.0413413878465


*/

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
	ros::init(argc, argv, "demo_grasp_action_client");
	
	ros::NodeHandle n;

	//create subscriber to joint angles
	ros::Subscriber sub_angles = n.subscribe ("/m1n6s200_driver/out/joint_state", 1, joint_state_cb);
	
	//create subscriber to tool position topic
	ros::Subscriber sub_tool = n.subscribe("/m1n6s200_driver/out/tool_pose", 1, toolpos_cb);

	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	//store out-of-view position here
	sensor_msgs::JointState joint_state_outofview;
	geometry_msgs::PoseStamped pose_outofview;

	pressEnter("Demo starting...move the arm to a position where it is not occluding the table.");
	
	//store out of table joint position
	listenForArmData();
	joint_state_outofview = current_state;
	pose_outofview = current_pose;
	

	while (ros::ok()){
	
		//move the arm to side view
	
	
		//get the table scene
		segbot_arm_perception::TabletopPerception::Response table_scene = segbot_arm_manipulation::getTabletopScene(n);
		
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
		
		//create and fill goal
		segbot_arm_manipulation::TabletopGraspGoal grasp_goal;
		
		//we want the robot to execute the GRASP action
		grasp_goal.action_name = segbot_arm_manipulation::TabletopGraspGoal::GRASP;
		
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
			lift(n,0.07);
			
			//home the arm
			segbot_arm_manipulation::homeArm(n);
			
			//make safe to travel -- generally, from home position, this works
			bool safe = segbot_arm_manipulation::makeSafeForTravel(n);
			
			//now home the arm again
			segbot_arm_manipulation::homeArm(n);
			
			//now wait for human to pull on object
			segbot_arm_manipulation::TabletopGraspGoal handover_goal;
			handover_goal.action_name = segbot_arm_manipulation::TabletopGraspGoal::HANDOVER;
			
		
			ac.waitForServer();
			ROS_INFO("Sending goal to action server...");
			ac.sendGoal(handover_goal);
			
			//block until the action is completed
			ROS_INFO("Waiting for result...");
			
			ac.waitForResult();
			ROS_INFO("Action Finished...");
			
			//move out of view and try again
			segbot_arm_manipulation::homeArm(n);
			segbot_arm_manipulation::moveToJointState(n,joint_state_outofview);
		}
		
		/*sleep(2.0);
		lift(n,-0.07);
		segbot_arm_manipulation::openHand();
		lift(n,0.07);
		
		segbot_arm_manipulation::homeArm(n);
		segbot_arm_manipulation::moveToJointState(n,joint_state_outofview);
		*/
	
		//return 1;
		pressEnter("Press 'Enter' to grasp again or 'q' to quit.");
	}
}
