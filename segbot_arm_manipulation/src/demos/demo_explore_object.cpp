#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <stdio.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/point_cloud_conversion.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <visualization_msgs/Marker.h>

//get table scene and color histogram
#include "segbot_arm_perception/TabletopPerception.h"
#include <segbot_arm_perception/segbot_arm_perception.h>

//actions
#include <actionlib/server/simple_action_server.h>
#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"
#include "kinova_msgs/ArmJointAnglesAction.h"
#include <segbot_arm_manipulation/arm_utils.h>

#include <segbot_arm_manipulation/PressAction.h>
#include <segbot_arm_manipulation/PushAction.h>
#include <segbot_arm_manipulation/TabletopGraspAction.h>
#include <segbot_arm_manipulation/LiftVerifyAction.h>
#include <segbot_arm_manipulation/ShakeAction.h>

//pcl includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/impl/centroid.hpp>


#define FINGER_FULLY_OPENED 6
#define FINGER_FULLY_CLOSED 7300

#define NUM_JOINTS_ARMONLY 6
#define NUM_JOINTS 8 //6+2 for the arm

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


ros::Publisher vis_pub;
sensor_msgs::JointState current_state;
geometry_msgs::PoseStamped current_pose;

bool heardJoinstState;
bool heardPose;


using namespace std;

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

//wait for updated data
void listenForArmData(){
	
	heardJoinstState = false;
	heardPose = false;
	ros::Rate r(10.0);

	while (ros::ok()){
		ros::spinOnce();
		if (heardJoinstState && heardPose){
			return;
		}
		r.sleep();
	}
}

//finds the object on the table
int largest_obj(segbot_arm_perception::TabletopPerception::Response table_scene){
	int largest_pc_index = -1;
	int largest_num_points = -1;
	for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
		int num_points_i = table_scene.cloud_clusters[i].height* table_scene.cloud_clusters[i].width;
		if (num_points_i > largest_num_points){
			largest_num_points = num_points_i;
			largest_pc_index = i;
		}
	}
}

//ask user to input the index of desired object or selects the largest object on the table
int chose_object(std::string message, segbot_arm_perception::TabletopPerception::Response table_scene){
	std::cout << message;
	char size = (char) table_scene.cloud_clusters.size();
	while (true){
		char c = std::cin.get();
		if (c == '\n') {
			return largest_obj(table_scene);
		} else if (c >= '0' && c <= size) {
			return c - '0';
		} else {
			std::cout <<  message;
		}
	}
}


//display a number in rviz for the index of every point cloud available
void show_indicies(segbot_arm_perception::TabletopPerception::Response table_scene){
	for(unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
		sensor_msgs::PointCloud2 tgt = table_scene.cloud_clusters[i];
		
		std::string sensor_frame_id = tgt.header.frame_id;

		PointCloudT pcl_curr;
		pcl::fromROSMsg(tgt, pcl_curr);
		
		PointT max;
		PointT min;
		pcl::getMinMax3D(pcl_curr, min, max);
		
		Eigen::Vector4f center;
		pcl::compute3DCentroid(pcl_curr, center);
		
		visualization_msgs::Marker marker;
		marker.header.frame_id = sensor_frame_id;
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id = i;
		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker.action = visualization_msgs::Marker::ADD;
		
		std::stringstream sstm;
		sstm << i;
		marker.text = sstm.str();
		
		marker.pose.position.x = center(0); 
		marker.pose.position.y = center(1);
		marker.pose.position.z = min.z;
	
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1; 
		
		marker.color.a = 1.0; //alpha
		
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		//only if using a MESH_RESOURCE marker type:
		marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		vis_pub.publish(marker);
		
	}
}

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

std::vector<double> get_color_hist(PointCloudT desired_cloud, int dim){ 
	//get a color histogram and turn it into a one dimensional vector for comparison
	std::vector<std::vector<std::vector<uint> > > hist3= segbot_arm_perception::computeRGBColorHistogram(desired_cloud, dim);
	int i_offset = dim * dim;
	int j_offset = dim;
	std::vector<double> hist3_double_vector (dim * dim * dim, 0);
	for (int i = 0; i < dim; i++) {
		for (int j = 0; j < dim; j++) {
			for (int k = 0; k < dim; k++) {
				hist3_double_vector[i * i_offset + j * j_offset + k] = hist3[i][j][k];
			}
		}
	}
	return hist3_double_vector;	
}
	
double correlation_coeff(std::vector<double> orig_colorhist, std::vector<double> new_colorhist){
	//compares color histogram to determine if the objects are similar, Pearson correlation coefficient
	if(orig_colorhist.size() != new_colorhist.size()){
		ROS_ERROR("color histograms are not the same size. Values will not be accurate!");
		return 0.0;
	}
	double sum_xy = 0.0;
	double sum_x = 0.0;
	double sum_y = 0.0;
	double sum_x_2 = 0.0;
	double sum_y_2 = 0.0;
	double num = 0.0;
	for(unsigned int i = 0; i < orig_colorhist.size(); i++){
		num++; 
		sum_x += orig_colorhist.at(i);
		sum_y += new_colorhist.at(i);
		sum_x_2 += pow(orig_colorhist.at(i), 2);
		sum_y_2 += pow(new_colorhist.at(i) , 2);
		sum_xy += (orig_colorhist.at(i) * new_colorhist.at(i));
	}
	double result = sum_xy - ((sum_x * sum_y)/ num);
	result /= sqrt((sum_x_2 - (pow(sum_x , 2) / num)) * (sum_y_2 - (pow(sum_y , 2) /num)));
	return result;
}

//method to refind the original target object on the table after previous actions have moved it
int refind_obj(segbot_arm_perception::TabletopPerception::Response table_scene, sensor_msgs::PointCloud2 tgt){
	PointCloudT pcl_tgt;
	pcl::fromROSMsg(tgt, pcl_tgt);
	std::vector<double> tgt_color_hist = get_color_hist(pcl_tgt, 8);
	double max = -0.1;
	double max_index = 0;
	for(int i = 0; i< table_scene.cloud_clusters.size(); i++){
		PointCloudT pcl_curr;
		pcl::fromROSMsg(table_scene.cloud_clusters.at(i), pcl_curr);
		std::vector<double> curr_color_hist = get_color_hist(pcl_curr, 8);
		double corr = correlation_coeff(tgt_color_hist, curr_color_hist); //compare original and new color histogram
		if(corr > max){
			max = corr;
			max_index = i;
		}
	}
	return max_index;
}


int main (int argc, char** argv){
	ros::init(argc, argv, "demo_explore_object");
	
	ros::NodeHandle n;
	
	heardPose = false;
	heardJoinstState = false;
	
	//create subscriber to joint angles
	ros::Subscriber sub_angles = n.subscribe ("/m1n6s200_driver/out/joint_state", 1, joint_state_cb);
	
	//create subscriber to tool position topic
	ros::Subscriber sub_tool = n.subscribe("/m1n6s200_driver/out/tool_pose", 1, toolpos_cb);
	
	//create a publisher for rviz text markers
	vis_pub = n.advertise<visualization_msgs::Marker>( "/visualization_marker", 0 );	
	
	pressEnter("Press enter to get table scene or q to quit");
	
	//get table scene and find all objects on table 
	segbot_arm_perception::TabletopPerception::Response table_scene = segbot_arm_manipulation::getTabletopScene(n);
	
	if ((int)table_scene.cloud_clusters.size() == 0){
			ROS_WARN("No objects found on table. The end...");
			exit(1);
	} 
	
	//show indicies of objects on the table using rviz text markers
	pressEnter("Press enter to show indices or q to quit");
	show_indicies(table_scene);
	
	//allow user to pick an object or use default largest object
	int index = chose_object("Enter index of object or press enter to pick largest ", table_scene);
	
	pressEnter("Press enter to start press action or q to quit");
	
	//create the action client to press object
	actionlib::SimpleActionClient<segbot_arm_manipulation::PressAction> press_ac("arm_press_as",true);
	press_ac.waitForServer();
	ROS_INFO("press action server made...");

	
	//fill in goals for press action, send to action
	segbot_arm_manipulation::PressGoal press_goal;
	press_goal.tgt_cloud = table_scene.cloud_clusters[index];
	press_goal.cloud_plane_coef = table_scene.cloud_plane_coef;
	ROS_INFO("Sending goal to press action server...");
	press_ac.sendGoal(press_goal);
		
	//block until the action is completed
	ROS_INFO("Waiting for result...");
	press_ac.waitForResult();
	ROS_INFO("press action Finished...");

	pressEnter("Press enter to start push action or q to quit");
	
	//create the action client to push an object
	actionlib::SimpleActionClient<segbot_arm_manipulation::PushAction> push_ac("arm_push_as",true);
	ROS_INFO("before wait for server");
	push_ac.waitForServer();
	ROS_INFO("push action server made...");

	
	//fill in goals for push action, send to action
	segbot_arm_manipulation::PushGoal push_goal;
	push_goal.tgt_cloud = table_scene.cloud_clusters[index];
	push_goal.cloud_plane = table_scene.cloud_plane; 
	push_goal.cloud_plane_coef = table_scene.cloud_plane_coef;
	
	ROS_INFO("Sending goal to push action server...");
	push_ac.sendGoal(push_goal);
	
	//block until action is completed
	ROS_INFO("Waiting for result...");
	push_ac.waitForResult();
	ROS_INFO("push action Finished...");
	
	pressEnter("Press enter to recheck table");
	//the object has moved from the above actions, recheck table	
	sensor_msgs::PointCloud2 tgt = table_scene.cloud_clusters[index];
	table_scene = segbot_arm_manipulation::getTabletopScene(n);
	
	if ((int)table_scene.cloud_clusters.size() == 0){
			ROS_WARN("No objects found on table. The end...");
			exit(1);
	} 
	
	//for now, use color histograms to find the object again
	index = refind_obj(table_scene, tgt);


	pressEnter("Press enter to start grasp and verify action or q to quit");
	
	//create the action client to grasp
	actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopGraspAction> grasp_ac("segbot_tabletop_grasp_as",true);
	grasp_ac.waitForServer();
	ROS_INFO("grasp action server made...");
		
	//create and fill goal
	segbot_arm_manipulation::TabletopGraspGoal grasp_goal;
		
	//we want the robot to execute the GRASP action
	grasp_goal.action_name = segbot_arm_manipulation::TabletopGraspGoal::GRASP;
	grasp_goal.grasp_selection_method=segbot_arm_manipulation::TabletopGraspGoal::CLOSEST_ORIENTATION_SELECTION;
	
	//fill in the table scene
	grasp_goal.cloud_plane = table_scene.cloud_plane;
	grasp_goal.cloud_plane_coef = table_scene.cloud_plane_coef;
	for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
		grasp_goal.cloud_clusters.push_back(table_scene.cloud_clusters[i]);
	}
	grasp_goal.target_object_cluster_index = index;
			
	//send goal
	ROS_INFO("Sending goal to action server...");
	grasp_ac.sendGoal(grasp_goal);
	
	//block until action is completed
	ROS_INFO("Waiting for result...");
	grasp_ac.waitForResult();
	
	ROS_INFO("Grasp action Finished...");

	//create action to lift and verify
	actionlib::SimpleActionClient<segbot_arm_manipulation::LiftVerifyAction> lift_ac("arm_lift_verify_as", true);
	lift_ac.waitForServer();
	
	//fill goals of lift and verify action, send to action
	segbot_arm_manipulation::LiftVerifyGoal lift_verify_goal;
	lift_verify_goal.tgt_cloud = table_scene.cloud_clusters[index];
	lift_verify_goal.bins = 8;
	
	ROS_INFO("sending goal to lift and verify action server...");
	lift_ac.sendGoal(lift_verify_goal);
	
	//block until action is completed
	ROS_INFO("waiting for lift and verify action server result....");
	lift_ac.waitForResult();
	ROS_INFO("lift and verify action finished.");
	
	//get result of the lift and verify action 
	segbot_arm_manipulation::LiftVerifyResult result = *lift_ac.getResult();
	
	bool verified = result.success;
	
	//display result to user
	if(verified){
		ROS_INFO("Verification succeeded.");
	}else{
		ROS_WARN("Verification failed");
	}
	
	pressEnter("Press enter to start shake action or q to quit");
	
	//create action to shake object
	actionlib::SimpleActionClient<segbot_arm_manipulation::ShakeAction> shake_ac("arm_shake_as", true);
	shake_ac.waitForServer();
	ROS_INFO("shake action server made...");
	
	//fill goals for shake action, send to action
	segbot_arm_manipulation::ShakeGoal shake_goal;
	shake_goal.tgt_cloud = table_scene.cloud_clusters[index];
	shake_goal.cloud_plane = table_scene.cloud_plane;
	shake_goal.verified = verified; 
	
	ROS_INFO("sending goal to shake action server....");
	shake_ac.sendGoal(shake_goal);
	
	//block until action is completed
	ROS_INFO("waiting for shake action server result....");
	shake_ac.waitForResult();
	
	ROS_INFO("shake action finished...");

	
	
	return 0;
}
