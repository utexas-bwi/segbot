#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32.h>


#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <nav_msgs/Odometry.h>

//tf stuff
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"

// PCL specific includes
//#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>


#include <geometry_msgs/TwistStamped.h>


#define PI 3.14159265

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//where we store results from calling the perception service
std::vector<PointCloudT::Ptr > detected_objects;

using namespace std;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;




geometry_msgs::PoseStamped current_moveit_pose;


//publishers
ros::Publisher pub_base_velocity;
ros::Publisher pose_pub;

 
sensor_msgs::PointCloud2 cloud_ros;

bool heard_odom = false;;
nav_msgs::Odometry current_odom;




/* what happens when ctr-c is pressed */
void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};


// Blocking call for user input
void pressEnter(){
	std::cout << "Press the ENTER key to continue";
	while (std::cin.get() != '\n')
		std::cout << "Please press ENTER\n";
}
//nav_msgs/Odometry

void odom_cb(const nav_msgs::OdometryConstPtr& input){
	current_odom = *input;
	heard_odom = true;
}

double getYaw(geometry_msgs::Pose pose){
	tf::Quaternion q(pose.orientation.x, 
							pose.orientation.y, 
							pose.orientation.z, 
							pose.orientation.w);
	tf::Matrix3x3 m(q);
	
	double r, p, y;
	m.getRPY(r, p, y);
	return y;
}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "candy_handover_demo");
	
	ros::NodeHandle n;

	
	//used to compute transfers
	tf::TransformListener tf_listener;
	
	//register ctrl-c
	signal(SIGINT, sig_handler);


	//subscribe to odometry 
	ros::Subscriber sub_odom = n.subscribe("/odom", 1, odom_cb);

	//publisher for debugging purposes
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/approach_table_target_pose", 1);
	pub_base_velocity = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	//user input
    char in;
	
	
	ROS_INFO("Demo starting...");
	pressEnter();
	
	//step 1: query table_object_detection_node to segment the blobs on the table
	ros::ServiceClient client_tabletop_perception = n.serviceClient<segbot_arm_perception::TabletopPerception>("tabletop_object_detection_service");
	segbot_arm_perception::TabletopPerception srv; //the srv request is just empty
	
	
	srv.request.override_filter_z = true;
	srv.request.filter_z_value = 1.5;
	
	
	if (client_tabletop_perception.call(srv))
	{
		ROS_INFO("Received Response from tabletop_object_detection_service");
	}
	else
	{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
	
	Eigen::Vector4f plane_coef_vector;
	for (int i = 0; i < 4; i ++)
		plane_coef_vector(i)=srv.response.cloud_plane_coef[i];

	
	if (srv.response.is_plane_found == false){
		ROS_INFO("Table not found. The end.");
		ros::shutdown();
		exit(1);
	}
	
	
	
	//transform clound into base_link frame of reference
	sensor_msgs::PointCloud cloud_pc1;
	sensor_msgs::convertPointCloud2ToPointCloud(srv.response.cloud_plane,cloud_pc1);
	tf_listener.waitForTransform(srv.response.cloud_plane.header.frame_id, "/base_footprint", ros::Time(0.0), ros::Duration(3.0)); 
	
	sensor_msgs::PointCloud transformed_cloud;
	tf_listener.transformPointCloud("/base_footprint",cloud_pc1,transformed_cloud);	
			
	//convert back to sensor_msgs::PointCloud2 and then to pcl format
	sensor_msgs::PointCloud2 plane_cloud_pc2;
	sensor_msgs::convertPointCloudToPointCloud2(transformed_cloud,plane_cloud_pc2);
	
	PointCloudT::Ptr cloud_plane (new PointCloudT);
	pcl::fromROSMsg(plane_cloud_pc2, *cloud_plane);
	
	//find the point on the table closest to the robot's 0,0
	int closest_point_index = -1;
	double closest_point_distance = 0.0;
	for (int i = 0; i < (int)cloud_plane->points.size(); i++){
		double d_i = sqrt(   pow(cloud_plane->points.at(i).x,2) +   pow(cloud_plane->points.at(i).y,2));
		if (closest_point_index == -1 || d_i < closest_point_distance){
			closest_point_index = i;
			closest_point_distance = d_i;
		}
	}
	
	geometry_msgs::PoseStamped pose_debug;
	pose_debug.header.frame_id = "/base_footprint";
	pose_debug.header.seq = 1;
	pose_debug.pose.position.x = cloud_plane->points.at(closest_point_index).x;
	pose_debug.pose.position.y = cloud_plane->points.at(closest_point_index).y;
	pose_debug.pose.position.z = cloud_plane->points.at(closest_point_index).z;
	pose_debug.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
	pose_pub.publish(pose_debug);
	
	//calculate turn angle
	double target_turn_angle = atan2( cloud_plane->points.at(closest_point_index).y, cloud_plane->points.at(closest_point_index).x);
	
	
	double duration = 2.0; //we want to take this many seconds to get there
	double pub_rate = 30;
	
	double turn_velocity = 0.5*target_turn_angle/duration;
	
	ros::Rate r(pub_rate);
	
	//first, wait for odometry
	heard_odom = false;
	while (!heard_odom){
		r.sleep();
		ros::spinOnce();
	}
	
	double initial_yaw = getYaw(current_odom.pose.pose);
	double target_yaw = initial_yaw + target_turn_angle;
	
	ROS_INFO("Turn angle = %f",target_turn_angle);
	
	
	geometry_msgs::Twist v_i;
	v_i.linear.x = 0; v_i.linear.y = 0; v_i.linear.z = 0;
	v_i.angular.x = 0; v_i.angular.y = 0;
	
	//for (int i = 0; i < (int)duration*pub_rate;i++){
	while (ros::ok()){
		v_i.angular.z = turn_velocity;
		//ROS_INFO_STREAM(v_i);
		
		pub_base_velocity.publish(v_i);
		
		ros::spinOnce();
		
		r.sleep();
		
		double current_yaw = getYaw(current_odom.pose.pose);
		
		if (fabs(current_yaw - target_yaw) < 0.05)
			break;
	}
	
	
	
	v_i.angular.z = 0;
	pub_base_velocity.publish(v_i);
	
	heard_odom = false;
	while (!heard_odom){
		r.sleep();
		ros::spinOnce();
	}
	double final_yaw = getYaw(current_odom.pose.pose);
	
	
	//now, approach table
	
	double distance_to_travel = closest_point_distance - 0.25;
	double start_odom_x = current_odom.pose.pose.position.x;
	double start_odom_y = current_odom.pose.pose.position.y;
	
	double x_vel = 0.15;
	
	while (ros::ok()){
		double distance_traveled = sqrt(  pow(current_odom.pose.pose.position.x - start_odom_x,2) +
										pow(current_odom.pose.pose.position.y - start_odom_y,2));
										
		ROS_INFO("Distance traveled = %f",distance_traveled);				
										
		if (distance_traveled > distance_to_travel){
			break;
		}
		
		
		v_i.linear.x = x_vel;
		pub_base_velocity.publish(v_i);
		
		r.sleep();
		ros::spinOnce();
		
		
		
	}
	v_i.linear.x = 0;
	pub_base_velocity.publish(v_i);
	
	
	ROS_INFO("Closest point distance = %f",closest_point_distance);
	ROS_INFO("Turn angle = %f",target_turn_angle);
	ROS_INFO("Target yaw = %f",target_yaw);
	ROS_INFO("Initial Yaw:\t %f",initial_yaw);
	ROS_INFO("Final Yaw:\t %f",final_yaw);
	
	
	return 0;
}
