#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/JointState.h>

//srv for talking to table_object_detection_node.cpp
#include "bwi_perception/TabletopPerception.h"
#include "bwi_perception/bwi_perception.h"

//action for grasping
#include "segbot_arm_manipulation/TabletopGraspAction.h"

#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/MicoManager.h>


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


//true if Ctrl-C is pressed
bool g_caught_sigint=false;

MicoManager *mico;

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

//create stamped pose from point cloud
geometry_msgs::PoseStamped pclToPoseStamped(sensor_msgs::PointCloud2 pc2){
	//transform to PCL
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
	pcl::fromROSMsg(pc2, pcl_cloud);
		
	//create a pose with x y z set to the center of point cloud
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(pcl_cloud, centroid);
		
	geometry_msgs::Pose pose_i;
	pose_i.position.x=centroid(0);
	pose_i.position.y=centroid(1);
	pose_i.position.z=centroid(2);
	pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,3.14/2,3.14/2);

	geometry_msgs::PoseStamped stampedPose;

	stampedPose.header.frame_id = pc2.header.frame_id;
	stampedPose.header.stamp = ros::Time(0);
	stampedPose.pose = pose_i;
		
	//adjust z and y positions
	stampedPose.pose.position.z += 0.1;
	stampedPose.pose.position.y += 0.07;
	return stampedPose;
}

//lower and lift hand
void pushButton() {
	double timeoutSeconds = 2.0;
	kinova_msgs::PoseVelocity velocityMsg;

    velocityMsg.twist_linear_z = -0.125;
	
	mico->move_with_cartesian_velocities(velocityMsg, timeoutSeconds);

    velocityMsg.twist_linear_y = -0.125;
    velocityMsg.twist_linear_z = 0.2;
    mico->move_with_cartesian_velocities(velocityMsg, 3.0);

}


int main(int argc, char**argv){
	
	ros::init(argc, argv, "prep_button");
	ros::NodeHandle n;

	//ctrl-c
	signal(SIGINT, sig_handler);

	mico = new MicoManager(n);
	//store out-of-view position here
	sensor_msgs::JointState joint_state_outofview;
	geometry_msgs::PoseStamped pose_outofview;
	
	//create listener for transforms
	tf::TransformListener tf_listener;

	//create publisher for pose
	ros::Publisher button_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/prep_button/pose", 10);;
	
	pressEnter("Demo starting...move the arm to a position where it is not occluding the table.");
	
	//store out of table joint position
	mico->wait_for_data();
	joint_state_outofview = mico->current_state;
	pose_outofview = mico->current_pose;
	
	if(ros::ok()){
		
		//get the table scene (from demo grasp action client)
		bwi_perception::TabletopPerception::Response table_scene = bwi_perception::getTabletopScene(n);
		
		if ((int)table_scene.cloud_clusters.size() == 0){
			ROS_WARN("No objects found on table. The end...");
			exit(1);
		}
		
		//select the object with most points as the target object (from demo grasp action client)
		int button_pc_index = -1;
		int largest_num_points = -1;
		for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
			
			int num_points_i = table_scene.cloud_clusters[i].height* table_scene.cloud_clusters[i].width;
		
			if (num_points_i > largest_num_points){
				largest_num_points = num_points_i;
				button_pc_index = i;
			}
		}
		
		//wait for transform and perform it
		tf_listener.waitForTransform(table_scene.cloud_clusters[button_pc_index].header.frame_id,"m1n6s200_link_base",ros::Time(0), ros::Duration(3.0)); 
		
	    sensor_msgs::PointCloud2 button_cloud = table_scene.cloud_clusters[button_pc_index];
		
		//transform to base link frame of reference
		pcl_ros::transformPointCloud ("m1n6s200_link_base", button_cloud, button_cloud, tf_listener);
		
		//create and publish pose
		geometry_msgs::PoseStamped stampedPose = pclToPoseStamped(button_cloud);
		button_pose_pub.publish(stampedPose);
		ros::spinOnce();
		
		//move to pose
		mico->move_to_pose_moveit( stampedPose);
		mico->move_to_pose_moveit( stampedPose);
		mico->move_to_pose_moveit( stampedPose);
		pushButton();
		mico->move_home();
	}
}
