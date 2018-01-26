#include <signal.h>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include "segbot_arm_perception/PlanarSegmentation.h"
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

//actions
#include <actionlib/client/simple_action_client.h>
#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"

//moveit interface service
#include <moveit_utils/MicoMoveitCartesianPose.h>

#define PI 3.14159265

/* Author: Maxwell Svetlik
 * Current state: finds closest point to end effector through transforms and KdTree search
 * Todo: compute the transform of the orientation of the plane, so the EF can approach the plane normal
 * 
 * Write something (will require format of drawing demo code for drawing)
 */


double cur_x, cur_y, cur_z, cur_qx, cur_qy, cur_qz, cur_qw;

ros::ServiceClient client;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;



void toolpos_cb(const geometry_msgs::PoseStamped &msg){
	geometry_msgs::PoseStamped current = msg;
	cur_x = current.pose.position.x;
	cur_y = current.pose.position.y;
	cur_z = current.pose.position.z;
	cur_qx = current.pose.orientation.x;
	cur_qy = current.pose.orientation.y;
	cur_qz = current.pose.orientation.z;
	cur_qw = current.pose.orientation.w;
}


void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

void approach_jaco(kinova_msgs::ArmPoseGoal goalPose){
	actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ac("/m1n6s200_driver/pose_action/tool_pose", true);
	ac.waitForServer();
	ac.sendGoal(goalPose);
	ac.waitForResult();
}
void approach(geometry_msgs::PoseStamped goal){
	moveit_utils::MicoMoveitCartesianPose srv;
	srv.request.target = goal;
	if(client.call(srv))
		ROS_INFO("Called IK interface service.");
	else
		ROS_INFO("Service call to IK interface failed, is it running?");
}
int main (int argc, char** argv)
{
	ros::init (argc, argv, "approach_board");
	ros::NodeHandle n;
	
	//segmented cloud service client
	ros::ServiceClient ps_client = n.serviceClient<segbot_arm_perception::PlanarSegmentation>("PlanarSegmentation");
	//pose pub
	ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("approach_board/pose", 10);
	//create subscriber to tool position topic
	ros::Subscriber sub_tool = n.subscribe("/m1n6s200_driver/out/tool_pose", 1, toolpos_cb);
	
	client = n.serviceClient<moveit_utils::MicoMoveitCartesianPose>("mico_cartesianpose_service");
	
	tf::TransformListener listener;
	signal(SIGINT, sig_handler);
	
	segbot_arm_perception::PlanarSegmentation srv;
	srv.request.excludePlane = false;
	srv.request.numberOfPlanes = 3;
	char input;
	
	while (ros::ok() && !g_caught_sigint) {
		std::cout << "Enter 1 to call the service and initiate the approach board action" << std::endl;
		std::cin >> input;
		if(input == '1'){
			if(ps_client.call(srv)){
				ros::spinOnce();
				
				segbot_arm_perception::PlanarSegmentation::Response res = srv.response; //assume (naively) that the largest plane is the one desired
				
				if(res.clouds.size() > 0){
					//conversion of pointcloud
					pcl::PCLPointCloud2 temp;
					pcl_conversions::toPCL(res.clouds.at(0),temp);
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::fromPCLPointCloud2(temp,*cloud_plane);
					
					//Transform EF XYZ to /camera/depth_registered
					geometry_msgs::Pose ef_pose;
					ef_pose.position.x = cur_x;
					ef_pose.position.y = cur_y;
					ef_pose.position.z = cur_z;
					//ef_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-PI/2);
					ef_pose.orientation.x = cur_qx;
					ef_pose.orientation.y = cur_qy;
					ef_pose.orientation.z = cur_qz;
					ef_pose.orientation.w = cur_qw;
					
					geometry_msgs::PoseStamped ef_mico;

					ef_mico.header.frame_id = "m1n6s200_link_base";
					ef_mico.header.stamp = ros::Time(0);
					ef_mico.pose = ef_pose;

					geometry_msgs::PoseStamped ef_camera;
					listener.waitForTransform("m1n6s200_link_base", cloud_plane->header.frame_id, ros::Time(0), ros::Duration(3.0));
					listener.transformPose("m1n6s200_link_base", ef_mico, ef_camera);
					
					
					//Find closest point to EF
					pcl::PointXYZ targetPoint;
					int K = 100; //K nearest neighbors
					std::vector<int> pointIdxNKNSearch(K);
					std::vector<float> pointNKNSquaredDistance(K);
					Eigen::Vector4f centroid;
					pcl::compute3DCentroid(*cloud_plane, centroid);
					targetPoint.x = centroid(0); //ef_camera.pose.position.x;
					targetPoint.y = centroid(1); //ef_camera.pose.position.y;
					targetPoint.z = centroid(2); //ef_camera.pose.position.z;
					
					pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
					kdtree.setInputCloud(cloud_plane);
					
					if(kdtree.nearestKSearch(targetPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
						//Compute the normal to the target point
						Eigen::Vector4f plane_parameters;
						float curvature;
						//setViewPoint(targetPoint.x, targetPoint.y, targetPoint.z);
						computePointNormal(*cloud_plane, pointIdxNKNSearch, plane_parameters, curvature);
						//flipNormalTowardsViewpoint(targetPoint, 0, 0, 1, plane_parameters);
						

						//transform pose to mico
						geometry_msgs::Pose target;
						target.position.x = cloud_plane->points[pointIdxNKNSearch[0]].x;
						target.position.y = cloud_plane->points[pointIdxNKNSearch[0]].y;
						target.position.z = cloud_plane->points[pointIdxNKNSearch[0]].z;
						target.orientation = tf::createQuaternionMsgFromRollPitchYaw(plane_parameters[1],-plane_parameters[0], PI);
						//target.orientation.x = cloud_plane->points[pointIdxNKNSearch[0]].x * plane_parameters[0];
						//target.orientation.y = cloud_plane->points[pointIdxNKNSearch[0]].y * plane_parameters[1];
						//target.orientation.z = cloud_plane->points[pointIdxNKNSearch[0]].z * plane_parameters[2];
						//target.orientation.w = plane_parameters[3];
						//std::cout << "PlaneZ: " << plane_parameters[2] << " and mult by pt z: " << (cloud_plane->points[pointIdxNKNSearch[0]].z * plane_parameters[2]) << std::endl;
						std::cout << plane_parameters[0] << std::endl;
						std::cout << plane_parameters[1] << std::endl;
						std::cout << plane_parameters[2] << std::endl;
						geometry_msgs::PoseStamped pose_in;

						pose_in.header.frame_id = cloud_plane->header.frame_id;
						pose_in.header.stamp = ros::Time(0);
						pose_in.pose = target;

						geometry_msgs::PoseStamped pose_out;
						listener.waitForTransform(cloud_plane->header.frame_id, "m1n6s200_link_base", ros::Time(0), ros::Duration(3.0));
						listener.transformPose("m1n6s200_link_base", pose_in, pose_out);
						
						//pose_out.pose.position.x += .05;
						pose_out.pose.position.y += .1;
						pose_out.pose.position.z += .3;

						std::cout << "The target approach point in the arm frame: ";
						std::cout << pose_out.pose.position.x << " " << pose_out.pose.position.y << " " << pose_out.pose.position.z << " ";
						std::cout << pose_out.pose.orientation.x << " " << pose_out.pose.orientation.y   << " " << pose_out.pose.orientation.z << " " << pose_out.pose.orientation.w << std::endl;
						
						kinova_msgs::ArmPoseGoal goalPose;
						goalPose.pose.header.frame_id = "m1n6s200_link_base";
						goalPose.pose.pose.position.x = pose_out.pose.position.x;	
						goalPose.pose.pose.position.y = pose_out.pose.position.y;				
						goalPose.pose.pose.position.z = pose_out.pose.position.z;			
						goalPose.pose.pose.orientation.x = pose_out.pose.orientation.x;		
						goalPose.pose.pose.orientation.y = pose_out.pose.orientation.y;		
						goalPose.pose.pose.orientation.z = pose_out.pose.orientation.z;		
						goalPose.pose.pose.orientation.w = pose_out.pose.orientation.w;
						pose_pub.publish(pose_out);
						char move;
						std::cin >> move;

						if(move == 'm')							
							//approach_jaco(goalPose);
							approach(pose_out);
						
					}
					else
						std::cout << "Unable to find closest point." << std::endl;
				}
				else
					std::cout << "The service returned 0 point clouds." << std::endl;
			}
			else
				std::cout << "Failed to call the service. Is it running?" << std::endl;
			ros::spinOnce();
		}

	}
};
