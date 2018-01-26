#include <ros/ros.h>
#include <ros/package.h>

#include <cstdlib>

#include <geometry_msgs/TwistStamped.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"
#include "kinova_msgs/ArmJointAnglesAction.h"

#include <moveit_msgs/DisplayRobotState.h>
// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>

#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <pcl_ros/impl/transforms.hpp>

//the action definition
#include "segbot_arm_manipulation/ObjReplacementAction.h"

//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"

#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/grasp_utils.h>
#include <segbot_arm_manipulation/arm_positions_db.h>

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
PointT sensor_origin;

#define NUM_JOINTS_ARMONLY 6
#define NUM_JOINTS 8 //6+2 for the arm

#define JOINT_RADIUS .195
#define ABOVE_TABLE 0.1
#define POSE_THRESHOLD 0.15




class ObjReplacementActionServer
{
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<segbot_arm_manipulation::ObjReplacementAction> as_; 
	std::string action_name_; 

	segbot_arm_manipulation::ObjReplacementFeedback feedback_; 
	segbot_arm_manipulation::ObjReplacementResult result_; 

	sensor_msgs::JointState current_state;
	sensor_msgs::JointState current_effort;
	kinova_msgs::FingerPosition current_finger;
	geometry_msgs::PoseStamped current_pose;
	bool heardPose;
	bool heardJoinstState; 

	bool heardGrasps;
	agile_grasp::Grasps current_grasps;

	//used to compute transforms
	tf::TransformListener listener;

	ros::Subscriber sub_angles;
	ros::Subscriber sub_torques;
	ros::Subscriber sub_tool;
	ros::Subscriber sub_finger;
	ros::Subscriber sub_grasps;

	ros::Publisher down_pub; 
    
    std::vector<geometry_msgs::Quaternion> orientations; 



public:
	ObjReplacementActionServer(std::string name) :
		as_(nh_, name, boost::bind(&ObjReplacementActionServer::executeCB, this, _1), false),
    		action_name_(name)
    {
    	heardPose = false;
		heardJoinstState = false;
		heardGrasps = false;

		//create subscriber to joint angles
		sub_angles = nh_.subscribe ("/joint_states", 1, &ObjReplacementActionServer::joint_state_cb, this);

		//create subscriber to joint torques
		sub_torques = nh_.subscribe ("/m1n6s200_driver/out/joint_efforts", 1, &ObjReplacementActionServer::joint_effort_cb,this);

		//create subscriber to tool position topic
		sub_tool = nh_.subscribe("/m1n6s200_driver/out/tool_position", 1, &ObjReplacementActionServer::toolpos_cb, this);

		//subscriber for fingers
		sub_finger = nh_.subscribe("/m1n6s200_driver/out/finger_position", 1, &ObjReplacementActionServer::fingers_cb, this);

		//publisher for downsampled point cloud (used for debugging purposes)
		down_pub = nh_.advertise<sensor_msgs::PointCloud2>("segbot_obj_replacement_as/down_cloud", 1);

    	ROS_INFO("Starting replacement grasp action server..."); 
    	as_.start(); 
    }

	~ObjReplacementActionServer(void)
	{
	}
	
	
	//Joint state cb
	void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
		if (input->position.size() == NUM_JOINTS){
			current_state = *input;
			heardJoinstState = true;
		}
	}

	//Joint effort cb
	void joint_effort_cb (const sensor_msgs::JointStateConstPtr& input) {
		current_effort = *input;
	}

	//tool position cb
	void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
		current_pose = msg;
		heardPose = true;
	}

	//fingers state cb
	void fingers_cb (const kinova_msgs::FingerPosition msg) {
		current_finger = msg;
	}
		
	//wait for updated arm information	
	void listenForArmData(float rate){
		heardPose = false;
		heardJoinstState = false;
		ros::Rate r(rate);
		
		while (ros::ok()){
			ros::spinOnce();
			
			if (heardPose && heardJoinstState)
				return;
			
			r.sleep();
		}
	}	
	
	/*Function to check the differences between the joint states*/
	bool check_position(sensor_msgs::JointState goal_state, float threshold){
		std::vector<double> joint_diffs = segbot_arm_manipulation::getJointAngleDifferences(current_state, goal_state);
		for(int i = 0; i< joint_diffs.size(); i++){
			if(joint_diffs[i] > threshold){
				return false;
			}
		}
		return true;
	}

	/*Function to downsample an input cloud using VoxelGrid filter*/
	void downsample_clouds(PointCloudT::Ptr in_cloud, PointCloudT::Ptr out_cloud, float leaf_size){
		pcl::VoxelGrid<PointT> grid; 
		grid.setInputCloud(in_cloud);
		grid.setLeafSize (leaf_size, leaf_size, leaf_size); 
		grid.filter(*out_cloud);
	}
	
	/*Function to find the distance between geometry_msgs points*/
	float euclidean_distance(geometry_msgs::Point target , geometry_msgs::Point actual){
		float x_diff = (float) target.x - actual.x;
		float y_diff = (float) target.y - actual.y;
		float z_diff = (float) target.z - actual.z;
		return (float) sqrt(pow(x_diff, 2) + pow(y_diff, 2) + pow (z_diff, 2));
		
	}
	
	/*helper function for sorting the points of a point cloud by the distance
	 * to the camera sensor. Requires that you have set the sensor origin variable
	 * prior to sorting the pointcloud*/
	static bool sort_hlp(const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2){
		//sort by distance to camera sensor
		float x_diff = (float) p1.x - sensor_origin.x;
		float y_diff = (float) p1.y - sensor_origin.y;
		float z_diff = (float) p1.z - sensor_origin.z;
		float p1_diff = (float) sqrt(pow(x_diff, 2) + pow(y_diff, 2) + pow (z_diff, 2));
		
		x_diff = (float) p2.x - sensor_origin.x;
		y_diff = (float) p2.y - sensor_origin.y;
		z_diff = (float) p2.z - sensor_origin.z;
		float p2_diff = (float) sqrt(pow(x_diff, 2) + pow(y_diff, 2) + pow (z_diff, 2));
		
		return p1_diff < p2_diff;
	}
	
	/*check if the position of the end effector is within some threshold of the target end effect position*/
	bool check_if_reached(geometry_msgs::PoseStamped target_pose, geometry_msgs::PoseStamped actual_pose){
		float distance =  euclidean_distance(target_pose.pose.position, actual_pose.pose.position);
		
		if(distance >= POSE_THRESHOLD){
			//further than threshold centimeters away from goal location
			return false;
		}
		return true;
	}
	
	void get_orientations(std::vector<geometry_msgs::Quaternion> orientations) {
		orientations.push_back(current_pose.pose.orientation); 
        orientations.push_back(tf::createQuaternionMsgFromRollPitchYaw(3.14/2, 0, 0)); 
        orientations.push_back(tf::createQuaternionMsgFromRollPitchYaw(-3.14/2, 0, 0));
        orientations.push_back(tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0));
	}
	
	/*Assumptions: the robot has already approached the table, 
	 * an object is in hand, the arm is currently still in safety mode,
	 * cloud is organized and dense*/
	void executeCB(const segbot_arm_manipulation::ObjReplacementGoalConstPtr &goal){
		//step1: get the table scene, check validity
		segbot_arm_perception::TabletopPerception::Response table_scene = segbot_arm_manipulation::getTabletopScene(nh_);
		
		if (!table_scene.is_plane_found){
			ROS_ERROR("[segbot_arm_replacement_as] a table must be present");
			result_.success = false;
			result_.error_msg = "a table must be present";
			as_.setAborted(result_);
			return;
		}
		sensor_msgs::PointCloud2 plane = table_scene.cloud_plane;
		
		
		//transform the table scene point cloud 
		sensor_msgs::PointCloud scene_plane;
		listener.waitForTransform(plane.header.frame_id, "m1n6s200_link_base", ros::Time(0), ros::Duration(3.0));
		sensor_msgs::convertPointCloud2ToPointCloud(plane, scene_plane);
		listener.transformPointCloud("m1n6s200_link_base", scene_plane ,scene_plane); 
		
		//update the plane cloud to the transformed cloud
		sensor_msgs::convertPointCloudToPointCloud2(scene_plane, plane);
		
		//get a pcl point cloud of the given plane
		PointCloudT::Ptr pcl_scene_plane (new PointCloudT);
		pcl::fromROSMsg(plane, *pcl_scene_plane);
		
		//step2: voxel grid filter with a distance size of 5cm
		PointCloudT::Ptr plane_down_sam (new PointCloudT);
		downsample_clouds(pcl_scene_plane, plane_down_sam, 0.05f);

		//publish downsampled cloud for debugging
		sensor_msgs::PointCloud2 plane_down_ros;
		pcl::toROSMsg(*plane_down_sam,plane_down_ros);
		plane_down_ros.header.frame_id = plane_down_sam->header.frame_id;
		down_pub.publish(plane_down_ros);

		//ensure a table is present
		int num_points = (int) plane_down_sam->points.size();
		if ((int)num_points == 0){
			ROS_ERROR("[segbot_arm_replacement_as] down sampled cloud has no points");
			result_.success = false;
			result_.error_msg = "down sampled cloud must still have points";
			as_.setAborted(result_);
			return;
		}
		
		//step3: sort the points in the point cloud by distance to camera
		//set the pointT of the camera's sensor
		Eigen::Vector4f origin = plane_down_sam->sensor_origin_; 
		sensor_origin.x = origin(0);
		sensor_origin.y = origin(1);
		sensor_origin.z = origin(2);
		
		std::sort(plane_down_sam->points.begin(), plane_down_sam->points.end(), sort_hlp);
        		
		//step4: go through the points and find a place to set the object
		for(int ind = 0; ind < num_points; ind++){
			
			//create the current goal for the arm to move to when resetting
			geometry_msgs::PoseStamped current_goal;
			current_goal.header.frame_id = current_pose.header.frame_id;
			current_goal.pose.position.x = plane_down_sam->points[ind].x;
			current_goal.pose.position.y = plane_down_sam->points[ind].y;
			
			//set z value slightly above the table
			current_goal.pose.position.z = plane_down_sam->points[ind].z + ABOVE_TABLE;

			current_goal.pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(-3.14/2, 3.14, 0); 
			
			//check the inverse kinematics, if possible, move to the pose and drop object
			moveit_msgs::GetPositionIK::Response ik_response_1 = segbot_arm_manipulation::computeIK(nh_,current_goal);

			if (ik_response_1.error_code.val == 1){
				segbot_arm_manipulation::moveToPoseMoveIt(nh_, current_goal);
				if(check_if_reached(current_goal, current_pose)){
					//reached location, success
					segbot_arm_manipulation::openHand();
					result_.success = true; 
					break;
				}
				//did not reach location, continue trying 
				result_.success = false; 
			}

		}
		
		//step5: home arm 
		segbot_arm_manipulation::homeArm(nh_);
		
		//step6: set success of the action
		as_.setSucceeded(result_);

	}

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "segbot_obj_replacement_as");

  ObjReplacementActionServer as(ros::this_node::getName());
  ros::spin();

  return 0;
}
