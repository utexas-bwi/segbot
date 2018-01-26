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
#include <kinova_msgs/PoseVelocity.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>


//tf stuff
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>


//actions
#include <actionlib/client/simple_action_client.h>
#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"
#include "kinova_msgs/ArmJointAnglesAction.h"


//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_manipulation/iSpyTouch.h"
#include "segbot_arm_manipulation/iSpyDetectTouch.h"
#include "segbot_arm_manipulation/iSpyFaceTable.h"

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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/distances.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <moveit_msgs/DisplayRobotState.h>
// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>

//some message used for publishing or in the callbacks
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <kinova_msgs/JointVelocity.h>

#define NUM_JOINTS 8 //6+2 for the arm
#define FINGER_FULLY_OPENED 6
#define FINGER_FULLY_CLOSED 7300

//some defines for behaviors
#define TOUCH_POSE_HEIGHT 0.095
#define TIMEOUT_THRESHOLD 30.0 //30 seconds

#define PI 3.14159265

//threshold used to determine if detect change indicates 
//a contact with the object
#define TOUCH_DISTANCE_THRESHOLD 0.05 //5 cm

using namespace std;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

// robot state information
sensor_msgs::JointState current_state;
kinova_msgs::FingerPosition current_finger;
geometry_msgs::PoseStamped current_pose;
bool heardPose = false;
bool heardJoinstState = false;
geometry_msgs::PoseStamped home_pose;


nav_msgs::Odometry current_odom;
bool heard_odom;

//global variables about the joint efforts
double total_grav_free_effort = 0;
double total_delta;
double delta_effort[6];


/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//cloud representing change
PointCloudT::Ptr cloud_change (new PointCloudT);
boost::mutex change_cloud_mutex;
bool new_change_cloud_detected = false;

//clients
ros::ServiceClient client_start_change;
ros::ServiceClient client_stop_change;
ros::ServiceClient client_joint_command;

//publishers
ros::Publisher pub_velocity;
ros::Publisher pub_base_velocity;
ros::Publisher change_cloud_debug_pub; //publishes the filtered change cloud 
ros::Publisher detected_change_cloud_pub; //publishes the cluster detected to be close to the object
sensor_msgs::PointCloud2 cloud_ros;
pcl::PCLPointCloud2 pc_target;
ros::Publisher pub_angular_velocity;

//const float home_position [] = { -1.84799570991366, -0.9422852495301872, -0.23388692957209883, -1.690986384686938, 1.37682658669572, 3.2439323416434624};

//original studey
//const float home_position [] = {-1.9461704803383473, -0.39558648095261406, -0.6342860089305954, -1.7290658598495474, 1.4053863262257316, 3.039252699220428};

//closer to laptop so robot can turn
const float home_position [] = {-2.104982765623053, -0.45429879666061385, -0.04619985280042514, -1.5303365182500774, 1.1245469345291512, 2.8024433498261305};

const float home_position_approach [] = {-1.9480954131742567, -0.9028227948134995, -0.6467984718381701, -1.4125267937404524, 0.8651278801122975, 3.73659131064558};
const float safe_position [] = {-2.0905452367215145, -1.6631960323958503, 0.14437462322511263, -2.626324245881435, 1.3756366863206724, 4.141190461359241};

//which table we currently face (from  1 to 3)
int current_table = 2;

//count how many times we have turned
int num_turns_taken = 0;

//some global variables related to "fidgeting" while the robot is listening for voice
double theta_angle = 0.0;
double z_vel_magnitude = 0.2;
double cycle_length = 2.0;
bool is_listening = false;

//some global variables related to fidgeting when waiting for touch
bool is_waiting_for_touch = false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

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


//odom state cb
void odom_cb(const nav_msgs::OdometryConstPtr& input){
	current_odom = *input;
	heard_odom = true;
}

//Joint state cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
	if (input->position.size() == NUM_JOINTS){
		//ROS_INFO("Heard arm joint states!");
		current_state = *input;	
	}
    	//compute the change in efforts if we had already heard the last one
	if (heardJoinstState){
		for (int i = 0; i < 6; i ++){
			delta_effort[i] = input->effort[i]-current_state.effort[i];
		}
	}
	
	//store the current effort
	current_state = *input;
	
	total_grav_free_effort = 0.0;
	for (int i = 0; i < 6; i ++){
		if (current_state.effort[i] < 0.0)
			total_grav_free_effort -= (current_state.effort[i]);
		else 
			total_grav_free_effort += (current_state.effort[i]);
	}
	
	//calc total change in efforts
	total_delta = delta_effort[0]+delta_effort[1]+delta_effort[2]+delta_effort[3]+delta_effort[4]+delta_effort[5];
	
	heardJoinstState = true;
  //ROS_INFO_STREAM(current_state);
}

void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
  //ROS_INFO("Heard arm tool pose!");
  current_pose = msg;
  heardPose = true;
  //  ROS_INFO_STREAM(current_pose);
}

void fingers_cb (const kinova_msgs::FingerPosition msg) {
  current_finger = msg;
}


void listenForArmData(float rate, double timeout){
	heardPose = false;
	heardJoinstState = false;
	ros::Rate r(rate);
	
	double elapsed_time = 0.0;
	
	while (ros::ok()){
		ROS_INFO_THROTTLE(2, "Listening for arm data...");
		
		ros::spinOnce();
		
		if (heardPose && heardJoinstState)
			return;
		
		r.sleep();
		elapsed_time += 1.0/rate;
		
		if (timeout > 0 && elapsed_time > timeout){
			ROS_WARN("Listening for arm data failed...this shouldn't happen!");
			return;
		}
	}
}


std::vector<PointCloudT::Ptr > computeClusters(PointCloudT::Ptr in){
	std::vector<PointCloudT::Ptr > clusters;

	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (in);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (300);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (in);
	ec.extract (cluster_indices);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		PointCloudT::Ptr cloud_cluster (new PointCloudT);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (in->points[*pit]); 
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		clusters.push_back(cloud_cluster);
  }
  
  return clusters;
}

geometry_msgs::PoseStamped createTouchPose(PointCloudT::Ptr blob, 
					Eigen::Vector4f plane_coefficients,
					std::string frame_id,
					bool transform_into_mico){
	//basically, find the point furthers away from the plane -- that's the top of the object
	double max_distance = -1000.0;
	int max_index = -1;
	
	//first, we find the point in the blob closest to the plane
	for (unsigned int i = 0; i < blob->points.size(); i++){
		pcl::PointXYZ p_i;
		p_i.x=blob->points.at(i).x;
		p_i.y=blob->points.at(i).y;
		p_i.z=blob->points.at(i).z;

		double distance = pcl::pointToPlaneDistance(p_i, plane_coefficients);
		

		if (distance > max_distance){
			max_distance = distance;
			max_index = i;
		}			
	}
	
	//now create the pose
	geometry_msgs::PoseStamped pose_st;
	pose_st.header.stamp = ros::Time(0);
	pose_st.header.frame_id =frame_id.c_str();
	
	pose_st.pose.position.x = blob->points.at(max_index).x;
	pose_st.pose.position.y = blob->points.at(max_index).y;
	pose_st.pose.position.z = blob->points.at(max_index).z;
	//this orientation here doesn't matter, it just has to be valid in order for the transform to work
	pose_st.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
	
	//tranform pose into arm frame of reference
	tf::TransformListener listener;
	listener.waitForTransform(pose_st.header.frame_id, "m1n6s200_link_base", ros::Time(0), ros::Duration(3.0));
	listener.transformPose("m1n6s200_link_base", pose_st, pose_st);
		
	pose_st.pose.position.x = pose_st.pose.position.x - 0.075;	
			
	//decide on orientation (horizontal palm)
	pose_st.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(PI/2,0,PI/2);
	
	
	if (!transform_into_mico){
		listener.waitForTransform(pose_st.header.frame_id,frame_id, ros::Time(0), ros::Duration(3.0));
		listener.transformPose(frame_id, pose_st, pose_st);
		
	}
	else {
		//add a bit of z 
		pose_st.pose.position.z+=TOUCH_POSE_HEIGHT;
	}
	
	//ROS_INFO("Touch pose:");
	//ROS_INFO_STREAM(pose_st);
	
	return pose_st;
}

void moveToJointState(const float* js){
	
	/*moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;   
    group.setPlanningTime(5.0); //10 second maximum for collision computation*/
	
	
	
	moveit_utils::MicoMoveitJointPose::Request req;
	moveit_utils::MicoMoveitJointPose::Response res;
	
	for(int i = 0; i < 6; i++){
        switch(i) {
            case 0  :    req.target.joint1 = js[i]; break;
            case 1  :    req.target.joint2 =  js[i]; break;
            case 2  :    req.target.joint3 =  js[i]; break;
            case 3  :    req.target.joint4 =  js[i]; break;
            case 4  :    req.target.joint5 =  js[i]; break;
            case 5  :    req.target.joint6 =  js[i]; break;
        }
	//ROS_INFO("Requested angle: %f", q_vals.at(i));
    }
	
	if(client_joint_command.call(req, res)){
 		ROS_INFO("Call successful. Response:");
 		ROS_INFO_STREAM(res);
 	} else {
 		ROS_ERROR("Call failed. Terminating.");
 		//ros::shutdown();
 	}
}

void moveToPoseCarteseanVelocity(geometry_msgs::PoseStamped pose_st, bool check_efforts, double timeout){
	listenForArmData(30.0, 2.0);
	
	int rateHertz = 40;
	kinova_msgs::PoseVelocity velocityMsg;
	
	
	ros::Rate r(rateHertz);
	
	float theta = 0.095;
	
	float constant_m = 3.0;
	
	ROS_INFO("Starting movement...");
	
	float last_dx = -1;
	float last_dy = -1;
	float last_dz = -1;
	
	int timeout_counter = 0;
	
	double elapsed_time = 0;
	
	while (ros::ok()){
		
		float dx = constant_m*( - current_pose.pose.position.x + pose_st.pose.position.x );
		float dy = constant_m*(- current_pose.pose.position.y + pose_st.pose.position.y);
		float dz = constant_m*(- current_pose.pose.position.z + pose_st.pose.position.z);
		
		/*if (last_dx != -1){
			//check if we're getting further from the target -- this means there is contact
			if (last_dx < dx && last_dy < dy && last_dz < dz){
				ROS_WARN("[ispy_arm_server.cpp] tool has moved further from the target. stopping movement.");
				timeout_counter++;
				
				if (timeout_counter > 5){
					break;
				}
			}
			else timeout_counter = 0;
		}*/
				
		last_dx = dx; 
		last_dy = dy;
		last_dz = dz;
		
		if (fabs(dx) < theta && fabs(dy) < theta && fabs(dz) < theta){
			//we reached the position, exit
			break;
		}
		
		velocityMsg.twist_linear_x = dx;
		velocityMsg.twist_linear_y = dy;
		velocityMsg.twist_linear_z = dz;
		
		velocityMsg.twist_angular_x = 0.0;
		velocityMsg.twist_angular_y = 0.0;
		velocityMsg.twist_angular_z = 0.0;
		
		
		pub_velocity.publish(velocityMsg);
		ros::spinOnce();
		//ROS_INFO("Published cartesian vel. command");
		r.sleep();
		
		//crude approximation of how much time has passed
		elapsed_time += 1.0/rateHertz;
		
		if (check_efforts){
			if (heardJoinstState){
				if (total_delta > 0.8){
					//we hit something, break;
					ROS_WARN("[ispy_arm_server.cpp] contact detecting during cartesean velocity movement.");
					break;
				}
			}
		}
		ROS_INFO("Elapsed time: %f",elapsed_time);
		if (timeout > 0 && elapsed_time > timeout){
			ROS_WARN("Timeout when performing cart. vel. move...moving on.");
			break;
		}
	}
	
	ROS_INFO("Ending movement...");

}



void startChangeDetection(){
	
	std_srvs::Empty::Request req;
	std_srvs::Empty::Response res;

	if(client_start_change.call(req, res)){
 		ROS_INFO("Call to change detect node successful. Response:");
 		ROS_INFO_STREAM(res);
 	} else {
 		ROS_ERROR("Call to change detect node failed. Terminating.");
 		//ros::shutdown();
 	}
	
}


void stopChangeDetection(){
	
	std_srvs::Empty::Request req;
	std_srvs::Empty::Response res;

	if(client_stop_change.call(req, res)){
 		ROS_INFO("Call to change detect node successful. Response:");
 		ROS_INFO_STREAM(res);
 	} else {
 		ROS_ERROR("Call to change detect node failed. Terminating.");
 		//ros::shutdown();
 	}
	
}

void change_cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	change_cloud_mutex.lock ();
	
	//convert to PCL format
	pcl::fromROSMsg (*input, *cloud_change);



	new_change_cloud_detected = true;
	
	change_cloud_mutex.unlock ();
}

bool detect_touch_cb(segbot_arm_manipulation::iSpyDetectTouch::Request &req, 
					segbot_arm_manipulation::iSpyDetectTouch::Response &res)
{
	
	//Step 1: extract perceptual information and compute locations at the top of the objects
	
	//convert object clouds to PCL format
	std::vector<PointCloudT::Ptr > detected_objects;
	for (unsigned int i = 0; i <req.objects.size(); i++){
		PointCloudT::Ptr object_i (new PointCloudT);
		pcl::PCLPointCloud2 pc_i;
		pcl_conversions::toPCL(req.objects.at(i),pc_i);
		pcl::fromPCLPointCloud2(pc_i,*object_i);
		detected_objects.push_back(object_i);
	}
	
	//get plane
	Eigen::Vector4f plane_coef_vector;
	for (int i = 0; i < 4; i ++)
		plane_coef_vector(i)=req.cloud_plane_coef[i];
		
	//for each object, compute the touch pose; also, extract the highest point from the table
	std::vector<geometry_msgs::PoseStamped> touch_poses;
	double highest_z = 0.0;
	for (int i = 0; i < detected_objects.size(); i++){
		//generate touch pose for the object
		geometry_msgs::PoseStamped touch_pose_i = createTouchPose(detected_objects.at(i),plane_coef_vector,
												req.objects.at(i).header.frame_id,false);
			
		if (touch_pose_i.pose.position.z > highest_z){
			highest_z = touch_pose_i.pose.position.z ;
		}
		
		touch_poses.push_back(touch_pose_i);
	}
	
	//transform each pose so that it is with the same frame_id as the objects (the camera frame of reference)
	/*tf::TransformListener listener;
	for (int i = 0; i < detected_objects.size(); i++){
		listener.waitForTransform(touch_poses.at(0).header.frame_id, req.objects.at(0).header.frame_id, ros::Time(0), ros::Duration(3.0));
		listener.transformPose(req.objects.at(i).header.frame_id, touch_poses.at(i), touch_poses.at(i));
	}*/
	
	
	
	
	//Step 2: start change detection server
	startChangeDetection();
	
	//Step 3: process each cloud
	//now detect change close to objects
	double rate = 10.0;
	ros::Rate r(rate);
	
	double elapsed_time = 0.0;
	
	//msg for joint velocity command
	kinova_msgs::JointVelocity jv_msg;
	double turn_direction = 1.0;
	jv_msg.joint1 = 0.0;
	jv_msg.joint2 = 0.0;
	jv_msg.joint3 = 0.0;
	jv_msg.joint4 = 0.0;
	jv_msg.joint5 = 0.0;
	jv_msg.joint6 = turn_direction*45;
	double turn_elapsed_time = 0.0;
	double direction_switch_duration = 5.0;
	
	while (ros::ok()){
		ros::spinOnce();
		
		elapsed_time += 1.0/rate;
		
		//move hand as indicator behavior
		jv_msg.joint6 = turn_direction*45; 
		pub_angular_velocity.publish(jv_msg);
		turn_elapsed_time += 1.0/rate;
		if (turn_elapsed_time > direction_switch_duration){ //switch directions every so often
			turn_elapsed_time = 0;
			turn_direction = turn_direction * (-1.0);
		}
		
		
		if (new_change_cloud_detected){
			
			//first, Z filter on the cloud
			pcl::PassThrough<PointT> pass;
			pass.setInputCloud (cloud_change);
			pass.setFilterFieldName ("z");
			pass.setFilterLimits (0.0, 1.15);
			pass.filter (*cloud_change);
			
			//next, perform statistical outlier filter
			pcl::StatisticalOutlierRemoval<PointT> sor;
			sor.setInputCloud (cloud_change);
			sor.setMeanK (50);
			sor.setStddevMulThresh (1.0);
			sor.filter (*cloud_change);
			
			//publish filtered cloud for debugging purposes
			//pcl::toPCLPointCloud2(*cloud_change,pc_target);
			//pcl_conversions::fromPCL(pc_target,cloud_ros);
			//change_cloud_debug_pub.publish(cloud_ros);
			
			
			//next, compute change clusters
			std::vector<PointCloudT::Ptr > change_clusters = computeClusters(cloud_change);
			
			//find largest cluster
			ROS_INFO("Found %i change clusters...",(int)change_clusters.size());
				
			if (change_clusters.size() > 0){
			
				int max_num_points = -1;
				int largest_index = -1;
				for (unsigned int i = 0; i < change_clusters.size(); i++){
					if ((int)change_clusters.at(i)->points.size() > max_num_points){
						max_num_points = (int)change_clusters.at(i)->points.size();
						largest_index = i;
					}
				}
				
				
				
				//find the smallest distance between any cluster and any object top
				float min_distance = 1000.0;
				int min_object_index = -1;
				int min_change_cluster_index = -1;
				for (unsigned int i = 0; i < change_clusters.size(); i++){
					
					
					for (unsigned int j = 0; j < touch_poses.size();j++){
						Eigen::Vector4f touch_pos_j;
						touch_pos_j(0)=touch_poses.at(j).pose.position.x;
						touch_pos_j(1)=touch_poses.at(j).pose.position.y;
						touch_pos_j(2)=touch_poses.at(j).pose.position.z;
						touch_pos_j(3)=0.0;//unused

						//this variable will be set to the closest distance between
						//the touch pose (the top of the object) and any point in the 
						//j^th change cluster
						float distance_ij = 1000.0;
						
						for (unsigned int k = 0; k < change_clusters.at(i)->points.size(); k++){
							Eigen::Vector4f t;
							t(0) = change_clusters.at(i)->points.at(k).x;
							t(1) = change_clusters.at(i)->points.at(k).y;
							t(2) = change_clusters.at(i)->points.at(k).z;
							t(3) = 0.0;//unused
							
							float dist_jk = pcl::distances::l2(touch_pos_j,t);
							
							if (dist_jk < distance_ij){
								distance_ij = dist_jk;
							}
						}
						
						//now check if this is the min distance between a change cluster and a touch pose
						if (distance_ij < min_distance){
							min_distance = distance_ij;
							min_object_index = j;
							min_change_cluster_index = i;
						}
					}
				}
				
				ROS_INFO("[ispy_arm_server.cpp] min. distance = %f between object %i and change cluster %i",min_distance,min_object_index,min_change_cluster_index);
				
				//now check if the min_distance is below a threshold and if so, report the detection
				
				//publish some debugging info
				
				//publish the change cluster that's closest to an object
				pcl::toPCLPointCloud2(*change_clusters.at(min_change_cluster_index),pc_target);
				pcl_conversions::fromPCL(pc_target,cloud_ros);
				cloud_ros.header.frame_id = cloud_change->header.frame_id;
				change_cloud_debug_pub.publish(cloud_ros);	
				
			
				
				if (min_distance < TOUCH_DISTANCE_THRESHOLD){
					res.detected_touch_index = min_object_index;
					res.success = true;
					
					//publish the change cluster
					/*pcl::toPCLPointCloud2(*change_clusters.at(min_change_cluster_index),pc_target);
					pcl_conversions::fromPCL(pc_target,cloud_ros);
					detected_change_cloud_pub.publish(cloud_ros);*/
					
					
					return true;
				}
			}
			
			
			
			new_change_cloud_detected = false;
		}
		
		//timeout condition
		if (elapsed_time > TIMEOUT_THRESHOLD){
				res.detected_touch_index = -1;
				res.success = false;
				return true;
		}
		
		
		
		r.sleep();
	}
	
	return true;
}



bool touch_object_cb(segbot_arm_manipulation::iSpyTouch::Request &req, 
					segbot_arm_manipulation::iSpyTouch::Response &res)
{
	
	//convert object clouds to PCL format
	std::vector<PointCloudT::Ptr > detected_objects;
	for (unsigned int i = 0; i <req.objects.size(); i++){
		PointCloudT::Ptr object_i (new PointCloudT);
		pcl::PCLPointCloud2 pc_i;
		pcl_conversions::toPCL(req.objects.at(i),pc_i);
		pcl::fromPCLPointCloud2(pc_i,*object_i);
		detected_objects.push_back(object_i);
	}
	
	//get plane
	Eigen::Vector4f plane_coef_vector;
	for (int i = 0; i < 4; i ++)
		plane_coef_vector(i)=req.cloud_plane_coef[i];
	
	ROS_INFO("Received touch object request for object %i",req.touch_index);
	
	//for each object, compute the touch pose; also, extract the highest point from the table
	std::vector<geometry_msgs::PoseStamped> touch_poses;
	double highest_z = 0.0;
	
	for (int i = 0; i < detected_objects.size(); i++){
		//generate touch pose for the object
		geometry_msgs::PoseStamped touch_pose_i = createTouchPose(detected_objects.at(i),plane_coef_vector,req.objects.at(i).header.frame_id,true);
			
		if (touch_pose_i.pose.position.z > highest_z){
			highest_z = touch_pose_i.pose.position.z ;
		}
			
		touch_poses.push_back(touch_pose_i);
	}
	
	if (req.touch_index != -1){
		
		ROS_INFO("Computing touch poses for all objects...");
		ROS_INFO("...done!");
		ROS_INFO("Waiting for arm data...");
		
		//store current pose
		/*listenForArmData(10.0);
		geometry_msgs::PoseStamped start_pose = current_pose;
		ROS_INFO("...done");*/
		
		geometry_msgs::PoseStamped touch_pose_i = touch_poses.at(req.touch_index);
			
		//before we get there, first go a bit above
		double z_above = highest_z+0.2;
			
		geometry_msgs::PoseStamped touch_approach = touch_pose_i;
		touch_approach.pose.position.z = z_above;
		
		ROS_INFO("Moving to approach pose...");	
		//first approach the object from the top
		moveToPoseCarteseanVelocity(touch_approach,false,7.0);
			
		//now touch it
		ROS_INFO("Moving to touch pose...");	
		moveToPoseCarteseanVelocity(touch_pose_i,true,4.0);
	}
	else { //retract
		//store current pose
		listenForArmData(30.0,2.0);
		
		geometry_msgs::PoseStamped touch_approach = current_pose;
		touch_approach.pose.position.z = highest_z+0.2;
		//touch_approach.pose.position.x -= 0.2;
		
		moveToPoseCarteseanVelocity(touch_approach,false,5.0);
		//moveToPoseCarteseanVelocity(home_pose,false);
		//finally call move it to get into the precise joint configuration as the start
		moveToJointState(home_position_approach);
		moveToJointState(home_position);
	}
	
	ROS_INFO("Finished touch object request for object %i",req.touch_index);
	
	
	return true;
}

bool touch_wait_mode_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	ROS_INFO("[ispy_arm_server.cpp] Service callback for toggle waiting for touch mode.");
	
	if (is_waiting_for_touch){
		is_waiting_for_touch = false;
	}
	else 
		is_waiting_for_touch = true;
		
	return true;
}

bool listen_mode_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	ROS_INFO("[ispy_arm_server.cpp] Service callback for toggle listening mode.");
	
	if (is_listening){
		ROS_INFO("Stopping movement...");
		is_listening = false;
	}
	else {
		ROS_INFO("Starting movement...");
		is_listening = true;
	}
	
	return true;
}

bool face_table_cb(segbot_arm_manipulation::iSpyFaceTable::Request &req,
					segbot_arm_manipulation::iSpyFaceTable::Response &res){
	
	ROS_INFO("[ispy_arm_server.cpp] Service callback to turn towards table %i",req.table_index);
	
	//get target table and check if we're there anyway
	int target_table = req.table_index;
	if (target_table == current_table){
		res.success = true;
		return true;
	}
	
	num_turns_taken ++;
	
	//else turn but first make arm safe
	moveToJointState(home_position);
	
	//compute the target amount to turn: 90% to get to the neighboring table
	int table_diff = (current_table - target_table);
	double target_turn_angle = (PI / 2.0) * table_diff;
	
	ROS_INFO("[ispy_arm_server.cpp] Target turn angle: %f",target_turn_angle);
	
	//wait for odometry
	double rate_hz = 100.0;
	ros::Rate r(rate_hz);
	heard_odom = false;
	while (!heard_odom){
		r.sleep();
		ros::spinOnce();
	}
	
	//compute the initial and target yaws according to odom
	double initial_yaw = getYaw(current_odom.pose.pose);
	double target_yaw = initial_yaw + target_turn_angle;
	double current_yaw  = initial_yaw;
	
	ROS_INFO("[ispy_arm_server.cpp] Initial and target yaws: %f, %f",initial_yaw,target_yaw);
	
	//angular turn velocity
	double max_turn_velocity = 0.25;
	double min_turn_velocity = 0.05;
	double velocity_threshold = PI/6;
	
	double turn_velocity = max_turn_velocity;
	double turn_direction = -1.0; //-1 is right
	if (current_table > target_table)
		turn_direction = 1.0; //left
	
	ROS_INFO("Current: %i, Target %i, direction: %f",current_table,target_table,turn_direction);
	
	//message
	geometry_msgs::Twist v_i;
	v_i.linear.x = 0; v_i.linear.y = 0; v_i.linear.z = 0;
	v_i.angular.x = 0; v_i.angular.y = 0;
	

	while (ros::ok()){
		
		//move
		v_i.angular.z = turn_velocity*turn_direction;	
		pub_base_velocity.publish(v_i);
				
		//check for odom messages
		ros::spinOnce();	
		r.sleep();
		
		//update current yaw
		double current_yaw = getYaw(current_odom.pose.pose);
				
		//decide whether to stop turning
		double error = fabs(current_yaw - target_yaw);
		//ROS_INFO("current: %f, target: %f, ERROR: %f",current_yaw, target_yaw, error);
		if ( error < 0.025)
			break;
			
		//update turn velocity based on error
		if (error > velocity_threshold){
			turn_velocity = max_turn_velocity;
		}
		else {
			turn_velocity = min_turn_velocity + (max_turn_velocity-min_turn_velocity)*error/velocity_threshold;
		}
	}
	
	v_i.angular.z = 0.0;	
	pub_base_velocity.publish(v_i);
	
	//else turn but first make arm safe
	moveToJointState(home_position);
	
	
	current_table = target_table;
	
	//check if we need to move forward
	if (num_turns_taken > 10 && current_table == 2){
		double start_odom_x = current_odom.pose.pose.position.x;
		double start_odom_y = current_odom.pose.pose.position.y;
			
		
		v_i.linear.x = 0.1; v_i.linear.y = 0; v_i.linear.z = 0;
		v_i.angular.x = 0; v_i.angular.y = 0; v_i.angular.z = 0;
		
		double elapsed_time = 0.0;
		
		while (ros::ok()){
			
			//move
			pub_base_velocity.publish(v_i);
					
			//check for odom messages
			ros::spinOnce();	
			r.sleep();
			
			elapsed_time += rate_hz;
			
			double distance_traveled = sqrt(  pow(current_odom.pose.pose.position.x - start_odom_x,2) +
												pow(current_odom.pose.pose.position.y - start_odom_y,2));
			ROS_INFO("Distance traveled = %f",distance_traveled);
			if (distance_traveled > 0.05 || elapsed_time > 1.0){
					break;
			}
			
		}
		num_turns_taken = 0;
	}
	
	
	res.success = true;
	return true;
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "ispy_arm_server");
	ros::NodeHandle n;
	
	//create subscriber to joint angles
	ros::Subscriber sub_angles = n.subscribe ("/m1n6s200_driver/out/joint_state", 10, joint_state_cb);

	//create subscriber to tool position topic
	ros::Subscriber sub_tool = n.subscribe("/m1n6s200_driver/out/tool_pose", 10, toolpos_cb);

	//subscriber for fingers
	ros::Subscriber sub_finger = n.subscribe("/m1n6s200_driver/out/finger_position", 10, fingers_cb);
	 
	//subscriber for change cloud
	ros::Subscriber sub_change_cloud = n.subscribe("/segbot_arm_table_change_detector/cloud",10,change_cloud_cb);  
	 
	//subscriber for odmetry
	ros::Subscriber sub_odom = n.subscribe("/odom", 10,odom_cb);
 
	 
	 
	//publish velocities
	pub_velocity = n.advertise<kinova_msgs::PoseVelocity>("/m1n6s200_driver/in/cartesian_velocity", 10);
	
	//publish angular velocities
	pub_angular_velocity = n.advertise<kinova_msgs::JointVelocity>("/m1n6s200_driver/in/joint_velocity", 10);

	
	//cloud publisher
	change_cloud_debug_pub = n.advertise<sensor_msgs::PointCloud2>("ispy_arm_server/change_cloud_filtered", 10);
	detected_change_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("ispy_arm_server/detected_touch_cloud", 10);
	
	
	//velocity publisher
	pub_base_velocity = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	
	//declare service for touching objects
	ros::ServiceServer service_touch = n.advertiseService("ispy/touch_object_service", touch_object_cb);
	
	//service for detecting when a human touches an object
	ros::ServiceServer service_detect = n.advertiseService("ispy/human_detect_touch_object_service", detect_touch_cb);
	
	//service for moving robot to different tables
	ros::ServiceServer service_face_table_one = n.advertiseService("ispy/face_table", face_table_cb);
	
	//service to toggle listening mode
	ros::ServiceServer service_listening = n.advertiseService("ispy/listening_mode_toggle", listen_mode_cb);
	
	//service to toggle waiting for touch mode
	ros::ServiceServer service_wait_touch = n.advertiseService("ispy/touch_waiting_mode_toggle", touch_wait_mode_cb);
	
	
	//clients
	client_start_change = n.serviceClient<std_srvs::Empty> ("/segbot_arm_table_change_detector/start");
	client_stop_change = n.serviceClient<std_srvs::Empty> ("/segbot_arm_table_change_detector/stop");
	client_joint_command = n.serviceClient<moveit_utils::MicoMoveitJointPose> ("/mico_jointpose_service");
	
	//store the home arm pose
	listenForArmData(40.0,2.0);
	home_pose = current_pose;
	
	
	//test moving to home
	moveToJointState(home_position);
	
	//register ctrl-c
	signal(SIGINT, sig_handler);

	//listen for messages, srvs, etc.
	
	//refresh rate
	double ros_rate = 40.0;
	ros::Rate r(ros_rate);

	//msg for publishing velocity commands
	geometry_msgs::TwistStamped v_msg;
	
	

	// Main loop:
	while (!g_caught_sigint && ros::ok())
	{
		//collect messages
		ros::spinOnce();
		
		//check if we're supposed to be moving as an indicator that the robot is listening
		if (is_listening){
			//update angle
			theta_angle += (2*PI)/(cycle_length * ros_rate);

			//compute z velocity and publish
			v_msg.twist.linear.z = z_vel_magnitude * cos(theta_angle);
			pub_velocity.publish(v_msg);
			
		}
		
		
		//sleep to maintain framerate
		r.sleep();
	}

	exit(1);
};
