#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>


#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/JointState.h>
#include <kinova_msgs/PoseVelocity.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

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

#include <moveit_msgs/DisplayRobotState.h>
// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>


#define PI 3.14159265

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//where we store results from calling the perception service
std::vector<PointCloudT::Ptr > detected_objects;
PointCloudT::Ptr cloud_plane (new PointCloudT);
PointCloudT::Ptr cloud_change (new PointCloudT);
bool new_change_cloud_detected = false;

boost::mutex cloud_mutex;


using namespace std;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

#define FINGER_FULLY_OPENED 6
#define FINGER_FULLY_CLOSED 7300

#define NUM_JOINTS_ARMONLY 6
#define NUM_JOINTS 8 //6+2 for the arm


sensor_msgs::JointState current_state;
kinova_msgs::FingerPosition current_finger;
geometry_msgs::PoseStamped current_pose;
bool heardPose = false;
bool heardJoinstState = false;

geometry_msgs::PoseStamped current_moveit_pose;


//publishers
ros::Publisher pub_velocity;
ros::Publisher cloud_pub;
ros::Publisher pose_pub;
ros::Publisher pose_fk_pub;
 
sensor_msgs::PointCloud2 cloud_ros;

bool heardGrasps = false;


/* what happens when ctr-c is pressed */
void sig_handler(int sig)
{
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
  //ROS_INFO_STREAM(current_state);
}

//Joint state cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
  current_pose = msg;
  heardPose = true;
  //  ROS_INFO_STREAM(current_pose);
}

//Joint state cb
void fingers_cb (const kinova_msgs::FingerPosition msg) {
  current_finger = msg;
}


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



// Range = [6, 7300] ([open, close])
void moveFinger(int finger_value) {
    actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> ac("/m1n6s200_driver/fingers_action/finger_positions", true);

    kinova_msgs::SetFingersPositionGoal goalFinger;

    goalFinger.fingers.finger1 = finger_value;
    goalFinger.fingers.finger2 = finger_value;
    // Not used for our arm
    goalFinger.fingers.finger3 = 0;
    
    ac.waitForServer();

    ac.sendGoal(goalFinger);

    ac.waitForResult();
}

double angular_difference(geometry_msgs::Quaternion c,geometry_msgs::Quaternion d){
	Eigen::Vector4f dv;
	dv[0] = d.w; dv[1] = d.x; dv[2] = d.y; dv[3] = d.z;
	Eigen::Matrix<float, 3,4> inv;
	inv(0,0) = -c.x; inv(0,1) = c.w; inv(0,2) = -c.z; inv(0,3) = c.y;
	inv(1,0) = -c.y; inv(1,1) = c.z; inv(1,2) = c.w;	inv(1,3) = -c.x;
	inv(2,0) = -c.z; inv(2,1) = -c.y;inv(2,2) = c.x;  inv(2,3) = c.w;
	
	Eigen::Vector3f m = inv * dv * -2.0;
	return m.norm();
}


void spinSleep(double duration){
	int rateHertz = 40;
	
	ros::Rate r(rateHertz);
	for(int i = 0; i < (int)duration * rateHertz; i++) {
		
		
		ros::spinOnce();
		r.sleep();
	}
}



// Blocking call for user input
void pressEnter(){
	std::cout << "Press the ENTER key to continue";
	while (std::cin.get() != '\n')
		std::cout << "Please press ENTER\n";
}


geometry_msgs::PoseStamped createTouchPose(PointCloudT::Ptr blob, Eigen::Vector4f plane_coefficients,std::string frame_id){
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
			
	//decide on orientation
	pose_st.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14/2,0,3.14/2);
	
	//add a bit of z 
	pose_st.pose.position.z+=0.045;
	
	ROS_INFO("Touch pose:");
	ROS_INFO_STREAM(pose_st);
	
	return pose_st;
}





void change_cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	cloud_mutex.lock ();
	
	//convert to PCL format
	pcl::fromROSMsg (*input, *cloud_change);



	new_change_cloud_detected = true;
	
	cloud_mutex.unlock ();
}

void moveToPoseCarteseanVelocity(ros::NodeHandle n, geometry_msgs::PoseStamped pose_st){
	listenForArmData(30.0);
	
	int rateHertz = 40;
	kinova_msgs::PoseVelocity velocityMsg;
	
	float duration = 3.0;
	
	ros::Rate r(rateHertz);
	
	float theta = 0.05;
	
	float constant_m = 2.0;
	
	while (true){
		
		float dx = constant_m*( - current_pose.pose.position.x + pose_st.pose.position.x );
		float dy = constant_m*(- current_pose.pose.position.y + pose_st.pose.position.y);
		float dz = constant_m*(- current_pose.pose.position.z + pose_st.pose.position.z);
		
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
		ROS_INFO("Published cartesian vel. command");
		r.sleep();
		
		
	}
}


void startChangeDetection(ros::NodeHandle n){
	
	std_srvs::Empty::Request req;
	std_srvs::Empty::Response res;

	ros::ServiceClient client = n.serviceClient<std_srvs::Empty> ("/segbot_arm_table_change_detector/start");
	if(client.call(req, res)){
 		ROS_INFO("Call to change detect node successful. Response:");
 		ROS_INFO_STREAM(res);
 	} else {
 		ROS_ERROR("Call to change detect node failed. Terminating.");
 		//ros::shutdown();
 	}
	
}


void stopChangeDetection(ros::NodeHandle n){
	
	std_srvs::Empty::Request req;
	std_srvs::Empty::Response res;

	ros::ServiceClient client = n.serviceClient<std_srvs::Empty> ("/segbot_arm_table_change_detector/stop");
	if(client.call(req, res)){
 		ROS_INFO("Call to change detect node successful. Response:");
 		ROS_INFO_STREAM(res);
 	} else {
 		ROS_ERROR("Call to change detect node failed. Terminating.");
 		//ros::shutdown();
 	}
	
}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "touch_demo");
	
	ros::NodeHandle n;

	//create subscriber to joint angles
	ros::Subscriber sub_angles = n.subscribe ("/m1n6s200_driver/out/joint_state", 1, joint_state_cb);

	//create subscriber to tool position topic
	ros::Subscriber sub_tool = n.subscribe("/m1n6s200_driver/out/tool_pose", 1, toolpos_cb);

	//subscriber for fingers
	ros::Subscriber sub_finger = n.subscribe("/m1n6s200_driver/out/finger_position", 1, fingers_cb);
	 
	//subscriber for change cloud
	ros::Subscriber sub_change_cloud = n.subscribe("/segbot_arm_table_change_detector/cloud",1,change_cloud_cb); 
	
	//publish velocities
	pub_velocity = n.advertise<kinova_msgs::PoseVelocity>("/m1n6s200_driver/in/cartesian_velocity", 10);
	
	//publish pose 
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/agile_grasp_demo/pose_out", 10);
	
	//cloud publisher
	cloud_pub = n.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud", 10);
	
	
	//register ctrl-c
	signal(SIGINT, sig_handler);

	
	//user input
    char in;
	
	ROS_INFO("Demo starting...");
	pressEnter();
	
	//step 1: query table_object_detection_node to segment the blobs on the table
	ros::ServiceClient client_tabletop_perception = n.serviceClient<segbot_arm_perception::TabletopPerception>("tabletop_object_detection_service");
	segbot_arm_perception::TabletopPerception srv; //the srv request is just empty
	if (client_tabletop_perception.call(srv))
	{
		ROS_INFO("[touch_tabletop_object_demo.cpp] Received Response from tabletop_object_detection_service");
	}
	else
	{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
	
	//step 2: extract the data from the response
	detected_objects.clear();
	for (unsigned int i = 0; i < srv.response.cloud_clusters.size(); i++){
		PointCloudT::Ptr object_i (new PointCloudT);
		pcl::PCLPointCloud2 pc_i;
		pcl_conversions::toPCL(srv.response.cloud_clusters.at(i),pc_i);
		pcl::fromPCLPointCloud2(pc_i,*object_i);
		detected_objects.push_back(object_i);
	}
	
	if (detected_objects.size() == 0){
		ROS_WARN("[agile_grasp_demo.cpp] No objects detected...aborting.");
		return 1;
	}
	
	Eigen::Vector4f plane_coef_vector;
	for (int i = 0; i < 4; i ++)
		plane_coef_vector(i)=srv.response.cloud_plane_coef[i];
	
	//for each object, compute the touch pose (i.e., the top point of the object); also, extract the highest point from the table
	std::vector<geometry_msgs::PoseStamped> touch_poses;
	double highest_z = 0.0;
	for (int i = 0; i < detected_objects.size(); i++){
		//generate touch pose for the object
		geometry_msgs::PoseStamped touch_pose_i = createTouchPose(detected_objects.at(i),plane_coef_vector,
												srv.response.cloud_clusters.at(0).header.frame_id);
		
		if (touch_pose_i.pose.position.z > 0.05)
			touch_poses.push_back(touch_pose_i);
		
		if (touch_pose_i.pose.position.z > highest_z){
			highest_z = touch_pose_i.pose.position.z ;
		}
	}
	
	//store current pose
	listenForArmData(10.0);
	geometry_msgs::PoseStamped start_pose = current_pose;
	
	//start change detection service
	startChangeDetection(n);
	
	//now detect change close to objects
	ros::Rate r(10.0);
	
	while (ros::ok()){
		ros::spinOnce();
		
		if (new_change_cloud_detected){
			
			//first, Z filter on the cloud
			pcl::PassThrough<PointT> pass;
			pass.setInputCloud (cloud_change);
			pass.setFilterFieldName ("z");
			pass.setFilterLimits (0.0, 1.15);
			pass.filter (*cloud_change);
			
			
			
			new_change_cloud_detected = false;
		}
		
		r.sleep();
	}
	
	//step 3: select which object to grasp
	/*int selected_object = selectObject(detected_objects); 
	
	//publish object to find grasp topic
	pcl::PCLPointCloud2 pc_target;
	pcl::toPCLPointCloud2(*detected_objects.at(selected_object),pc_target);
	pcl_conversions::fromPCL(pc_target,cloud_ros);
	cloud_pub.publish(cloud_ros);

	//generate touch pose for the object
	geometry_msgs::PoseStamped touch_pose = createTouchPose(detected_objects.at(selected_object),plane_coef_vector,
												srv.response.cloud_clusters.at(0).header.frame_id);
	
	pose_pub.publish(touch_pose);
	
	moveToPoseCarteseanVelocity(n,touch_pose);*/
 
	return 0;
}
