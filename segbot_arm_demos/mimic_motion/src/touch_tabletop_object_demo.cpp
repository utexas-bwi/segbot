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

#include <geometry_msgs/TwistStamped.h>


#define PI 3.14159265

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//where we store results from calling the perception service
std::vector<PointCloudT::Ptr > detected_objects;
PointCloudT::Ptr cloud_plane (new PointCloudT);

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

void movePose(float d_z) {
  actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ac("/m1n6s200_driver/pose_action/tool_pose", true);

  kinova_msgs::ArmPoseGoal goalPose;

  // Set goal pose coordinates

  goalPose.pose.header.frame_id = "m1n6s200_link_base";
  
  ROS_INFO_STREAM(current_pose);

  goalPose.pose.pose.position.x = current_pose.pose.position.x;
  goalPose.pose.pose.position.y = current_pose.pose.position.y;
  goalPose.pose.pose.position.z = current_pose.pose.position.z + d_z;
  goalPose.pose.pose.orientation.x = current_pose.pose.orientation.x;
  goalPose.pose.pose.orientation.y = current_pose.pose.orientation.y;
  goalPose.pose.pose.orientation.z = current_pose.pose.orientation.z;
  goalPose.pose.pose.orientation.w = current_pose.pose.orientation.w;

  ROS_INFO_STREAM(goalPose);

  ac.waitForServer();
  ROS_INFO("Waiting for server.");
  //finally, send goal and wait
  ROS_INFO("Sending goal.");
  ac.sendGoal(goalPose);
  ac.waitForResult();

}

void moveToCurrentAngles(){
	actionlib::SimpleActionClient<kinova_msgs::ArmJointAnglesAction> ac("/m1n6s200_driver/joint_angles/arm_joint_angles", true);
	
	kinova_msgs::ArmJointAnglesGoal goalJoints;
	
	listenForArmData(30.0);
	
	goalJoints.angles.joint1 = current_state.position[0];
	goalJoints.angles.joint2 = current_state.position[1];
	goalJoints.angles.joint3 = current_state.position[2];
	goalJoints.angles.joint4 = current_state.position[3];
	goalJoints.angles.joint5 = current_state.position[4];
	goalJoints.angles.joint6 = current_state.position[5];
	
	ac.waitForServer();

    ac.sendGoal(goalJoints);

    ac.waitForResult();
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


bool moveToPose(geometry_msgs::PoseStamped g){
	actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ac("/m1n6s200_driver/pose_action/tool_pose", true);

	kinova_msgs::ArmPoseGoal goalPose;
  
 

	goalPose.pose = g;


	ROS_INFO_STREAM(goalPose);

	  ac.waitForServer();
	  ROS_DEBUG("Waiting for server.");
	  //finally, send goal and wait
	  ROS_INFO("Sending goal.");
	  ac.sendGoal(goalPose);
	  ac.waitForResult();
		
	return true;
}

moveit_msgs::GetPositionIK::Response computeIK(ros::NodeHandle n, geometry_msgs::PoseStamped p){
	ros::ServiceClient ikine_client = n.serviceClient<moveit_msgs::GetPositionIK> ("/compute_ik");
	
	
	moveit_msgs::GetPositionIK::Request ikine_request;
	moveit_msgs::GetPositionIK::Response ikine_response;
	ikine_request.ik_request.group_name = "arm";
	ikine_request.ik_request.pose_stamped = p;
	
	/* Call the service */
	if(ikine_client.call(ikine_request, ikine_response)){
		ROS_INFO("IK service call success:");
		ROS_INFO_STREAM(ikine_response);
	} else {
		ROS_INFO("IK service call FAILED. Exiting");
	}
	
	return ikine_response;
}

void spinSleep(double duration){
	int rateHertz = 40;
	
	ros::Rate r(rateHertz);
	for(int i = 0; i < (int)duration * rateHertz; i++) {
		
		
		ros::spinOnce();
		r.sleep();
	}
}

void updateFK(ros::NodeHandle n){
	ros::ServiceClient fkine_client = n.serviceClient<moveit_msgs::GetPositionFK> ("/compute_fk");
	
	moveit_msgs::GetPositionFK::Request fkine_request;
	moveit_msgs::GetPositionFK::Response fkine_response;

	
	//wait to get lates joint state values
	listenForArmData(30.0);
	sensor_msgs::JointState q_true = current_state;
	
	//Load request with the desired link
	fkine_request.fk_link_names.push_back("mico_end_effector");

	//and the current frame
	fkine_request.header.frame_id = "m1n6s200_link_base";

	//finally we let moveit know what joint positions we want to compute
	//in this case, the current state
	fkine_request.robot_state.joint_state = q_true;

	ROS_INFO("Making FK call");
 	if(fkine_client.call(fkine_request, fkine_response)){
 		pose_fk_pub.publish(fkine_response.pose_stamped.at(0));
 		ros::spinOnce();
 		current_moveit_pose = fkine_response.pose_stamped.at(0);
 		ROS_INFO("Call successful. Response:");
 		ROS_INFO_STREAM(fkine_response);
 	} else {
 		ROS_ERROR("Call failed. Terminating.");
 		//ros::shutdown();
 	}
 	
 	
 	//
}


// Blocking call for user input
void pressEnter(){
	std::cout << "Press the ENTER key to continue";
	while (std::cin.get() != '\n')
		std::cout << "Please press ENTER\n";
}


int selectObject(std::vector<PointCloudT::Ptr > candidates){
	//currently, we just pick the one with the most points
	int max_num_points = -1;
	int index = -1;
	
	for (unsigned int i = 0; i < candidates.size(); i ++){
		if ((int)candidates.at(i)->points.size() > max_num_points){
			max_num_points = (int)candidates.at(i)->points.size();
			index = (int)i;
			
		}
	}
	
	return index;
}

void moveToJointState(ros::NodeHandle n, sensor_msgs::JointState target){
	//check if this is specified just for the arm
	sensor_msgs::JointState q_target;
	if (target.position.size() != NUM_JOINTS_ARMONLY){
		//in this case, the first four values are for the base joints
		for (int i = 4; i < target.position.size(); i ++){
			q_target.position.push_back(target.position.at(i));
			q_target.name.push_back(target.name.at(i));
		}
		q_target.header = target.header;
	}
	else 
		q_target = target;
	
	ROS_INFO("Target joint state:");
	ROS_INFO_STREAM(q_target);
	
	moveit_utils::AngularVelCtrl::Request	req;
	moveit_utils::AngularVelCtrl::Response	resp;
	
	ros::ServiceClient ikine_client = n.serviceClient<moveit_utils::AngularVelCtrl> ("/angular_vel_control");
	
	req.state = q_target;
	
	pressEnter();
	
	if(ikine_client.call(req, resp)){
 		ROS_INFO("Call successful. Response:");
 		ROS_INFO_STREAM(resp);
 	} else {
 		ROS_ERROR("Call failed. Terminating.");
 		//ros::shutdown();
 	}
	
}

void moveToJointStateMoveIt(ros::NodeHandle n, geometry_msgs::PoseStamped p_target/*sensor_msgs::JointState q_target*/){
	moveit_utils::MicoMoveitCartesianPose::Request 	req;
	moveit_utils::MicoMoveitCartesianPose::Response res;
	
	req.target = p_target;
	
	ros::ServiceClient client = n.serviceClient<moveit_utils::MicoMoveitCartesianPose> ("/mico_cartesianpose_service");
	if(client.call(req, res)){
 		ROS_INFO("Call successful. Response:");
 		ROS_INFO_STREAM(res);
 	} else {
 		ROS_ERROR("Call failed. Terminating.");
 		//ros::shutdown();
 	}
	
	
}

void cartesianVelocityMove(double dx, double dy, double dz, double duration){
	int rateHertz = 40;
	kinova_msgs::PoseVelocity velocityMsg;
	
	ros::Rate r(rateHertz);
	for(int i = 0; i < (int)duration * rateHertz; i++) {
		
		
		velocityMsg.twist_linear_x = dx;
		velocityMsg.twist_linear_y = dy;
		velocityMsg.twist_linear_z = dz;
		
		velocityMsg.twist_angular_x = 0.0;
		velocityMsg.twist_angular_y = 0.0;
		velocityMsg.twist_angular_z = 0.0;
		
		
		pub_velocity.publish(velocityMsg);
		ROS_INFO("Published cartesian vel. command");
		r.sleep();
	}
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

void lift(ros::NodeHandle n, double x){
	listenForArmData(30.0);
	
	geometry_msgs::PoseStamped p_target = current_pose;
	
	p_target.pose.position.z += x;
	moveToJointStateMoveIt(n,p_target);
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
	
	//for each object, compute the touch pose; also, extract the highest point from the table
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
	
	// now, touch each object
	for (int i = 0; i < touch_poses.size(); i ++){
		geometry_msgs::PoseStamped touch_pose_i = touch_poses.at(i);
		
		//before we get there, first go a bit above
		double z_above = highest_z+0.2;
		
		geometry_msgs::PoseStamped touch_approach = touch_pose_i;
		touch_approach.pose.position.z = z_above;
		
		pose_pub.publish(touch_approach);
		moveToPoseCarteseanVelocity(n,touch_approach);
		
		pose_pub.publish(touch_pose_i);
		moveToPoseCarteseanVelocity(n,touch_pose_i);
		
		pose_pub.publish(touch_approach);
		moveToPoseCarteseanVelocity(n,touch_approach);
	}
	
	moveToPoseCarteseanVelocity(n,start_pose);

	
	
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
