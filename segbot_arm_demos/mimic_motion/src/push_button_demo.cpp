#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>

#include <sensor_msgs/JointState.h>
#include <kinova_msgs/PoseVelocity.h>
#include <std_msgs/Float32.h>

#include "segbot_arm_perception/ButtonDetection.h"

//actions
#include <actionlib/client/simple_action_client.h>
#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>

// PCL specific includes
#include <pcl/conversions.h>
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

#include <moveit_msgs/DisplayRobotState.h>
// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>

#define PI 3.14159265

using namespace std;

#define FINGER_FULLY_OPENED 6
#define FINGER_FULLY_CLOSED 7300

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
//PointCloudT::Ptr pcl_cloud (new PointCloudT);

pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

sensor_msgs::JointState current_state;
kinova_msgs::FingerPosition current_finger;
geometry_msgs::PoseStamped current_pose;

ros::Publisher pub_velocity;

bool button_pose_received = false;
geometry_msgs::PoseStamped current_button_pose;
 

ros::Publisher pose_pub;

//Joint state cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
  current_state = *input;
  //ROS_INFO_STREAM(current_state);
}

//Joint state cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
  current_pose = msg;
  //  ROS_INFO_STREAM(current_pose);
}

//Joint state cb
void fingers_cb (const kinova_msgs::FingerPosition msg) {
  current_finger = msg;
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

void pushButton() {
	double timeoutSeconds = 1.85;
	int rateHertz = 100;
	kinova_msgs::PoseVelocity velocityMsg;
	
	ros::Rate r(rateHertz);
	for(int i = 0; i < (int)timeoutSeconds * rateHertz; i++) {
		
		
		velocityMsg.twist_linear_x = 0;
		velocityMsg.twist_linear_y = 0.0;
		velocityMsg.twist_linear_z = -0.125;
		
		velocityMsg.twist_angular_x = 0.0;
		velocityMsg.twist_angular_y = 0.0;
		velocityMsg.twist_angular_z = 0.0;
		
		
		pub_velocity.publish(velocityMsg);
		
		r.sleep();
	}
	

	for(int i = 0; i < (int)3.0 * rateHertz; i++) {
		
		
		velocityMsg.twist_linear_x = 0.0;
		velocityMsg.twist_linear_y = -0.125;
		velocityMsg.twist_linear_z = 0.2;
		
		velocityMsg.twist_angular_x = 0.0;
		velocityMsg.twist_angular_y = 0.0;
		velocityMsg.twist_angular_z = 0.0;
		
		
		pub_velocity.publish(velocityMsg);
		
		r.sleep();
	}
}


void moveAboveButton(){
	
	/*x: 0.600193565311
    y: -0.35348613384
    z: 0.630947815735
    w: 0.341643222034
*/
	actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ac("/m1n6s200_driver/pose_action/tool_pose", true);

	kinova_msgs::ArmPoseGoal goalPose;

	// Set goal pose coordinates

	goalPose.pose.header.frame_id = "m1n6s200_link_base";
  
 

	goalPose.pose.pose.position.x = current_button_pose.pose.position.x;//+0.09;
	goalPose.pose.pose.position.y = current_button_pose.pose.position.y;//-0.03;
	goalPose.pose.pose.position.z = current_button_pose.pose.position.z;// + 0.125;
	
	goalPose.pose.pose.orientation = current_button_pose.pose.orientation;
	
	//goalPose.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14,0,0);//vertcal pose (fingers down, palm aligned with the x direction
	//goalPose.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14,3.14/1.9,0);//points fingers to the left with the knuckles facing (up/down);
	

	ROS_INFO_STREAM(goalPose);

  ac.waitForServer();
  ROS_DEBUG("Waiting for server.");
  //finally, send goal and wait
  ROS_INFO("Sending goal.");
  ac.sendGoal(goalPose);
  ac.waitForResult();
}



void waitForButtonPose(ros::NodeHandle n){
	//step 1: call the button detection service and get the response
	ros::ServiceClient client = n.serviceClient<segbot_arm_perception::ButtonDetection>("segbot_arm_button_detector/detect");
	segbot_arm_perception::ButtonDetection srv;
	
	if(client.call(srv)) {
		if(srv.response.button_found == false)
			ros::shutdown();
			
		//step 2. convert the cloud in the response to PCL format
		sensor_msgs::PointCloud2 input = srv.response.cloud_button;
		pcl::fromROSMsg(input, pcl_cloud);
		
		//step 3. create a pose with x y z set to the center of point cloud

		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(pcl_cloud, centroid);

		//transforms the pose into /map frame
		geometry_msgs::Pose pose_i;
		pose_i.position.x=centroid(0);
		pose_i.position.y=centroid(1);
		pose_i.position.z=centroid(2);
		pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-3.14/2);

		geometry_msgs::PoseStamped stampedPose;

		stampedPose.header.frame_id = pcl_cloud.header.frame_id;
		stampedPose.header.stamp = ros::Time(0);
		stampedPose.pose = pose_i;

		tf::TransformListener listener; 

		//step 4. transform the pose into Mico API origin frame of reference
		geometry_msgs::PoseStamped stampOut;
		listener.waitForTransform(pcl_cloud.header.frame_id, "m1n6s200_link_base", ros::Time(0), ros::Duration(3.0));
		listener.transformPose("m1n6s200_link_base", stampedPose, stampOut);

		ROS_INFO("[push_button_demo] publishing pose...");

		//stampOut.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,-3.14/2,0);
		
		//roll rotates around x axis
		
		stampOut.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,3.14/2.0,0.0);
		stampOut.pose.position.z+=0.125;
		stampOut.pose.position.x+=0.09;
		
		ROS_INFO_STREAM(stampOut);
		
		pose_pub.publish(stampOut);

		ros::spinOnce();
		
		//store pose
		current_button_pose = stampOut;
	
	
	
		//step 4.5. adjust post xyz and/or orientation so that the pose is above the button and oriented correctly
	
		//step 5. publish the pose			
	}
}


void moveToPoseMoveIt(ros::NodeHandle n, geometry_msgs::PoseStamped p_target){
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

// Blocking call for user input
void pressEnter(){
	std::cout << "Press the ENTER key to continue";
	while (std::cin.get() != '\n')
		std::cout << "Please press ENTER\n";
}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "push_button_demo");

	ros::NodeHandle n;

    //create subscriber to joint angles
    ros::Subscriber sub_angles = n.subscribe ("/m1n6s200_driver/out/joint_state", 1, joint_state_cb);

	//create subscriber to tool position topic
	ros::Subscriber sub_tool = n.subscribe("/m1n6s200_driver/out/tool_pose", 1, toolpos_cb);

	//subscriber for fingers
	ros::Subscriber sub_finger = n.subscribe("/m1n6s200_driver/out/finger_position", 1, fingers_cb);
  
  
	//publish velocities
	pub_velocity = n.advertise<kinova_msgs::PoseVelocity>("/m1n6s200_driver/in/cartesian_velocity", 10);


	//button position publisher
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/push_button_demo/pose", 10);

	ROS_INFO("Demo starting...Move the arm to a 'ready' position that does not occlude the table.");
	pressEnter();
	
	//listen for arm messages 
	//TO DO: store current joint values and current end effector pose
	
	//close fingers
	moveFinger(7300);

	//Step 1: listen to the button pose
	waitForButtonPose(n);
	
	//Step 3: move above the button
	moveAboveButton();
	
	//step 2. if button was found, we will call the MoveIt! planner to move the arm to the pose
	// also, we will have to call inverse kinematics 
	pressEnter();
	pushButton();
	
	//TO DO: move back to the 'ready' joint position or tool position
	
	
	//ros::spin();
	
	/*ROS_INFO("Button detected at: %f, %f, %f",current_button_pose.pose.position.x,current_button_pose.pose.position.y,current_button_pose.pose.position.z);

	//Step 2: close fingers
	moveFinger(7300);
	
	
	
	pushButton();*/

  /*unsigned int finger_open_close_toggle = 0;
  while (ros::ok()) {
      ros::spinOnce();
      ros::Duration(5).sleep();
      ROS_INFO("Iteration = %d", finger_open_close_toggle);
      if (finger_open_close_toggle % 2 == 0) {
          moveFinger(FINGER_FULLY_OPENED);
          ROS_INFO("Opening...");
      } else {
          moveFinger(FINGER_FULLY_CLOSED);
          ROS_INFO("Closing...");
      }
      finger_open_close_toggle++;
  }*/
  
  /*for (int i = 0; i < 20; i ++){
	  ros::spinOnce();
      ros::Duration(0.05).sleep();
  }*/
  
  //movePose(0.05);
  
   /*for (int i = 0; i < 20; i ++){
	  ros::spinOnce();
      ros::Duration(0.05).sleep();
  }*/
  
  //moveArmVelocity();
  //movePose(-0.05);
//  moveFinger(int finger_value);
  return 0;
}
