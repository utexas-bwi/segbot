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

//tf stuff
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>


//actions
#include <actionlib/client/simple_action_client.h>
#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"
#include "kinova_msgs/ArmJointAnglesAction.h"
#include "kinova_msgs/PoseVelocity.h"



#include "agile_grasp/Grasps.h"

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

//some defines related to filtering candidate grasps
#define MAX_DISTANCE_TO_PLANE 0.075

sensor_msgs::JointState current_state;

kinova_msgs::FingerPosition current_finger;
geometry_msgs::PoseStamped current_pose;
bool heardPose = false;
bool heardJoinstState = false;
bool heardFingers = false;

double total_grav_free_effort = 0;
double total_delta;
double delta_effort[6];

geometry_msgs::PoseStamped current_moveit_pose;


//publishers
ros::Publisher pub_velocity;
ros::Publisher cloud_pub;
ros::Publisher cloud_grasp_pub;
ros::Publisher pose_array_pub;
ros::Publisher pose_pub;
ros::Publisher pose_fk_pub;
 
sensor_msgs::PointCloud2 cloud_ros;





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
	
	//compute the change in efforts if we had already heard the last one
	if (heardJoinstState){
		for (int i = 0; i < 6; i ++){
			delta_effort[i] = fabs(input->effort[i]-current_state.effort[i]);
		}
	}
	
	total_grav_free_effort = 0.0;
	for (int i = 0; i < 6; i ++){
		if (current_state.effort[i] < 0.0)
			total_grav_free_effort -= (current_state.effort[i]);
		else 
			total_grav_free_effort += (current_state.effort[i]);
	}
	
	//calc total change in efforts
	total_delta = delta_effort[0]+delta_effort[1]+delta_effort[2]+delta_effort[3]+delta_effort[4]+delta_effort[5];
	
	heardJoinstState=true;
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
  heardFingers = true;
}



void listenForArmData(float rate){
	heardPose = false;
	heardJoinstState = false;
	heardFingers = false;
	ros::Rate r(rate);
	
	while (ros::ok()){
		ros::spinOnce();
		
		if (heardPose && heardJoinstState && heardFingers)
			return;
		
		r.sleep();
	}
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


int selectObjectToGrasp(std::vector<PointCloudT::Ptr > candidates){
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


void moveToPoseCarteseanVelocity(geometry_msgs::PoseStamped pose_st, float effort_theta, float timeOutSeconds, float m_factor){
	listenForArmData(30.0);
	
	int rateHertz = 40;
	kinova_msgs::PoseVelocity velocityMsg;
	
	
	ros::Rate r(rateHertz);
	float elapsedTime = 0;
	
	float theta = 0.05;
	
	float constant_m = m_factor;
	
	ROS_INFO("Starting movement...");
	
	float last_dx = -1;
	float last_dy = -1;
	float last_dz = -1;
	
	int timeout_counter = 0;
	
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
			ROS_INFO("Reached target");
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
		elapsedTime+=1.0/(float)rateHertz;
		
		if (effort_theta > 0){
			ROS_INFO("total_delta = %f",total_delta);
			if (heardJoinstState){
				if (total_delta > effort_theta){
					//we hit something, break;
					ROS_WARN("[ispy_arm_server.cpp] contact detecting during cartesean velocity movement.");
					break;
				}
			}
		}
		
		if (timeOutSeconds > 0 && elapsedTime > timeOutSeconds){
			ROS_WARN("Timeout!");
			break;
		}
	}
	
	ROS_INFO("Ending movement...");

}


bool moveToPoseMico(geometry_msgs::PoseStamped g){
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

void waitForForce(float force_threshold){
	int rateHertz = 40;
	ros::Rate r(rateHertz);
	
	while (ros::ok()){
		ROS_INFO("delta = %f",total_delta);
		
		if (total_delta > fabs(force_threshold)){
			//we hit something, break;
			ROS_WARN("Force detected");
			break;
		}
		
		ros::spinOnce();
		r.sleep();
	}
}


void moveToPoseMoveIt(ros::NodeHandle n, geometry_msgs::PoseStamped p_target/*sensor_msgs::JointState q_target*/){
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
	
	/*moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;   
    group.setPlanningTime(5.0); //10 second maximum for collision computation*/

	
	
	/*moveit_utils::MicoMoveitJointPose::Request req;
	moveit_utils::MicoMoveitJointPose::Response res;
	*/
	/*for(int i = 0; i < NUM_JOINTS_ARMONLY; i++){
        switch(i) {
            case 0  :    req.target.joint1 = q_target.position[0]; break;
            case 1  :    req.target.joint2 = q_target.position[1]; break;
            case 2  :    req.target.joint3 = q_target.position[2]; break;
            case 3  :    req.target.joint4 = q_target.position[3]; break;
            case 4  :    req.target.joint5 = q_target.position[4]; break;
            case 5  :    req.target.joint6 = q_target.position[5]; break;
        }
	//ROS_INFO("Requested angle: %f", q_vals.at(i));
    }
	ros::ServiceClient client = n.serviceClient<moveit_utils::MicoMoveitJointPose> ("/mico_jointpose_service");
	if(client.call(req, res)){
 		ROS_INFO("Call successful. Response:");
 		ROS_INFO_STREAM(res);
 	} else {
 		ROS_ERROR("Call failed. Terminating.");
 		//ros::shutdown();
 	}*/
}


int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "candy_handover_demo");
	
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
	pose_fk_pub = n.advertise<geometry_msgs::PoseStamped>("/agile_grasp_demo/pose_fk_out", 10);
	
	//debugging publisher
	cloud_pub = n.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud_debug", 10);
	cloud_grasp_pub = n.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud", 10);
	
	//used to compute transfers
	tf::TransformListener listener;
	
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
		ROS_INFO("[agile_grasp_demo.cpp] Received Response from tabletop_object_detection_service");
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
	
	//step 3: select which object to grasp
	int selected_object = selectObjectToGrasp(detected_objects);
	
	//publish object to find grasp topic
	pcl::PCLPointCloud2 pc_target;
	pcl::toPCLPointCloud2(*detected_objects.at(selected_object),pc_target);
	pcl_conversions::fromPCL(pc_target,cloud_ros);
	cloud_pub.publish(cloud_ros);
	
	
	//make sure we're working with the correct tool pose
	listenForArmData(30.0);
	ROS_INFO("[agile_grasp_demo.cpp] Heard arm pose.");
	
	updateFK(n);
	
	
	//move above object
	
	//Compute target pose above the object and transform it
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*detected_objects.at(selected_object), centroid);
	geometry_msgs::PoseStamped abovePose;
	abovePose.header.stamp = ros::Time(0);
	abovePose.header.frame_id =cloud_ros.header.frame_id;
	abovePose.pose.position.x=centroid(0);
	abovePose.pose.position.y=centroid(1);
	abovePose.pose.position.z=centroid(2);
	abovePose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);


	//wait for transform from visual space to arm space
	listener.waitForTransform(cloud_ros.header.frame_id, "m1n6s200_link_base", ros::Time(0), ros::Duration(3.0));	
	listener.transformPose("m1n6s200_link_base", abovePose, abovePose);
	abovePose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw((165 / 360.0) * (2*PI),0,(90 / 360.0) * (2*PI));
	abovePose.pose.position.z += 0.2;

	pose_pub.publish(abovePose);
	pressEnter();
	
	bool hasCandy = false;
	
	geometry_msgs::PoseStamped abovePoseR;
	while (hasCandy == false){
		
		//move above the object
		//add a tiny bit of random xy noise
		abovePoseR = abovePose;
		float rx = -0.02+0.04*static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		float ry = -0.02+0.04*static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		abovePoseR.pose.position.x += rx;
		abovePoseR.pose.position.y += ry;
		
		moveToPoseCarteseanVelocity(abovePoseR,-1.0,-1.0,3.0);
		
		//open fingers
		moveFinger(2500);
		//pressEnter();
		
		//now lower into the box
		abovePoseR.pose.position.z -= 0.24;	
		moveToPoseCarteseanVelocity(abovePoseR,0.7,4.0,0.5);
		ROS_INFO("Moving up a bit");
		//pressEnter();
		
		//now close fingers
		/*abovePoseR.pose.position.z += 0.125;	
		moveToPoseCarteseanVelocity(abovePoseR,false,4.0);
		pressEnter();*/

		moveFinger(7000);
		spinSleep(1.0);
		//pressEnter();
	
		//check finger status
		listenForArmData(40);
		//pressEnter();

		ROS_INFO("Fingers: %f, %f",current_finger.finger1,current_finger.finger2);
		if (current_finger.finger1 < 7000 && current_finger.finger2 < 7000){
			hasCandy = true;
		}
	}
	
	//lift
	listenForArmData(40);
	abovePoseR = current_pose;
	abovePoseR.pose.position.z += 0.05;	
	//moveToPoseMico(abovePose);
	//moveToPoseCarteseanVelocity(abovePose,false,6.0);
	moveToPoseMoveIt(n,abovePoseR);
	
	//pressEnter();

	
	//go side way
	abovePose.pose.position.y -= 0.4;	
	moveToPoseCarteseanVelocity(abovePose,-1.0,6.0,3.0);
	//pressEnter();

	//wait for somoene to grab the candy
	spinSleep(1.0);
	waitForForce(0.4);

	//release
	moveFinger(1300);
	//pressEnter();


	/*while (ros::ok()){
		float roll, pitch, yaw;
		
		ROS_INFO("Enter roll (degrees):");
		std::cin >> roll;
		ROS_INFO("Enter pitch (degrees):");
		std::cin >> pitch;
		ROS_INFO("Enter yaw (degrees):");
		std::cin >> yaw;
		
		ROS_INFO("You entered: %f, %f, %f",roll,pitch,yaw);
		
		//convert degrees to radians
		roll = (roll / 360.0) * (2*PI);
		pitch = (pitch / 360.0) * (2*PI);
		yaw = (yaw / 360.0) * (2*PI);
		
		abovePose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
		
		//publish individual pose
		pose_pub.publish(abovePose);
		
		pressEnter();
		
	}*/
			
			
	
	//moveToJointStateMoveIt(n,grasp_commands.at(min_diff_index).approach_pose);
	//pressEnter();
	
	//open fingers
	//pressEnter();
	//moveFinger(100);


	//sleep(1.0);
	//pose_pub.publish(grasp_commands.at(min_diff_index).grasp_pose);
	/*std::cout << "Press '1' to move to approach pose or Ctrl-z to quit..." << std::endl;		
	std::cin >> in;*/
	
	//moveToJointStateMoveIt(n,grasp_commands.at(min_diff_index).grasp_pose);
	//spinSleep(3.0);
	
	//listenForArmData(30.0);
	//close fingers
	//pressEnter();
	//moveFinger(7200);
	
	
	//lift for a while
	//pressEnter();
	//cartesianVelocityMove(0,0,0.2,1.0);
	
	/*lift(n,0.1);
	lift(n,-0.09);
	spinSleep(3.0);
	moveFinger(100);
	lift(n,0.1);*/
	
	return 0;
}
