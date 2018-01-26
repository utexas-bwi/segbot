#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */


#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/grasp_utils.h>


//actions
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"
#include "kinova_msgs/ArmJointAnglesAction.h"

//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"

#include <moveit_msgs/DisplayRobotState.h>
// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>

#include <kinova_msgs/PoseVelocity.h>
#include <geometry_msgs/PoseArray.h>

//the action definition
#include "segbot_arm_manipulation/TabletopGraspAction.h"

//tf stuff
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#define FINGER_FULLY_OPENED 6
#define FINGER_FULLY_CLOSED 7300

#define NUM_JOINTS_ARMONLY 6
#define NUM_JOINTS 8 //6+2 for the arm

//some defines related to filtering candidate grasps
#define MIN_DISTANCE_TO_PLANE 0.05

#define HAND_OFFSET_GRASP -0.02
#define HAND_OFFSET_APPROACH -0.13

//used to decide if someone is pulling an object from the arm
#define FORCE_HANDOVER_THRESHOLD 3.0

//used when deciding whether a pair of an approach pose and a grasp pose are good;
//if the angular difference in joint space is too big, this means that the robot 
//cannot directly go from approach to grasp pose (so we filter those pairs out)
#define ANGULAR_DIFF_THRESHOLD 3.0


//the individual action server
//#include "segbot_arm_manipulation/tabletop_grasp_action.h"

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;




class TabletopGraspActionServer
{
protected:
	
	ros::NodeHandle nh_;
	// NodeHandle instance must be created before this line. Otherwise strange error may occur.
	actionlib::SimpleActionServer<segbot_arm_manipulation::TabletopGraspAction> as_; 
	std::string action_name_;
	// create messages that are used to published feedback/result
	segbot_arm_manipulation::TabletopGraspFeedback feedback_;
	segbot_arm_manipulation::TabletopGraspResult result_;
	//segbot_arm_manipulation::TabletopGraspGoal goal_;
  
	
	sensor_msgs::JointState current_state;
	kinova_msgs::FingerPosition current_finger;
	geometry_msgs::PoseStamped current_pose;
	bool heardPose;
	bool heardJoinstState; 
	
	bool heardGrasps;
	agile_grasp::Grasps current_grasps;

	ros::Publisher pub_velocity;
	ros::Publisher cloud_pub;
	ros::Publisher cloud_grasp_pub;
	ros::Publisher pose_array_pub;
	ros::Publisher pose_pub;
	ros::Publisher pose_fk_pub;

	std::vector<PointCloudT::Ptr > detected_objects;
	
	//used to compute transforms
	tf::TransformListener listener;
	
	bool heard_string;
	ros::Subscriber sub_string_;
	
	
	//subscribers -- in an action server, these have to be class members
	ros::Subscriber sub_angles;
	ros::Subscriber sub_torques;
	ros::Subscriber sub_tool;
	ros::Subscriber sub_finger;
	ros::Subscriber sub_grasps;
	
public:

  TabletopGraspActionServer(std::string name) :
	as_(nh_, name, boost::bind(&TabletopGraspActionServer::executeCB, this, _1), false),
    action_name_(name)
  {
	
	
	
	heardPose = false;
	heardJoinstState = false;
	heardGrasps = false;

	//create subscriber to joint angles
	sub_angles = nh_.subscribe ("/m1n6s200_driver/out/joint_state", 1, &TabletopGraspActionServer::joint_state_cb, this);


	//create subscriber to tool position topic
	sub_tool = nh_.subscribe("/m1n6s200_driver/out/tool_pose", 1, &TabletopGraspActionServer::toolpos_cb, this);

	//subscriber for fingers
	sub_finger = nh_.subscribe("/m1n6s200_driver/out/finger_position", 1, &TabletopGraspActionServer::fingers_cb, this);
	  
	//subscriber for grasps
	sub_grasps = nh_.subscribe("/find_grasps/grasps_handles",1, &TabletopGraspActionServer::grasps_cb,this);  
	  
	//publish velocities
	pub_velocity = nh_.advertise<kinova_msgs::PoseVelocity>("/m1n6s200_driver/in/cartesian_velocity", 10);
	
	//publish pose array
	pose_array_pub = nh_.advertise<geometry_msgs::PoseArray>("/agile_grasp_demo/pose_array", 10);
	
	//publish pose 
	pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/agile_grasp_demo/pose_out", 10);
	pose_fk_pub = nh_.advertise<geometry_msgs::PoseStamped>("/agile_grasp_demo/pose_fk_out", 10);
	
	//debugging publisher
	cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud_debug", 10);
	cloud_grasp_pub = nh_.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud", 10);
	
    ROS_INFO("Starting grasp action server...");
    
    as_.start();
  }

  ~TabletopGraspActionServer(void)
  {
  }
  
	void test_string_cb(const std_msgs::String& msg){
		ROS_INFO("Received string!");
		heard_string = true;
	}
  
  //Joint state cb
	void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
		if (input->position.size() == NUM_JOINTS){
			current_state = *input;
			heardJoinstState = true;
		}
	}
	

	//tool position cb
	void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
	  current_pose = msg;
	  heardPose = true;
	  
	  /*tf::Quaternion q(current_pose.pose.orientation.x, 
								current_pose.pose.orientation.y, 
								current_pose.pose.orientation.z, 
								current_pose.pose.orientation.w);
		tf::Matrix3x3 m(q);
		
		double r, p, y;
		m.getRPY(r, p, y);*/
	  //ROS_INFO("RPY: %f %f %f",r,p,y);
	  //  ROS_INFO_STREAM(current_pose);
	}

	//fingers state cb
	void fingers_cb (const kinova_msgs::FingerPosition msg) {
	  current_finger = msg;
	}

	void grasps_cb(const agile_grasp::Grasps &msg){
		ROS_INFO("Heard grasps!");
		current_grasps = msg;
		heardGrasps = true;
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
		
	void listenForGrasps(float rate){
		ros::Rate r(rate);
		heardGrasps = false;
		while (ros::ok()){
			ros::spinOnce();
			if (heardGrasps)
				return;
			r.sleep();
		}
	}
	
	// Range = [6, 7300] ([open, close])
	void moveFingers(int finger_value) {
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

	void spinSleep(double duration){
		int rateHertz = 40;	
		ros::Rate r(rateHertz);
		for(int i = 0; i < (int)duration * rateHertz; i++) {
			ros::spinOnce();
			r.sleep();
		}
	}
	
	/*void selectGrasp(const segbot_arm_manipulation::TabletopGraspGoalConstPtr  &goal, ){
		
		//first, construct list of grasp commands
		std::vector<GraspCartesianCommand> grasp_commands;
			
		for (unsigned int i = 0; i < current_grasps.grasps.size(); i++){
				
							
			GraspCartesianCommand gc_i = segbot_arm_manipulation::grasp_utils::constructGraspCommand(current_grasps.grasps.at(i),HAND_OFFSET_APPROACH,HAND_OFFSET_GRASP, cloud_ros.header.frame_id);
				
			bool ok_with_plane = segbot_arm_manipulation::grasp_utils::checkPlaneConflict(gc_i,plane_coef_vector,MIN_DISTANCE_TO_PLANE);
				
			if (ok_with_plane){
					
				listener.transformPose("m1n6s200_link_base", gc_i.approach_pose, gc_i.approach_pose);
				listener.transformPose("m1n6s200_link_base", gc_i.grasp_pose, gc_i.grasp_pose);
					
				//filter two -- if IK fails
				moveit_msgs::GetPositionIK::Response  ik_response_approach = segbot_arm_manipulation::computeIK(nh_,gc_i.approach_pose);
					
				if (ik_response_approach.error_code.val == 1){
					moveit_msgs::GetPositionIK::Response  ik_response_grasp = segbot_arm_manipulation::computeIK(nh_,gc_i.grasp_pose);
				
					if (ik_response_grasp.error_code.val == 1){
							
							
						//now check to see how close the two sets of joint angles are
						std::vector<double> D = segbot_arm_manipulation::getJointAngleDifferences(ik_response_approach.solution.joint_state, ik_response_grasp.solution.joint_state );
							
						double sum_d = 0;
						for (int p = 0; p < D.size(); p++){
							sum_d += D[p];
						}
						
						
						if (sum_d < ANGULAR_DIFF_THRESHOLD){
							ROS_INFO("Angle diffs for grasp %i: %f, %f, %f, %f, %f, %f",(int)grasp_commands.size(),D[0],D[1],D[2],D[3],D[4],D[5]);
								
							ROS_INFO("Sum diff: %f",sum_d);
							
							
							//store the IK results
							gc_i.approach_q = ik_response_approach.solution.joint_state;
							gc_i.grasp_q = ik_response_grasp.solution.joint_state;
								
							grasp_commands.push_back(gc_i);
						}
					}
				}
			}
		}
	}*/
	
	bool passesFilter(std::string filterName, GraspCartesianCommand gc){
		
		tf::Quaternion q(gc.approach_pose.pose.orientation.x, 
								gc.approach_pose.pose.orientation.y, 
								gc.approach_pose.pose.orientation.z, 
								gc.approach_pose.pose.orientation.w);
		tf::Matrix3x3 m(q);
			
		double r, p, y;
		m.getRPY(r, p, y);
		
		ROS_INFO_STREAM(gc.approach_pose);
		ROS_INFO("RPY: %f %f %f",r,p,y);
		
		if (filterName == segbot_arm_manipulation::TabletopGraspGoal::SIDEWAY_GRASP_FILTER){
			
			//ROS_INFO("%f, %f",fabs(p),fabs(3.14/2.0 - r) ); 
			
			//ideally roll should be PI/2, while pitch should be 0
			
			if ( r > 1.1 && r < 1.9 && p > -0.25 && p < 0.25)
				return true;
			else return false;
			
		}
		else if (filterName == segbot_arm_manipulation::TabletopGraspGoal::TOPDOWN_GRASP_FILTER){
			double roll_abs = fabs(r);
			
			if ( roll_abs < 3.3 && roll_abs > 2.6 && p > -0.3 && p < 0.3){
				return true;
			}
			else {
				return false;
			}
		}
		
		return true;
	}
	
	/*
	 * blocks until force of sufficient amount is detected or timeout is exceeded
	 * returns true of force degected, false if timeout
	 */
	bool waitForForce(double force_threshold, double timeout){
		double rate = 40.0;
		ros::Rate r(rate);

		double total_grav_free_effort = 0;
		double total_delta;
		double delta_effort[6];

		listenForArmData(rate);
		sensor_msgs::JointState prev_effort_state = current_state;

		double elapsed_time = 0;

		while (ros::ok()){
		
			ros::spinOnce();
				
			total_delta=0.0;
			for (int i = 0; i < 6; i ++){
				delta_effort[i] = fabs(current_state.effort[i]-prev_effort_state.effort[i]);
				total_delta+=delta_effort[i];
				//ROS_INFO("Total delta=%f",total_delta);
			}
				
			if (total_delta > fabs(FORCE_HANDOVER_THRESHOLD)){
				ROS_INFO("[segbot_tabletop_grasp_as.cpp] Force detected");
				return true;	
			}
				
			r.sleep();
			elapsed_time+=(1.0)/rate;
				
			if (timeout > 0 && elapsed_time > timeout){		
				ROS_WARN("[segbot_tabletop_grasp_as.cpp] Wait for force function timed out");
				return false;
			}
		}
	}
	
	void executeCB(const segbot_arm_manipulation::TabletopGraspGoalConstPtr  &goal)
	{
		if (goal->action_name == segbot_arm_manipulation::TabletopGraspGoal::GRASP){
		
			if (goal->cloud_clusters.size() == 0){
				ROS_INFO("[segbot_tabletop_grap_as.cpp] No object point clouds received...aborting");
				as_.setAborted(result_);
				return;
			}
		
			
			ROS_INFO("[segbot_tabletop_grap_as.cpp] Received action request...proceeding.");
			listenForArmData(40.0);
			
			//the result
			segbot_arm_manipulation::TabletopGraspResult result;
		
			//get the data out of the goal
			Eigen::Vector4f plane_coef_vector;
			for (int i = 0; i < 4; i ++)
				plane_coef_vector(i)=goal->cloud_plane_coef[i];
			int selected_object = goal->target_object_cluster_index;
			
			PointCloudT::Ptr target_object (new PointCloudT);
			pcl::PCLPointCloud2 target_object_pc2;
			pcl_conversions::toPCL(goal->cloud_clusters.at(goal->target_object_cluster_index),target_object_pc2);
			pcl::fromPCLPointCloud2(target_object_pc2,*target_object);
			
			ROS_INFO("[segbot_tabletop_grasp_as.cpp] Publishing point cloud...");
			cloud_grasp_pub.publish(goal->cloud_clusters.at(goal->target_object_cluster_index));
			
			//wait for response at 5 Hz
			listenForGrasps(40.0);
			
			ROS_INFO("[segbot_tabletop_grasp_as.cpp] Heard %i grasps",(int)current_grasps.grasps.size());
		
			//next, compute approach and grasp poses for each detected grasp
			
			//wait for transform from visual space to arm space
			
			std::string sensor_frame_id = goal->cloud_clusters.at(goal->target_object_cluster_index).header.frame_id;
			
			listener.waitForTransform(sensor_frame_id, "m1n6s200_link_base", ros::Time(0), ros::Duration(3.0));
			
			
			//here, we'll store all grasp options that pass the filters
			std::vector<GraspCartesianCommand> grasp_commands;
			
			
			for (unsigned int i = 0; i < current_grasps.grasps.size(); i++){
				
							
				GraspCartesianCommand gc_i = segbot_arm_manipulation::grasp_utils::constructGraspCommand(current_grasps.grasps.at(i),HAND_OFFSET_APPROACH,HAND_OFFSET_GRASP, sensor_frame_id);
				
				
				
				//filter 1: if the grasp is too close to plane, reject it
				bool ok_with_plane = segbot_arm_manipulation::grasp_utils::checkPlaneConflict(gc_i,plane_coef_vector,MIN_DISTANCE_TO_PLANE);
				
				//for filter 2, the grasps need to be in the arm's frame of reference
				listener.transformPose("m1n6s200_link_base", gc_i.approach_pose, gc_i.approach_pose);
				listener.transformPose("m1n6s200_link_base", gc_i.grasp_pose, gc_i.grasp_pose);

				
				//filter 2: apply grasp filter method in request
				bool passed_filter = passesFilter(goal->grasp_filter_method,gc_i);
				
				if (passed_filter && ok_with_plane){
					ROS_INFO("Found grasp fine with filter and plane");
					
					
					
					//filter two -- if IK fails
					moveit_msgs::GetPositionIK::Response  ik_response_approach = segbot_arm_manipulation::computeIK(nh_,gc_i.approach_pose);
					
					if (ik_response_approach.error_code.val == 1){
						moveit_msgs::GetPositionIK::Response  ik_response_grasp = segbot_arm_manipulation::computeIK(nh_,gc_i.grasp_pose);
				
						if (ik_response_grasp.error_code.val == 1){
							
							ROS_INFO("...grasp fine with IK");
							
							
							//now check to see how close the two sets of joint angles are -- if the joint configurations for the approach and grasp poses differ by too much, the grasp will not be accepted
							std::vector<double> D = segbot_arm_manipulation::getJointAngleDifferences(ik_response_approach.solution.joint_state, ik_response_grasp.solution.joint_state);
							
							double sum_d = 0;
							for (int p = 0; p < D.size(); p++){
								sum_d += D[p];
							}
						
							
							if (sum_d < ANGULAR_DIFF_THRESHOLD){
								//ROS_INFO("Angle diffs for grasp %i: %f, %f, %f, %f, %f, %f",(int)grasp_commands.size(),D[0],D[1],D[2],D[3],D[4],D[5]);
								
								//ROS_INFO("Sum diff: %f",sum_d);
							
								//store the IK results
								gc_i.approach_q = ik_response_approach.solution.joint_state;
								gc_i.grasp_q = ik_response_grasp.solution.joint_state;
								
								grasp_commands.push_back(gc_i);
								
								ROS_INFO("...fine with continuity");
							}
						}
					}
				}
			}
			
			//check to see if all potential grasps have been filtered out
			if (grasp_commands.size() == 0){
				ROS_WARN("[segbot_tabletop_grasp_as.cpp] No feasible grasps found. Aborting.");
				as_.setAborted(result_);
				return;
			}
			
			//make sure we're working with the correct tool pose
			listenForArmData(30.0);
			
			int selected_grasp_index = -1;
			
			
			
			if (goal->grasp_selection_method == segbot_arm_manipulation::TabletopGraspGoal::CLOSEST_ORIENTATION_SELECTION){
				//find the grasp with closest orientatino to current pose
				double min_diff = 1000000.0;
				for (unsigned int i = 0; i < grasp_commands.size(); i++){
					double d_i = segbot_arm_manipulation::grasp_utils::quat_angular_difference(grasp_commands.at(i).approach_pose.pose.orientation, current_pose.pose.orientation);
					
					ROS_INFO("Distance for pose %i:\t%f",(int)i,d_i);
					if (d_i < min_diff){
						selected_grasp_index = (int)i;
						min_diff = d_i;
					}
				}
			}
			else if (goal->grasp_selection_method == segbot_arm_manipulation::TabletopGraspGoal::RANDOM_SELECTION){

				srand (time(NULL));
				selected_grasp_index = rand() % grasp_commands.size(); 
				ROS_INFO("Randomly selected grasp = %i",selected_grasp_index);     
			}
			else if (goal->grasp_selection_method == segbot_arm_manipulation::TabletopGraspGoal::CLOSEST_JOINTSPACE_SELECTION){
				
				double min_diff = 1000000.0;
				for (unsigned int i = 0; i < grasp_commands.size(); i++){
					std::vector<double> D_i = segbot_arm_manipulation::getJointAngleDifferences(grasp_commands.at(i).approach_q, current_state);
					
					double sum_d = 0;
					for (int p = 0; p < D_i.size(); p++)
						sum_d += D_i[p];
					
					if (sum_d < min_diff){
						selected_grasp_index = (int)i;
						min_diff = sum_d;
					}
				}
				
				
				
			}
			
			if (selected_grasp_index == -1){
				ROS_WARN("[segbot_tabletop_grasp_as.cpp] Grasp selection failed. Aborting.");
				as_.setAborted(result_);
				return;
			}
			
			//compute RPY for target pose
			ROS_INFO("Selected approach pose:");
			ROS_INFO_STREAM(grasp_commands.at(selected_grasp_index).approach_pose);

			//publish individual pose for visualization purposes
			pose_pub.publish(grasp_commands.at(selected_grasp_index).approach_pose);
			
			//close fingers while moving
			segbot_arm_manipulation::closeHand();
			
			//move to approach pose -- do it twice to correct 
			segbot_arm_manipulation::moveToPoseMoveIt(nh_,grasp_commands.at(selected_grasp_index).approach_pose);
			segbot_arm_manipulation::moveToPoseMoveIt(nh_,grasp_commands.at(selected_grasp_index).approach_pose);
			
			//open fingers
			segbot_arm_manipulation::openHand();
		
			//move to grasp pose
			segbot_arm_manipulation::moveToPoseMoveIt(nh_,grasp_commands.at(selected_grasp_index).grasp_pose);
		
			//close hand
			segbot_arm_manipulation::closeHand();
			
			result_.success = true;
			as_.setSucceeded(result_);
			return;
		}
		else if (goal->action_name == segbot_arm_manipulation::TabletopGraspGoal::HANDOVER){
			//TO DO: move to handover position
			
			ROS_INFO("Starting handover action...");
			
			//listen for haptic feedback
			
			double rate = 40;
			
			ros::Rate r(rate);

			double total_grav_free_effort = 0;
			double total_delta;
			double delta_effort[6];

			listenForArmData(40.0);
			sensor_msgs::JointState prev_effort_state = current_state;


			double elapsed_time = 0;

			while (ros::ok()){
		
				ros::spinOnce();
				
				total_delta=0.0;
				for (int i = 0; i < 6; i ++){
					delta_effort[i] = fabs(current_state.effort[i]-prev_effort_state.effort[i]);
					total_delta+=delta_effort[i];
					ROS_INFO("Total delta=%f",total_delta);
				}
				
				if (total_delta > fabs(FORCE_HANDOVER_THRESHOLD)){
					ROS_INFO("[segbot_tabletop_grasp_as.cpp] Force detected");
					
					//now open the hand
					segbot_arm_manipulation::openHand();
					
					result_.success = true;
					as_.setSucceeded(result_);
					return;
					
				}
				
				r.sleep();
				elapsed_time+=(1.0)/rate;
				
				if (goal->timeout_seconds > 0 && elapsed_time > goal->timeout_seconds){
					
					ROS_WARN("Handover action timed out...");
					
					result_.success = false;
					as_.setAborted(result_);
					
					return;
				}
			}
			
			
			
			result_.success = true;
			as_.setSucceeded(result_);
		}
		else if (goal->action_name == segbot_arm_manipulation::TabletopGraspGoal::HANDOVER_FROM_HUMAN){
			//open fingers
			segbot_arm_manipulation::openHand();
			
			//update readings
			listenForArmData(40.0);
			
			bool result = waitForForce(FORCE_HANDOVER_THRESHOLD,goal->timeout_seconds);
		
			if (result){
				segbot_arm_manipulation::closeHand();
				result_.success = true;
				as_.setSucceeded(result_);
			}
			else {
				result_.success = false;
				as_.setAborted(result_);
			}
		
		}
		else if (goal->action_name == segbot_arm_manipulation::TabletopGraspGoal::REPLACEMENT) {
			//step 1: move to side position
			
			//TO DO: smarter way to select placement, in this case, need to move arm out of the way
			//and percieve the table, find a bounding area to place obj and create a pose/state
			
			
			//step 2: move to goal pose
			segbot_arm_manipulation::moveToJointState(nh_, goal -> grasped_joint_state); //need to make sure it can go to this
			
			//step 3: open fingers
			segbot_arm_manipulation::openHand();
			
			//step 4: move to home
			segbot_arm_manipulation::homeArm(nh_);
			
			
			
		}
	
	
	}


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "segbot_tabletop_grasp_as");

  TabletopGraspActionServer as(ros::this_node::getName());
  ros::spin();

  return 0;
}


