#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <stdlib.h>
#include <std_msgs/String.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/WrenchStamped.h>

//for table scene and color histogram
#include "segbot_arm_perception/TabletopPerception.h"
#include <segbot_arm_perception/segbot_arm_perception.h>

//actions
#include <actionlib/server/simple_action_server.h>
#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"
#include "kinova_msgs/ArmJointAnglesAction.h"
#include <segbot_arm_manipulation/LiftVerifyAction.h>
#include <segbot_arm_manipulation/arm_utils.h>

//pcl includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/impl/centroid.hpp>
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
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#define FINGER_FULLY_OPENED 6
#define FINGER_FULLY_CLOSED 7300

#define NUM_JOINTS 8 //6+2 for the arm

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


class LiftVerifyActionServer
{
protected:

  ros::NodeHandle nh_;
  
  actionlib::SimpleActionServer<segbot_arm_manipulation::LiftVerifyAction> as_; 
  
  std::string action_name_;
  
  //messages to publish feedback and result of action
  segbot_arm_manipulation::LiftVerifyFeedback feedback_;
  segbot_arm_manipulation::LiftVerifyResult result_;
  
  
  ros::Subscriber sub_angles;
  ros::Subscriber sub_torques;
  ros::Subscriber sub_tool;
  ros::Subscriber sub_finger;
  ros::Subscriber sub_wrench;
  
  sensor_msgs::JointState current_state;
  kinova_msgs::FingerPosition current_finger;
  geometry_msgs::PoseStamped current_pose;
  geometry_msgs::WrenchStamped current_wrench;
  
  bool heardPose;
  bool heardJoinstState;
  bool heardWrench;
  bool heardEffort;
  
  int num_bins;
 
public:

  LiftVerifyActionServer(std::string name) :
    as_(nh_, name, boost::bind(&LiftVerifyActionServer::executeCB, this, _1), false),
    action_name_(name)
  {
	heardPose = false;
	heardJoinstState = false;
	heardWrench = false;
	heardEffort = false;

	//create subscriber to joint angles
	sub_angles = nh_.subscribe ("/m1n6s200_driver/out/joint_state", 1, &LiftVerifyActionServer::joint_state_cb, this);

	//create subscriber to tool position topic
	sub_tool = nh_.subscribe("/m1n6s200_driver/out/tool_pose", 1, &LiftVerifyActionServer::toolpos_cb, this);

	//subscriber for fingers
	sub_finger = nh_.subscribe("/m1n6s200_driver/out/finger_position", 1, &LiftVerifyActionServer::fingers_cb, this);
	  
	//subscriber for wrench
	sub_wrench = nh_.subscribe("/m1n6s200_driver/out/tool_wrench", 1, &LiftVerifyActionServer::wrench_cb, this);
	
	ROS_INFO("Lift and verify action has started");
	
    as_.start();
  }

  ~LiftVerifyActionServer(void)
  {
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
	}

	//fingers state cb
	void fingers_cb (const kinova_msgs::FingerPosition msg) {
	  current_finger = msg;
	}

	//Callback for toolwrench 
	void wrench_cb(const geometry_msgs::WrenchStamped &msg){ 
		current_wrench = msg;
		heardWrench = true;
	}
	
	//wait for updated data
	void listenForArmData(float rate){
		heardPose = false;
		heardJoinstState = false;
		heardWrench = false;
		ros::Rate r(rate);
		
		while (ros::ok()){
			ros::spinOnce();
			
			if (heardPose && heardJoinstState && heardWrench)
				return;
			
			r.sleep();
		}
	}	
	
	std::vector<double> get_color_hist(PointCloudT desired_cloud, int dim){ 
		//get a color histogram and save it as a one dimensional vector for comparison
		std::vector<std::vector<std::vector<uint> > > hist3= segbot_arm_perception::computeRGBColorHistogram(desired_cloud, dim);
		int i_offset = dim * dim;
        int j_offset = dim;
        std::vector<double> hist3_double_vector (dim * dim * dim, 0);
        for (int i = 0; i < dim; i++) {
            for (int j = 0; j < dim; j++) {
                for (int k = 0; k < dim; k++) {
                    hist3_double_vector[i * i_offset + j * j_offset + k] = hist3[i][j][k];
                }
            }
        }
        return hist3_double_vector;	
	}

	double euclidean_distance(Eigen::Vector4f center_vector, Eigen::Vector4f new_center){
		//find the distance between the new object and the original object's original location
		double x_diff = (double) new_center(0) - center_vector(0);
		double y_diff = (double) new_center(1) - center_vector(1);
		double z_diff = (double) new_center(2) - center_vector(2);
		return (double) sqrt(pow(x_diff, 2) + pow(y_diff, 2) + pow (z_diff, 2));
		
	}
	
	double correlation_coeff(std::vector<double> orig_colorhist, std::vector<double> new_colorhist){
		//compares color histogram to determine if the objects are similar
		//uses the Pearson correlation coefficient
		if(orig_colorhist.size() != new_colorhist.size()){
			ROS_ERROR("Error: Color histograms are not the same size. Aborting...");
			as_.setAborted(result_);
			exit(1);
		}
		
		double sum_xy = 0.0;
		double sum_x = 0.0;
		double sum_y = 0.0;
		double sum_x_2 = 0.0;
		double sum_y_2 = 0.0;
		double num = 0.0;
		
		for(unsigned int i = 0; i < orig_colorhist.size(); i++){
			num++; 
			sum_x += orig_colorhist.at(i);
			sum_y += new_colorhist.at(i);
			sum_x_2 += pow(orig_colorhist.at(i), 2);
			sum_y_2 += pow(new_colorhist.at(i) , 2);
			sum_xy += (orig_colorhist.at(i) * new_colorhist.at(i));
		}
		
		double result = sum_xy - ((sum_x * sum_y)/ num);
		result /= sqrt((sum_x_2 - (pow(sum_x , 2) / num)) * (sum_y_2 - (pow(sum_y , 2) /num)));
		return result;
	}
	
	bool down_force(double goal_down_force){
		//compares the downward force after grabbing the object to the expected force without an object
		 listenForArmData(30.0);
		 
		 bool greater_force = false;
		 double threshold = 0.2;
		 
		 listenForArmData(30.0);
		 
		 double diff = current_wrench.wrench.force.z - goal_down_force; 
		 
		 if(diff>threshold){ 
			 ROS_INFO("force test succeeded");
			 greater_force = true;
		 }else{
			 ROS_WARN("force test failed");
		 }
		 return greater_force;
	}
	
	bool fingers_open(){ 
		//if the fingers are within some distance to closed, the arm didn't grab the object
		 listenForArmData(30.0);
		 double tolerance = 120;
		 double finger1_diff = (double) abs(current_finger.finger1 - FINGER_FULLY_CLOSED); 
		 double finger2_diff = (double) abs(current_finger.finger2 - FINGER_FULLY_CLOSED);
		 
		 if(finger1_diff < tolerance && finger2_diff < tolerance){
			 ROS_WARN("fingers test failed");
			 return false;
		 }
		 ROS_INFO("fingers test succeeded");
		 return true; 
	}
	
	bool not_on_table(Eigen::Vector4f center_vector, std::vector<double> orig_colorhist){
		//check table for new objects
		segbot_arm_perception::TabletopPerception::Response new_scene = segbot_arm_manipulation::getTabletopScene(nh_);
		
		double tolerance = 0.1; 
		
		//if center of an object is within some distance, compare color histograms
		for(unsigned int i = 0; i< new_scene.cloud_clusters.size(); i++){
			PointCloudT::Ptr object_i (new PointCloudT);
			pcl::fromROSMsg(new_scene.cloud_clusters[i], *object_i);
			
			Eigen::Vector4f obj_i_center;
			pcl::compute3DCentroid(*object_i, obj_i_center);
			ROS_INFO("The current object's center is %f, %f, %f",obj_i_center(0),obj_i_center(1),obj_i_center(2));
			
			double distance = euclidean_distance(center_vector, obj_i_center);
			
			if(distance < tolerance){ 
				std::vector<double> new_colorhist = get_color_hist(*object_i, num_bins);
				ROS_INFO("found an object with a similar center...");
				
				double corr = correlation_coeff(orig_colorhist, new_colorhist);
				
				if (corr >= 0.8){ 
					ROS_WARN("object is the same, table test failed");
					return false; //this is the same object
				}
			}
			
		}
		
		ROS_INFO("table test succeeded");
		return true; //checked all objets on table, none are the same as the target
	}
	
	void executeCB(const segbot_arm_manipulation::LiftVerifyGoalConstPtr  &goal){
		
		listenForArmData(30.0);
		
		if (as_.isPreemptRequested() || !ros::ok()){
			ROS_INFO("Lift verification: Preempted");
			// set the action state to preempted
			as_.setPreempted();
			result_.success = false;
			as_.setSucceeded(result_);
			return;
        }
		
		segbot_arm_manipulation::arm_side_view(nh_);
		
		//expected downward force, must be changed for new arm locations
		double goal_down_force = 0.9;
		
		//set number of color bins for use in computing color histograms
		num_bins = goal -> bins;
		

		PointCloudT pcl_pc;
		pcl::fromROSMsg(goal -> tgt_cloud, pcl_pc);
		
		//create a color histogram of the goal object
		std::vector<double> orig_colorhist = get_color_hist(pcl_pc, num_bins);
		
		//find center of goal object 
		Eigen::Vector4f center_vector;
		pcl::compute3DCentroid(pcl_pc, center_vector);
		

		//check if the downward force is greater than expected
		bool greater_force = down_force(goal_down_force);
		
		//check if the fingers are open
		bool fingers_still_open = fingers_open();
		
		//check to see if the object is still on the table
		bool gone_from_table = not_on_table(center_vector, orig_colorhist);
		
		//return true if two of the three conditions are met
		result_.success = ((greater_force && fingers_still_open) || (greater_force && gone_from_table) || 
			(fingers_still_open && gone_from_table));
			
		as_.setSucceeded(result_);
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_lift_verify_as");

  LiftVerifyActionServer as(ros::this_node::getName());
  ros::spin();

  return 0;
}
