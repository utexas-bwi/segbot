#include <ros/ros.h>

#include <signal.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <kinova_msgs/PoseVelocity.h>


//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"

//action for grasping
#include "segbot_arm_manipulation/TabletopGraspAction.h"


#include <segbot_arm_manipulation/arm_utils.h>

#define NUM_JOINTS 8 //6+2 for the arm

//some thresholds related to forces
#define MIN_FORWARD_FORCE_THRESHOLD 2.0
#define MAX_FORWARD_FORCE_THRESHOLD 3.0

#define MIN_SIDEWAY_FORCE_THRESHOLD 2.2
#define MAX_SIDEWAY_FORCE_THRESHOLD 3.0

#define MAX_FORWARD_VEL 0.4
#define MAX_TURN_VEL 0.25

//global variables for storing data
sensor_msgs::JointState current_state;
bool heardJoinstState;

geometry_msgs::PoseStamped current_pose;
bool heardPose;

geometry_msgs::WrenchStamped current_wrench;
bool heardWrench;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;


/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
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
}


//Joint state cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
  current_pose = msg;
  heardPose = true;
}

//Callback for toolwrench 
void wrench_cb(const geometry_msgs::WrenchStamped &msg){ 
	current_wrench = msg;
	heardWrench = true;
}

//blocking call to listen for arm data (in this case, joint states)
void listenForArmData(){
	
	heardJoinstState = false;
	heardPose = false;
	heardWrench = false;
	ros::Rate r(10.0);
	
	while (ros::ok()){
		ros::spinOnce();
		
		if (heardJoinstState && heardPose && heardWrench)
			return;
		
		r.sleep();
	}
}


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


int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "demo_hand_lead");
	
	ros::NodeHandle n;

	//create subscriber to joint angles
	ros::Subscriber sub_angles = n.subscribe ("/m1n6s200_driver/out/joint_state", 1, joint_state_cb);
	
	//create subscriber to tool position topic
	ros::Subscriber sub_tool = n.subscribe("/m1n6s200_driver/out/tool_pose", 1, toolpos_cb);

	//subscriber for wrench
	ros::Subscriber sub_wrench = n.subscribe("/m1n6s200_driver/out/tool_wrench", 1, wrench_cb);
	
	//base velocity publisher
	ros::Publisher pub_base_velocity = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	//hand velocity publisher
	ros::Publisher pub_hand_velocity = n.advertise<kinova_msgs::PoseVelocity>("/m1n6s200_driver/in/cartesian_velocity", 1);
	


	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	pressEnter("Demo starting. Position arm and press [Enter]");
	
	listenForArmData();
	geometry_msgs::PoseStamped default_pose = current_pose;
	
	
	double rate = 40.0;
	ros::Rate r(rate);
	
	//first, listen and calibrate what the true 0 x,y,z values are
	std::vector<double> x_vals,y_vals,z_vals;
	
	double elapsed_time = 0.0;
	
	
	
	while (elapsed_time < 1.0){
		ros::spinOnce();
		
		if (heardWrench){
			x_vals.push_back(current_wrench.wrench.force.x);
			y_vals.push_back(current_wrench.wrench.force.y);
			z_vals.push_back(current_wrench.wrench.force.z);	
			ROS_INFO("Heard wrench!");
		}
		
		r.sleep();
		elapsed_time+=1.0/rate;
	}
	
	double x_mean_zero = 0;
	double y_mean_zero = 0;
	double z_mean_zero = 0;
	for (unsigned int i = 0; i < x_vals.size(); i++){
		x_mean_zero += x_vals.at(i);
		y_mean_zero += y_vals.at(i);
		z_mean_zero += z_vals.at(i);
	}
	
	x_mean_zero = x_mean_zero / (double)x_vals.size();
	y_mean_zero = y_mean_zero / (double)y_vals.size();
	z_mean_zero = z_mean_zero / (double)z_vals.size();
	
	ROS_INFO("Calibration complete. Force values when still: %f, %f, %f",x_mean_zero,y_mean_zero,z_mean_zero);
	
	//
	
	
	while (ros::ok()){
		
		ros::spinOnce();
		
		if (heardWrench){
			ROS_INFO("Current force: %f, %f, %f",
			current_wrench.wrench.force.x,current_wrench.wrench.force.y,current_wrench.wrench.force.z);
			
			double forward_force = -1*(current_wrench.wrench.force.x - x_mean_zero);
			double sideway_force = current_wrench.wrench.force.y - y_mean_zero;
			
			ROS_INFO("Forward: %f\tSideways: %f",forward_force,sideway_force);
			
			geometry_msgs::Twist v_i;
			v_i.linear.x = 0; v_i.linear.y = 0; v_i.linear.z = 0;
			v_i.angular.x = 0; v_i.angular.y = 0; v_i.angular.z = 0;
			
			if (forward_force > MIN_FORWARD_FORCE_THRESHOLD){
				if (forward_force > MAX_FORWARD_FORCE_THRESHOLD)
					forward_force = MAX_FORWARD_FORCE_THRESHOLD;
					
				double x_vel = MAX_FORWARD_VEL*(forward_force-MIN_FORWARD_FORCE_THRESHOLD) / (MAX_FORWARD_FORCE_THRESHOLD - MIN_FORWARD_FORCE_THRESHOLD);
				
				v_i.linear.x = x_vel;
			}
			
			if (fabs(sideway_force) > MIN_SIDEWAY_FORCE_THRESHOLD){
				
				
				double sign = -1.0;
				if (sideway_force < 0.0)
					sign = 1.0;
					
				if (fabs(sideway_force) > MAX_SIDEWAY_FORCE_THRESHOLD)
					sideway_force = MAX_SIDEWAY_FORCE_THRESHOLD;
				
					
				double angular_vel = MAX_TURN_VEL*sign*(fabs(sideway_force)-MIN_SIDEWAY_FORCE_THRESHOLD) / (MAX_SIDEWAY_FORCE_THRESHOLD - MIN_SIDEWAY_FORCE_THRESHOLD);
				
				v_i.angular.z = angular_vel;
			}
			
			ROS_INFO("Linear vel.: %f\tAngular vel.: %f",v_i.linear.x,v_i.angular.z);
			
			//publish
			pub_base_velocity.publish(v_i);
			
			//if no force is applied, then move hand back to starting position
			if (v_i.angular.z == 0 && v_i.linear.x == 0){
				kinova_msgs::PoseVelocity velocityMsg; //vel command to hand
				float theta = 0.075;
				float constant_m = 3.0;
				
				//find out how far we are from the default position
				float dx = constant_m*( - current_pose.pose.position.x + default_pose.pose.position.x );
				float dy = constant_m*(- current_pose.pose.position.y + default_pose.pose.position.y);
				float dz = constant_m*(- current_pose.pose.position.z + default_pose.pose.position.z);
				
				if (dx > theta)
					velocityMsg.twist_linear_x = dx;
				
				if (dy > theta)
					velocityMsg.twist_linear_y = dy;
				
				if (dz > theta)
					velocityMsg.twist_linear_z = dz;
					
				ROS_INFO("Publishing c.vels: %f, %f, %f",dx,dy,dz);
				
				pub_hand_velocity.publish(velocityMsg);
			}
			
			heardWrench = false;
		}
		
		r.sleep();
		
		
	}
	
	
	
	
	
}
