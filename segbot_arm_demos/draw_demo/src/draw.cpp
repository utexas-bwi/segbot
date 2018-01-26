#include <ros/ros.h>
#include <signal.h> 
#include <iostream>
#include <vector>
#include <math.h> 
#include <cstdlib>

#define PI 3.14159265
#define TOOL_POS_RATE .025	//Speed in which /m1n6s200_driver/out/tool_pose topic is updated. Used in calculating future distances
							//NOTE: tool_position refresh rate set in paramaters of launch file. As that changes, this value should be updated.
#define BASE_VELOCITY .05	//Base velocity. distance / time = 1 / 10 seconds = .1 meters a second.
#define BASE_VELOCITY_SHORT .05	//slower velocity for smaller distances. higher accuracy
#define TRAVEL_VELOCITY .10
//subscriber msgs
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

//actions
#include <actionlib/client/simple_action_client.h>
#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"
#include <kinova_msgs/PoseVelocity.h>


//file manipulation
//file manip
#include <cstdlib>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <cstring>

using namespace std;

ros::Publisher joint_state_pub;
ros::Publisher c_vel_pub_;

sensor_msgs::JointState current_jpos;

//efforts and force detection
bool heard_efforts = false;
sensor_msgs::JointState current_efforts;
sensor_msgs::JointState last_efforts;
double total_grav_free_effort = 0;
double total_delta;
double delta_effort[6];

//Distance variables - tool position
bool poc = false;
bool initPosition = false;
bool alreadySet = false;
bool traveling = false;
bool debug = false;
double poc_x, poc_y, poc_z;
double cur_x, cur_y, cur_z;
double b_origin_x, b_origin_y, b_origin_z;
double origin_x, origin_y, origin_z;
double origin_x_d, origin_y_d, origin_z_d;
double drawDistance;


//Finger vars
float f1;
float f2;
//Origin tracking
bool setOrigin = false;
bool trackFromOrigin = false;

//true if Ctrl-C is pressed - kill switch
bool g_caught_sigint=false;

//kill switch
void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};




//********************Call back functions

//Tool position callback (distances)
void toolpos_cb(const geometry_msgs::PoseStamped &msg){
	ros::Rate r(100);
	geometry_msgs::PoseStamped current = msg;
	cur_x = current.pose.position.x;
	cur_y = current.pose.position.y;
	cur_z = current.pose.position.z;


	if(total_delta > 0.7){	
		poc = true;
	}
	if(setOrigin && poc){
		b_origin_x = cur_x;
		b_origin_y = cur_y;
		b_origin_z = cur_z;
		trackFromOrigin = true;
		setOrigin = false;
		cout << "Point recorded!" << endl;
	}
	
	if(trackFromOrigin){
		if(cur_x < 0 && b_origin_x > 0)
			origin_x = b_origin_x - cur_x;
		else
			origin_x = cur_x - b_origin_x;
		if(cur_y < 0 && b_origin_y > 0)
			origin_y = b_origin_y - cur_y;
		else
			origin_y = cur_y - b_origin_y;
		if(cur_z < 0 && b_origin_z > 0)
			origin_z = b_origin_z - cur_z;
		else
			origin_z = cur_z - b_origin_z;
	}

	if(poc){
		if(!initPosition){
			poc_x = current.pose.position.x;
			poc_y = current.pose.position.y;
			poc_z = current.pose.position.z;
			initPosition = true;
			alreadySet = true;
		}
		
		double xDis = cur_x - poc_x;
		double yDis = cur_y - poc_y;
		xDis *= xDis;
		yDis *= yDis;
		drawDistance = sqrt(xDis + yDis);
	}

}

//checks fingers position - used for object holding assurance
void fingers_cb(const kinova_msgs::FingerPosition input){
	f1 = input.finger1;
	f2 = input.finger2;
}


//Joint state cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input)
{
	current_jpos = *input;
		//compute the change in efforts if we had already heard the last one
	if (heard_efforts){
		for (int i = 0; i < 6; i ++){
			delta_effort[i] = input->effort[i]-current_efforts.effort[i];
		}
	}
	
	//store the current effort
	current_efforts = *input;
	
	total_grav_free_effort = 0.0;
	for (int i = 0; i < 6; i ++){
		if (current_efforts.effort[i] < 0.0)
			total_grav_free_effort -= (current_efforts.effort[i]);
		else 
			total_grav_free_effort += (current_efforts.effort[i]);
	}
	
	//calc total change in efforts
	total_delta = delta_effort[0]+delta_effort[1]+delta_effort[2]+delta_effort[3]+delta_effort[4]+delta_effort[5];
	
	heard_efforts=true;
}


//************************************Normal functions


//Invokes the arm_pose server to grab object
//Several predefined poses - "grab" for obtaining an object from human
//"horizontal" to orient the joints in the horizontal writing position
void getWriteReady(string position){
	actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ac("/m1n6s200_driver/pose_action/tool_pose", true);
	
	kinova_msgs::ArmPoseGoal goalPose;
	
	if(position.compare("start") == 0){
		//Only moving in 2d space, orientation stays
		goalPose.pose.header.frame_id = "m1n6s200_link_base";
		goalPose.pose.pose.position.x = 0.151496844;	
		goalPose.pose.pose.position.y = -0.3651103675;				
		goalPose.pose.pose.position.z = 0.3085489869;			
		goalPose.pose.pose.orientation.x = 0.20792265145;		
		goalPose.pose.pose.orientation.y = 0.68410680388;		
		goalPose.pose.pose.orientation.z = -0.232979253411;		
		goalPose.pose.pose.orientation.w = 0.659156066027;		
	}
	else if (position.compare("grab") == 0){
		//hand is tilted upwards to make inserting a pen easier.
		goalPose.pose.header.frame_id = "m1n6s200_link_base";
		goalPose.pose.pose.position.x = 0.259099006653;	
		goalPose.pose.pose.position.y = -0.401062101126;				
		goalPose.pose.pose.position.z = 0.514227807522;			
		goalPose.pose.pose.orientation.x = -0.12479989278;		
		goalPose.pose.pose.orientation.y = 0.458658084177;		
		goalPose.pose.pose.orientation.z = 0.430996390374;		
		goalPose.pose.pose.orientation.w = 0.767007079541;
	}
	else if (position.compare("horizontal") == 0){
		goalPose.pose.header.frame_id = "m1n6s200_link_base";
		goalPose.pose.pose.position.x = -0.0159843936563;
		goalPose.pose.pose.position.y =  -0.453825235367;
		goalPose.pose.pose.position.z =  0.065297177672;
		goalPose.pose.pose.orientation.x = .0117474132316;//0.0117474132316;		
		goalPose.pose.pose.orientation.y = -0.998421445202;//-0.998421445202;		
		goalPose.pose.pose.orientation.z =  0.0040660488499;	//0.0040660488499;		
		goalPose.pose.pose.orientation.w = 0.0266835433905;	//0.0266835433905;	
		goalPose.pose.pose.orientation.x = 0.0117474132316;		
		goalPose.pose.pose.orientation.y = -0.998421445202;		
		goalPose.pose.pose.orientation.z = 0.0040660488499;		
		goalPose.pose.pose.orientation.w = 0.0266835433905;		

	}
	ac.waitForServer();
	ROS_DEBUG("Waiting for server.");
	//finally, send goal and wait
	ROS_INFO("Sending goal.");
	ac.sendGoal(goalPose);
	ac.waitForResult();
}

//This function is useful after moving joints to a new position - it allows the joints to settle and the force sensors to aclimate to the new position
//.7 seconds seems to be the maximum amount of time needed for this in application
void clearMsgs(){
	ros::Time start = ros::Time::now();
	ros::Duration timeout = ros::Duration(.7);
	ros::Rate r2(5);
	//clears out old effort msgs
	while( (ros::Time::now() - start) < timeout){
		ros::spinOnce();
		r2.sleep();
	}
}

void clearMsgs(double duration){
	ros::Time start = ros::Time::now();
	ros::Duration timeout = ros::Duration(duration);
	ros::Rate r2(5);
	//clears out old effort msgs
	while( (ros::Time::now() - start) < timeout){
		ros::spinOnce();
		r2.sleep();
	}
}

//Used for debugging force changes. Outputs both the deltas and gravityFree into a file for analysis
void writeToFile(vector<double> gravFree, vector<double> deltas){
	fstream filestr;

 	 filestr.open("src/robot_arm/config/establish_contact_efforts.txt", fstream::in | fstream::out | fstream::app);
 	 filestr << "Grav Free" << endl;
 	 for(int i = 0; i < gravFree.size(); i++){
 	 	filestr << gravFree.at(i) << endl;
 	 }
 	 filestr << endl << "Deltas" << endl;
 	 for(int i = 0; i < deltas.size(); i++){
 	 	filestr << deltas.at(i) << endl;
 	 }
 	 
 	 filestr.close();
}

//helper function for openAndClose
//opens fingers compeltely
int openFull(){
	actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> ac("/m1n6s200_driver/fingers/finger_positions/", true);
	kinova_msgs::SetFingersPositionGoal goal;
	goal.fingers.finger1 = 6;
	goal.fingers.finger2 = 6;
	goal.fingers.finger3 = 0;
	ac.waitForServer();
	ac.sendGoal(goal);
	ac.waitForResult();
 
}
//lifts end effector .01 m
void lift(){
	ros::Rate r(100);
	ros::spinOnce();
	double initZ = cur_z;
	kinova_msgs::PoseVelocity T;
		T.twist_linear_x= 0.0;
		T.twist_linear_y= 0.0;
		T.twist_angular_x=0.0;
		T.twist_angular_y=0.0;
		T.twist_angular_z=0.0;
	while(abs(cur_z - initZ) < .01){
		ros::spinOnce();

		T.twist_linear_z= 0.1;

		c_vel_pub_.publish(T);
		r.sleep();
	}
	T.twist_linear_z= 0.0;
	c_vel_pub_.publish(T);

}
//helper function for openAndCloseFingers.
//closes the fingers completely
int closeComplt(){
	actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> ac("/m1n6s200_driver/fingers/finger_positions/", true);
	kinova_msgs::SetFingersPositionGoal goal;
 	goal.fingers.finger1 = 7000;
	goal.fingers.finger2 = 7000;
	goal.fingers.finger3 = 0;
	ac.waitForServer();
	ac.sendGoal(goal);
	ac.waitForResult();
}

//moves the arm to an innocuous location.
//the arm then waits until significant force is detected.
int moveAndPause(){
	
	ros::Time start = ros::Time::now();
	ros::Duration timeout = ros::Duration(60.0);
	ros::Rate r(20);
	actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ac("/m1n6s200_driver/pose_action/tool_pose", true);
	
	kinova_msgs::ArmPoseGoal goalPose;
	
	//Only moving in 2d space, orientation stays
	goalPose.pose.header.frame_id = "m1n6s200_link_base";
	goalPose.pose.pose.position.x = -0.169210940599;	
	goalPose.pose.pose.position.y = -0.135450258851;				
	goalPose.pose.pose.position.z = 0.299253374338;			
	goalPose.pose.pose.orientation.x = -0.960183273276;		
	goalPose.pose.pose.orientation.y = -0.277915923109;		
	goalPose.pose.pose.orientation.z = -0.0284747701323;		
	goalPose.pose.pose.orientation.w = 9.41823637923e-05;
	
	
	ac.waitForServer();
	ac.sendGoal(goalPose);
	ac.waitForResult();
	clearMsgs();
	while(true){
		ros::spinOnce();
		if(heard_efforts){										
			if (total_delta > 1){
				if(debug){
					ROS_DEBUG("Forces above 1N detected.");
					ROS_DEBUG("Efforts: %f", total_delta);
				}
				closeComplt();
				break;
			}
		} 
		r.sleep();
	}
	
	goalPose.pose.header.frame_id = "m1n6s200_link_base";
	goalPose.pose.pose.position.x = -0.0159843936563;
	goalPose.pose.pose.position.y =  -0.453825235367;
	goalPose.pose.pose.position.z =  0.065297177672;
	goalPose.pose.pose.orientation.x = 0.0117474132316;		
	goalPose.pose.pose.orientation.y = -0.998421445202;		
	goalPose.pose.pose.orientation.z = 0.0040660488499;		
	goalPose.pose.pose.orientation.w = 0.0266835433905;	
	
	ac.waitForServer();
	ac.sendGoal(goalPose);
	ac.waitForResult();
}


//method reads torque readings and will close grippers once something has been placed in the hand
int openFingersAndWait(){
	
	ros::Time start = ros::Time::now();
	ros::Duration timeout = ros::Duration(60.0);
	ros::Rate r(20);
	
	//Open fingers.
	openFull();
	clearMsgs();
	while(true){
		ros::spinOnce();
		if(heard_efforts){										
			if (total_delta > 0.29){
				if(debug){
					ROS_DEBUG("Forces above 0.29N detected. Closing fingers.");
					ROS_DEBUG("Efforts: %f", total_delta);
				}
				closeComplt();
				break;
			}
		} 
		r.sleep();
	}
	//This checks if the fingers missed their target
	ros::spinOnce();
	if(f1 > 7400 || f2 > 7000){
		if(debug)
			cout << "Finger 1: " << f1 << " Finger 2: " << f2 << endl;
		openFingersAndWait();
	}
	return 0;
}

int forceFeltDrop(){
	
	ros::Time start = ros::Time::now();
	ros::Duration timeout = ros::Duration(2.0);
	ros::Rate r(200);
	//clears out old effort msgs
	while( (ros::Time::now() - start) < timeout){
		ros::spinOnce();
		r.sleep();
	}
	while(true){
		
		ros::spinOnce();
		if (heard_efforts){									
			if (total_delta > 0.20){
				if(debug){
					ROS_INFO("Forces above 0.15N detected. Dropping object.");
					ROS_INFO("Efforts: %f", total_delta);
				}
				openFull();
				break;
			}
		} 
	}
	return 0;
}

//pubs velocity raw commands.
//currently only implemented for retracting
//Legacy
void sendRawVelocity(double xin, double yin, double zin){
	double duration = 1.0;
	ros::Rate r(100);
	
	for (int i = 0; i < (int)duration*100; i++){
		ros::spinOnce();
		
		kinova_msgs::PoseVelocity T; 
		T.twist_linear_x = xin;
		T.twist_linear_y = yin;
		T.twist_linear_z = zin;
		
		T.twist_angular_x=0.0;
		T.twist_angular_y=0.0;
		T.twist_angular_z=0.0;
		
		c_vel_pub_.publish(T);

		r.sleep();
	}
}

//draws a square with velocity pubs
//Legacy. (first drawing method made!)
void drawSquare(){
	sendRawVelocity(0.015,0.1,0);
	sendRawVelocity(0.015,0.0,-0.1);
	sendRawVelocity(0.015,-0.1,0);
	sendRawVelocity(0.015,0.0,0.1);
}
//Legacy
void drawSquareHorizontal(){
	sendRawVelocity(0.00,0.1,-0.04);
	sendRawVelocity(0.1,0.0,-0.015);
	sendRawVelocity(0.00,-0.1,-0.015);
	sendRawVelocity(-0.1,0.0,-0.015);
	sendRawVelocity(0.00,0.1,-0.015);
	lift();
}

//Sends an velocity-x command to draw a line as long as passed in
//Useful debugging tool and first step in establishing own reference frame
void drawDistanceLine(double targetDistance){
	
	float vel_speed = .05;
	
	ros::Time start = ros::Time::now();
	ros::Duration timeout = ros::Duration(0.23);
	ros::Rate r(100);
	
	while(drawDistance < targetDistance){
		ros::spinOnce();
		
		
		kinova_msgs::PoseVelocity T; 
		T.twist_linear_x = vel_speed;
		T.twist_linear_y = 0.0;
		T.twist_linear_z = 0.0;
		
		T.twist_angular_x=0.0;
		T.twist_angular_y=0.0;
		T.twist_angular_z=0.0;
		
		//compensates for overhead and movement to keep the line as accurate as possible.
		if(vel_speed * targetDistance + drawDistance >= targetDistance){
			T.twist_linear_x = 0.0;
			T.twist_linear_y = 0.0;
			T.twist_linear_z = 0.0;
			
			T.twist_angular_x=0.0;
			T.twist_angular_y=0.0;
			T.twist_angular_z=0.0;
		}
		ROS_INFO("Target distance: %f. Calculated distance: %f", targetDistance, drawDistance);
		c_vel_pub_.publish(T);
		
		r.sleep();
	
	}
	//distance threshold hit. send zero vel cmd to cut it short.
	kinova_msgs::PoseVelocity T; 
	T.twist_linear_x = 0;
	T.twist_linear_y = 0;
	T.twist_linear_z = 0;
	
	T.twist_angular_x=0.0;
	T.twist_angular_y=0.0;
	T.twist_angular_z=0.0;
	
	c_vel_pub_.publish(T);
	ROS_INFO("Target distance: %f. Calculated distance: %f", targetDistance, drawDistance);
}

//establishes contact assuming that the SAFETY FORCE CONTROL is enabled via the driver
//todo: take an orientation as a string
//Approaches the target - touches it once
//Two types of precision - use "double" precision if the end effector is high above the surface (ie first contact)
//Use "single" precision if the end effector is .04m or less (ie lift from drawing)
void establish_contact(string precision){
	double timeout_duration = 30.0;
	bool hitSurface = false;
	
	clearMsgs();
	kinova_msgs::PoseVelocity T; 
	
	//"double" precision requires two surface touches. 
	//Double precision should be used for an initial touch when the height of the end effector is unknown
	//sing precision is useful if you know the height of the end effector is relatively small (ex using the lift() function)
	if(precision == "double")
		hitSurface = false;
	else
		hitSurface = true;
	
	if(!hitSurface){
		ros::Rate r(100);
		for (int i = 0; i < (int)timeout_duration*100; i ++){
			
			ros::spinOnce();
			
			T.twist_linear_x=0.0;
			T.twist_linear_y=0.0;
			T.twist_linear_z=-0.15;
			
			T.twist_angular_x=0.0;
			T.twist_angular_y=0.0;
			T.twist_angular_z=0.0;
			
			c_vel_pub_.publish(T);

			r.sleep();

			if (heard_efforts){
				if (total_delta > .7){ // <<----------------------------hard coded force value
					//ROS_INFO("Efforts: %f", total_delta);
					//heard_efforts = false;
					hitSurface = true;
					break;
				}
			}	
		}
	}
	clearMsgs();
	ros::Rate r2(100);
	if(hitSurface){ //approach slowly
		if(debug)
			ROS_INFO("Efforts: %f", total_grav_free_effort);
		while(total_grav_free_effort < 1.9){ // <<------------------------hard coded force value
			//ROS_INFO("Efforts: %f", total_delta);
			ros::spinOnce();
			T.twist_linear_x=0.0;
			T.twist_linear_y=0.0;
			T.twist_linear_z=-0.05;
			
			T.twist_angular_x=0.0;
			T.twist_angular_y=0.0;
			T.twist_angular_z=0.0;
			
			c_vel_pub_.publish(T);

			r2.sleep();
		}
			T.twist_linear_x=0.0;
			T.twist_linear_y=0.0;
			T.twist_linear_z=-0.00;
			
			T.twist_angular_x=0.0;
			T.twist_angular_y=0.0;
			T.twist_angular_z=0.0;
			
			c_vel_pub_.publish(T);

	}
}

//caller function for the write-tofile debug function for the force efforts
void output_static_efforts(){
	kinova_msgs::PoseVelocity T; 
	double timeout_duration = 5.0;
	vector<double> grav_free;
	vector<double> delta;
	ros::Time start = ros::Time::now();
	ros::Duration timeout = ros::Duration(1.0);
	ros::Rate r2(200);
	//clears out old effort msgs
	while( (ros::Time::now() - start) < timeout){
		ros::spinOnce();
		r2.sleep();
	}
	
	ros::Rate r(100);
	for (int i = 0; i < (int)timeout_duration*100; i ++){
		ros::spinOnce();
		
		T.twist_linear_x=0;//0.4f;
		T.twist_linear_y=0;
		T.twist_linear_z=0;//-40.0;
		
		T.twist_angular_x=0.0;
		T.twist_angular_y=0.0;
		T.twist_angular_z=0.0;
		c_vel_pub_.publish(T);
		grav_free.push_back(total_grav_free_effort);
		delta.push_back(total_delta);
		r.sleep();
	}
	writeToFile(grav_free, delta);
}
//reads efforts to know when its touched something
//**LEGACY CODE, DO NOT DELETE

void establish_contact(double v_x, double v_y, double v_z){
	
	kinova_msgs::PoseVelocity T; 
	double timeout_duration = 30.0;
	vector<double> grav_free;
	vector<double> delta;
	ros::Time start = ros::Time::now();
	ros::Duration timeout = ros::Duration(1.0);
	ros::Rate r2(200);
	//clears out old effort msgs
	while( (ros::Time::now() - start) < timeout){
		ros::spinOnce();
		r2.sleep();
	}
	
	ros::Rate r(100);
	for (int i = 0; i < (int)timeout_duration*100; i ++){
		ros::spinOnce();
		
		T.twist_linear_x=v_x;//0.4f;
		T.twist_linear_y=v_y;
		T.twist_linear_z=v_z;//-40.0;
		
		T.twist_angular_x=0.0;
		T.twist_angular_y=0.0;
		T.twist_angular_z=0.0;

		c_vel_pub_.publish(T);

		r.sleep();
		
		if (heard_efforts){
			//grav_free.push_back(total_grav_free_effort);
			//delta.push_back(total_delta);
			//ROS_DEBUG("Total effort: %f",total_grav_free_effort);
			//ROS_DEBUG("%f, %f, %f, %f, %f, %f",current_efforts.effort[0],current_efforts.effort[1],current_efforts.effort[2],
			//							current_efforts.effort[3],current_efforts.effort[4],current_efforts.effort[5]);
			//ROS_DEBUG("Changes in efforts: ");
			//ROS_DEBUG("%f, %f, %f, %f, %f, %f",delta_effort[0],delta_effort[1],delta_effort[2],
			//							delta_effort[3],delta_effort[4],delta_effort[5]);
											
			
		
			//ROS_DEBUG("Total change: %f",total_delta);
							//ROS_INFO("total_delta: %f", total_delta);

			if (total_grav_free_effort > 4.0){
				ROS_INFO("total_grav_free_effort: %f", total_grav_free_effort);
				heard_efforts = false;
				//writeToFile(grav_free, delta);
				break;
			}
			//ROS_INFO("Efforts: %f", total_delta);
			if (total_delta > .7){
				ROS_INFO("total_delta: %f", total_delta);
				//writeToFile(grav_free, delta);
				break;
			}
		}	
	}
		T.twist_linear_x= 0;
		T.twist_linear_y=v_y;
		T.twist_linear_z=v_z;//-40.0;
		
		T.twist_angular_x=0.0;
		T.twist_angular_y=0.0;
		T.twist_angular_z=0.0;
		
}

//debug function for reference frame - outputs current position based on origin set
void checkOrigin(){
	ros::Rate r(1000);
	cout << "Current x_origin: " << b_origin_x << endl;
	cout << "Current y_origin: " << b_origin_y << endl;
	cout << "Current z_origin: " << b_origin_z << endl;
	while(true){
		//ros::spinOnce();
		cout << "Current distance from origin x: " << origin_x << endl;
		cout << "Current distance from origin y: " << origin_y << endl;
		cout << "Current distance from origin z: " << origin_z << endl;
		break;
		r.sleep();
	}

}

//Meat and potatos of whole operation - 
//go to point on a newly established coordinate system
//assumes origin has been set
//overloaded gotoPoint that will use a static force to draw rather than detect
void gotoPoint(double x_coord, double y_coord, bool staticForce){
	
	for(int i = 0; i < 1; i++){
		
	float force_thresh = .4;
	double delta_z = -.01;
	double delta_x = x_coord - origin_x;
	double delta_y = y_coord - origin_y;
	double distance = sqrt((delta_x * delta_x) + (delta_y * delta_y));
	double initDis = distance;
	double timeRemaining = 0;
	double xDis = cur_x - poc_x;
	double yDis = cur_y - poc_y;
	double localdrawDistance = sqrt(xDis + yDis);
	xDis *= xDis;
	yDis *= yDis;
	if(debug){
		cout << "At point : " << origin_x << ", " << origin_y << endl;
		cout << "Going to point " << x_coord << ", " << y_coord << endl;
		cout << "base velocity : " << delta_x << ", " << delta_y << endl;
		cout << "distance: " << distance << endl;
	}
	ros::Rate r2(20);
	
	kinova_msgs::PoseVelocity T; 
	double magnitude = 0;
	double alpha = 0;
	double vel = 0.0;
	poc = true;
	if(initDis <= (BASE_VELOCITY * TOOL_POS_RATE))
		vel = BASE_VELOCITY_SHORT;
	else if(traveling)
		vel = TRAVEL_VELOCITY;
	else
		vel = BASE_VELOCITY;
	ros::Duration timeout = ros::Duration(.20);
	ros::Time start = ros::Time::now();
	ros::Rate r3(100);
	while(drawDistance < (initDis - (initDis * .1))){	//.1 as percentage should be paramaratized

		ros::spinOnce();
		delta_x = x_coord - origin_x;
		delta_y = y_coord - origin_y;
		distance = sqrt((delta_x * delta_x) + (delta_y * delta_y));
		magnitude = abs(delta_x + delta_y);
		//ros::Time start = ros::Time::now();
		if(distance <= (BASE_VELOCITY * TOOL_POS_RATE))
			vel = BASE_VELOCITY_SHORT; 
		alpha = vel / magnitude; //a constant that is multiplied to each velocity in order to maintain an overall speed of BASE_VELOCITY
		delta_x	*= alpha;
		delta_y *= alpha;
		ros::spinOnce();
		if(staticForce){
			if(total_delta < (0 + .008)){ //should paramaratize this as a learned variable/threshold
				delta_z -= .0006;
				r3.sleep();
			}
			
			else if (total_delta > (0 - .012)){
				delta_z += .0006;
				r3.sleep();
			}
		}
		else
			delta_z = -.06;
		T.twist_linear_x= delta_x;
		T.twist_linear_y= delta_y;
		
		if(traveling)
			T.twist_linear_z = 0;
		else
			T.twist_linear_z = delta_z;
		
		T.twist_angular_x=0.0;
		T.twist_angular_y=0.0;
		T.twist_angular_z=0.0;
		c_vel_pub_.publish(T);

		if(debug)
			ROS_INFO("Target distance: %f. Calculated distance: %f", distance, drawDistance);
	}
	T.twist_linear_x = 0.0;
	T.twist_linear_y = 0.0;
	T.twist_linear_z = 0.0;
	T.twist_angular_x=0.0;
	T.twist_angular_y=0.0;
	T.twist_angular_z=0.0;
	c_vel_pub_.publish(T);
	
	ros::spinOnce();
	poc = false;
	initPosition = false;
	
	if(debug){
		cout << "At point : " << origin_x << ", " << origin_y << endl;
		ROS_INFO("Target distance: %f. Calculated distance: %f", initDis, drawDistance);
	}
	drawDistance = 0;
	r3.sleep();
	
	ros::Time s = ros::Time::now();
	ros::Duration t = ros::Duration(.05);
	//clears out old effort msgs
	while((ros::Time::now() - s) < t){
		ros::spinOnce();
		//r4.sleep();
	}
	}
	
}

//go to point on coordinate system
//assumes origin has been set (implicit)
void gotoPoint(double x_coord, double y_coord){
	
	for(int i = 0; i < 1; i++){
		
	float force_thresh = .4;
	double delta_z = -.01;
	double delta_x = x_coord - origin_x;
	double delta_y = y_coord - origin_y;
	double distance = sqrt((delta_x * delta_x) + (delta_y * delta_y));
	double initDis = distance;
	double timeRemaining = 0;
	double xDis = cur_x - poc_x;
	double yDis = cur_y - poc_y;
	double localdrawDistance = sqrt(xDis + yDis);
	xDis *= xDis;
	yDis *= yDis;
	if(debug){
		cout << "At point : " << origin_x << ", " << origin_y << endl;
		cout << "Going to point " << x_coord << ", " << y_coord << endl;
		cout << "base velocity : " << delta_x << ", " << delta_y << endl;
		cout << "distance: " << distance << endl;
	}
	ros::Rate r2(20);
	
	kinova_msgs::PoseVelocity T; 
	double magnitude = 0;
	double alpha = 0;
	double vel = 0.0;
	poc = true;
	if(initDis <= (BASE_VELOCITY * TOOL_POS_RATE))
		vel = BASE_VELOCITY_SHORT;
	else if(traveling)
		vel = TRAVEL_VELOCITY;
	else
		vel = BASE_VELOCITY;
	ros::Duration timeout = ros::Duration(.20);
	ros::Time start = ros::Time::now();
	ros::Rate r3(100);

	while(drawDistance < (initDis - (initDis * .1))){	//.1 as percentage should be paramaratized

		ros::spinOnce();
		delta_x = x_coord - origin_x;
		delta_y = y_coord - origin_y;
		distance = sqrt((delta_x * delta_x) + (delta_y * delta_y));
		magnitude = abs(delta_x + delta_y);
		//ros::Time start = ros::Time::now();
		if(distance <= (BASE_VELOCITY * TOOL_POS_RATE))
			vel = BASE_VELOCITY_SHORT; 
		alpha = vel / magnitude; //a constant that is multiplied to each velocity in order to maintain an overall speed of BASE_VELOCITY
		delta_x	*= alpha;
		delta_y *= alpha;
		ros::spinOnce();
		if(total_delta < (0 + .008)){ //should paramaratize this as a learned variable/threshold
			delta_z -= .0006;
			r3.sleep();
		}
		
		else if (total_delta > (0 - .012)){
			delta_z += .0006;
			r3.sleep();
		}

		if(delta_z < -.015)
			delta_z = -.015;
		T.twist_linear_x= delta_x;
		T.twist_linear_y= delta_y;
		
		if(traveling)
			T.twist_linear_z = 0;
		else
			T.twist_linear_z = delta_z;
		
		T.twist_angular_x=0.0;
		T.twist_angular_y=0.0;
		T.twist_angular_z=0.0;
		c_vel_pub_.publish(T);

	}
	T.twist_linear_x = 0.0;
	T.twist_linear_y = 0.0;
	T.twist_linear_z = 0.0;
	T.twist_angular_x=0.0;
	T.twist_angular_y=0.0;
	T.twist_angular_z=0.0;
	c_vel_pub_.publish(T);
	
	ros::spinOnce();
	poc = false;
	initPosition = false;
	
	if(debug){
		cout << "At point : " << origin_x << ", " << origin_y << endl;
		ROS_INFO("Target distance: %f. Calculated distance: %f", initDis, drawDistance);
	}
	drawDistance = 0;
	r3.sleep();
	
	ros::Time s = ros::Time::now();
	//clears out old effort msgs
	clearMsgs(.05);
	}
	
}
//Touches end effector to each corner of the drawing board
//Hardcoded for a demo in the lab with the arm attached to table.
//Used in 'Final Demo'
void establish_bounds(){
	actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ac("/m1n6s200_driver/pose_action/tool_pose", true);
	
	kinova_msgs::ArmPoseGoal goalPose;
	ros::Rate r(30);
	//Only moving in 2d space, orientation stays
	//first point should be the origin
	goalPose.pose.header.frame_id = "m1n6s200_link_base";
	goalPose.pose.pose.position.x = -0.117668524384;	
	goalPose.pose.pose.position.y = -0.412680387497;				
	goalPose.pose.pose.position.z = 0.0538666550815;			
	goalPose.pose.pose.orientation.x = 0.0117474132316;		
	goalPose.pose.pose.orientation.y = -0.998421445202;		
	goalPose.pose.pose.orientation.z = 0.0040660488499;		
	goalPose.pose.pose.orientation.w = 0.0266835433905;		
	ac.waitForServer();
	

	ROS_DEBUG("Waiting for server.");
	//finally, send goal and wait
	ROS_INFO("Sending goal.");
	ac.sendGoal(goalPose);
	ac.waitForResult();
	poc = true;
	setOrigin = true;
	ros::spinOnce();
	goalPose.pose.pose.position.z = 0.0318666550815;
	ac.sendGoal(goalPose);		
	ac.waitForResult();

	lift();
	r.sleep();
	goalPose.pose.pose.position.x = -0.117668524384;	
	goalPose.pose.pose.position.y = -0.32680387497;				
	goalPose.pose.pose.position.z = 0.0537384790182;
	ac.sendGoal(goalPose);
	ac.waitForResult();
	sleep(.1);
	goalPose.pose.pose.position.z = 0.0359666550815;
	ac.sendGoal(goalPose);	
	ac.waitForResult();
	
	goalPose.pose.pose.position.z = 0.0537384790182;
	ac.sendGoal(goalPose);	
	ac.waitForResult();
	
	r.sleep();
	goalPose.pose.pose.position.x = 0.117247706056;	
	goalPose.pose.pose.position.y = -0.32680387497;				
	goalPose.pose.pose.position.z = 0.0537384790182;
	ac.sendGoal(goalPose);
	ac.waitForResult();
	goalPose.pose.pose.position.z = 0.0358666550815;
	ac.sendGoal(goalPose);	
	ac.waitForResult();
	goalPose.pose.pose.position.z = 0.0537384790182;
	ac.sendGoal(goalPose);	
	ac.waitForResult();

	goalPose.pose.pose.position.x = 0.118457129002;	
	goalPose.pose.pose.position.y = -0.412680387497;				
	goalPose.pose.pose.position.z = 0.0537384790182;	
	ac.sendGoal(goalPose);
	ac.waitForResult();
	goalPose.pose.pose.position.z = 0.0358666550815;
	ac.sendGoal(goalPose);	
	ac.waitForResult();
	clearMsgs();
}

//Function parses a .cdcode file and calls the goToPoint function to move end effector to the destination coordinate
void drawFromCD(std::string inputName)
{
	vector <vector <string> > data;
	std::string fullFile = "/home/bwi/catkin_ws/src/segbot_arm/demos/standalone/opencv/drawing_files/" + inputName + ".cdcode";
	ifstream infile( fullFile.c_str() );
	if(!infile){
		cout << endl << "ERROR: Could not file file with that name." << endl;
		cout << "I look for files in the /home/bwi/catkin_ws/src/segbot_arm/demos/standalone/opencv/drawing_files/ folder. Ensure that the file exists." << endl << endl;
	}
	while (infile)
	{
		string s;
		if (!getline( infile, s )) break;
		istringstream ss( s );
		vector <string> record;

		while (ss)
		{
			string s;
			if (!getline( ss, s, ' ' )) break;
			record.push_back( s );
		}

		data.push_back( record );
	}

	int i = 0;
	string input;
	ros::Rate r(200);
	bool onSurface = true;
	double x,y;
	while( i < data.size()){
		vector <string> line = data.at(i);
		if(line.at(0) == "P2P"){
			lift();
			traveling = true;
			onSurface = false;
			cout << line.at(0) << line.at(1) << line.at(2) << endl;
			for(int g = 1; g < line.size(); g+=2){
				std::string in1 = line.at(g).substr(0,1);
				std::string coord1 = (line.at(g).substr(2,6));
				std::string in2 = line.at(g + 1).substr(0,1);
				std::string coord2 = (line.at(g + 1).substr(2,6));
				if(in1 == "X"){
					stringstream ss(coord1);
					ss >> x;
					stringstream dd(coord2);
					dd >> y;
					cout << endl << y;
					gotoPoint(x,y);
					sleep(.3);
					if(debug){
						cout << "At " << i << " out of " << data.size() << endl;
					}
					if(!onSurface){
						gotoPoint(x, y);
						clearMsgs();
						establish_contact("");
						traveling = false;
						onSurface = true;
					}
				}
				if(debug)
					cout << line.at(g+1) << line.at(g);
				r.sleep();
			}
		}
		
		i++;
	}
	lift();
}

//reads and draws from file, but adds a given offset to every point.
//useful for demo drawing several files on one area without having to erase
//otherwise works in same way as previous function
void drawFromCD(std::string inputName, float xOffset, float yOffset, float scale)
{
	vector <vector <string> > data;
	std::string fullFile = "/home/bwi/catkin_ws/src/segbot_arm/demos/standalone/opencv/drawing_files/" + inputName + ".cdcode";
	ifstream infile( fullFile.c_str() );
	if(!infile){
		cout << endl << "ERROR: Could not file file with that name." << endl;
		cout << "I look for files in the /home/bwi/catkin_ws/src/segbot_arm/demos/standalone/opencv/drawing_files/ folder. Ensure that the file exists." << endl << endl;
	}
	while (infile)
	{
		string s;
		if (!getline( infile, s )) break;
		istringstream ss( s );
		vector <string> record;

		while (ss)
		{
			string s;
			if (!getline( ss, s, ' ' )) break;
			record.push_back( s );
		}

		data.push_back( record );
	}

	int i = 0;
	string input;
	ros::Rate r(200);
	bool onSurface = true;
	while( i < data.size()){
		vector <string> line = data.at(i);
		double x, y;
		if(line.at(0) == "P2P"){
			lift();
			traveling = true;
			onSurface = false;
			if(debug)
				cout << line.at(0) << line.at(1) << line.at(2) << endl;
			for(int g = 1; g < line.size(); g+=2){
				std::string in1 = line.at(g).substr(0,1);
				std::string coord1 = (line.at(g).substr(2,6));
				std::string in2 = line.at(g + 1).substr(0,1);
				std::string coord2 = (line.at(g + 1).substr(2,6));
				if(in1 == "X"){
					stringstream ss(coord1);
					ss >> x;
					stringstream dd(coord2);
					dd >> y;
					cout << endl << y;
					gotoPoint((x*scale) + xOffset, (y*scale) + yOffset);
					sleep(.1);
					if(debug){
						cout << "At " << i << " out of " << data.size() << endl;
					}
					if(!onSurface){
						gotoPoint((x*scale) + xOffset, (y*scale) + yOffset);
						establish_contact("");
						traveling = false;
						onSurface = true;
					}
				}
				if(debug)
					cout << line.at(g+1) << line.at(g);
				r.sleep();
			}
		}
		
		i++;
	}
	lift();
}

//draws equalateral triangle
void drawTriangle(double sideLength){
	double basex = origin_x;
	double basey = origin_y;
	double finalX, finalY;
	gotoPoint(basex + sideLength, basey + sideLength);
	gotoPoint(origin_x - sideLength, origin_y);
	gotoPoint(basex, basey);
	lift();
}
//Alternative drawSquare. uses input and gotoPoint rather than rawVelocities - prefered square method
void drawSquareHorizontal(double length){
	double basex = origin_x;
	double basey = origin_y;
	gotoPoint(basex + length, basey);
	gotoPoint(origin_x, basey + length);
	gotoPoint(basex, origin_y);
	gotoPoint(basex, basey);
	lift();
}
//Alternative drawSquare that uses a static force for drawing. Used for a single demo (so far)
void drawSquareHorizontalStatic(double length){
	double basex = origin_x;
	double basey = origin_y;
	gotoPoint(basex + length, basey, true);
	gotoPoint(origin_x, basey + length, true);
	gotoPoint(basex, origin_y, true);
	gotoPoint(basex, basey, true);
	lift();
}
//Works well for larger radii. Mess with step to get good circle
void drawCircle(double radius){
	double vel_x, vel_y;
	double angle = 0;
	float step = .5; //2 / (radius * 100);
	ros::Rate r(10);
	double basex = origin_x;
	double basey = origin_y;
	double finalX, finalY;
	bool firstRun = true;
	while(angle < (2*PI)){
		vel_x = basex + (radius * cos(angle));
		vel_y = basey + (radius * sin(angle));
		if(firstRun){
			finalX = vel_x;
			finalY = vel_y;
			lift();
			gotoPoint(vel_x, vel_y);
			establish_contact("double");
			firstRun = false;
		}
		else
			gotoPoint(vel_x, vel_y);
		angle += step;
		//r.sleep();
		ros::spinOnce();
		//ROS_INFO("Target distance: %f. Calculated distance: %f", targetCircum, drawDistance);
	}
	gotoPoint(finalX, finalY);
	lift();
}

//this function is for demonstrations.
//It is prescripted and the parameters of the shapes are not changable to the user.
//Take note that this requires that a coordinate frame be set.
//There is no check for this ATM
void drawGeometricShapes(){
			//establish_bounds();
			ros::Time start = ros::Time::now();
			ros::Duration timeout = ros::Duration(.7);
			ros::Rate r2(10);
			poc = false;
			//clears out old effort msgs
			clearMsgs();
			traveling = true;
			gotoPoint(.2, .038);
			gotoPoint(.190123, .03361);
			traveling = false;
			//establish_contact("");
			drawCircle(.035); //.03
			traveling = true;
			gotoPoint(.0818003, .00416976);
			traveling = false;
			clearMsgs();
			establish_contact("");
			drawTriangle(.07);
			traveling = true;
			gotoPoint(0.015566, 0.00698176);
			traveling = false;
			clearMsgs();
			establish_contact("double");
			drawSquareHorizontal(.06);
}


//Erases board with dry eraser
void eraseBoard(){
	//clears out old effort msgs
	clearMsgs();
	gotoPoint(.001,.0319028);
	gotoPoint(.001,.0319028);
	establish_contact("double");
	gotoPoint(.24, 0.0319028, true);
	gotoPoint(.24, 0.075792, true);
	gotoPoint(-.03, 0.077792, true);
	lift();
}

void displayIntro(){
		cout << endl << "****Welcome to the BWI Drawing Arm Project program****" << endl;
		cout << "Written by Maxwell Svetlik, in conjuction with Jivko Sinapov" << endl;
		cout << "Latest update: December 26, 2014 2:34pm. Press 'u' in the main menu for a list of updates." << endl << endl;
		cout << "Full demos include grabbing the pen, drawing and returning the pen." << endl;
		cout << "Partial demos assume the end effector is holding a pen." << endl;
}
void displayMenu(){
		cout << "Please select one of the following: " << endl << endl;
		cout << "0 - Exit" << endl;
		cout << "1 - Subfunctions" << endl;
		cout << "2 - Full draw square demo - horizontal" << endl;
		cout << "3 - Full draw a distance line demo - horizontal" << endl;
		cout << "4 - Partial draw distance line demo - horizontal" << endl;
		cout << "5 - Partial draw circle demo - horizontal" << endl;
		cout << "6 - Partial draw writer's name - horizontal" << endl;
		cout << "7 - Partial draw from file" << endl;
		cout << "8 - Full interactive demo (Max's 'Final' Demo)" << endl;
}
void drawFromFileMenu(){
		cout << "In order to draw, we must establish an origin." << endl;
		cout << "Please touch the arm to the writing surface in the proper place " << endl;
		setOrigin = true;
		ros::Rate r (100);
		getWriteReady("horizontal");
		establish_contact(0.00f,0.00f,-0.05f);
		while(!poc){
			ros::spinOnce();
			cout << "Waiting for poc" << endl;
			r.sleep();
		}
		cout << "Origin set!" << endl;
		checkOrigin();
}

void subfunctionsMenu(){
	int input;
	while(true){
		cout << "Please select one of the following: " << endl << endl;
		cout << "0 - Exit Current Menu" << endl;
		cout << "1 - Set Reference Frame" << endl;
		cout << "2 - Go Distance From Origin" << endl;
		cout << "3 - Draw from file" << endl;
		cout << "4 - Send raw velocity command ~ONLY USE IF YOU KNOW WHAT YOURE DOING~" << endl;
		cout << "9 - Output Current Coordinate" << endl;
		cin >> input;
		if(input == 0)
			break;
		else if(input == 1){
			establish_contact("double");
			//establish_contact(0.00f,0.00f,-0.06f);
			poc = true;
			setOrigin = true;
			ros::spinOnce();
		}
		else if(input == 2){
			double in_x;
			double in_y;
			cout << "Please enter an X coordinate (good example: 0.01) ";
			cin >> in_x;
			cout << "Please enter an Y coordinate (good example: 0.01) ";
			cin >> in_y;
			//getWriteReady("horizontal");
			gotoPoint(in_x, in_y);
		}
		//todo: display all cdcode files in folder.
		//recieve input on what file should be drawn
		else if(input == 3){
			ros::spinOnce();
			if(!trackFromOrigin){
				cout << "This function requires that the reference frame be set." << endl;
				cout << "You can set the reference frame in the Subfunctions menu!" << endl;
			}
			else
				//drawFromCD();
				cout << "This function has moved onto the main menu!" << endl;
		}
		else if(input == 4){
			float x,y,z;
			getWriteReady("horizontal");
			cout << "Please enter x: "; cin >> x; 
			cout << endl << "Please enter y: "; cin >> y;
			cout << endl << "Please enter z: "; cin >> z;
			sendRawVelocity(x,y,z);
		}
		else if(input == 9){
			ros::spinOnce();
			cout << "Current x: " << origin_x;
			cout << " y: " << origin_y;
			cout << " z: " << origin_z;
		}
		else
			cout << "Input not valid" << endl;
	}
}

int main(int argc, char **argv)
{
	    char the_path[256];

    getcwd(the_path, 255);
    strcat(the_path, "/");
    strcat(the_path, argv[0]);

    printf("%s\n", the_path);
    
    ros::init(argc, argv, "draw");
    ros::NodeHandle n;
    int receivedUtensil = 0; //change back to 1 for grabby

	//subscriber to distance pub
	//ros::Subscriber distSub = n.subscribe("/poc_distance", 1, distance_cb);
	
	//publisher for cartesian velocity
	c_vel_pub_ = n.advertise<kinova_msgs::PoseVelocity>("/m1n6s200_driver/in/cartesian_velocity", 2);

	//create subscriber to joint angles
	ros::Subscriber sub_angles = n.subscribe ("/m1n6s200_driver/out/joint_state", 1, joint_state_cb);
  
	//create subscriber to tool position topic
	ros::Subscriber sub_tool = n.subscribe("/m1n6s200_driver/out/tool_pose", 1, toolpos_cb);

	//subscriber for fingers
  	ros::Subscriber sub_finger = n.subscribe("/m1n6s200_driver/out/finger_position", 1, fingers_cb);

	string ansChoice;
	displayIntro();
	//Main menu
	while(true){
		signal(SIGINT, sig_handler);	

		displayMenu();
		cin >> ansChoice;
		if(ansChoice == "0")
			break;
		else if(ansChoice == "1"){
			//display secondary menu
			subfunctionsMenu();
		}
		else if(ansChoice == "2"){
			getWriteReady("grab");
			receivedUtensil = openFingersAndWait();
			getWriteReady("horizontal");
			establish_contact(0.00f,0.00f,-0.10f);
			sleep(.2);
			establish_contact(0.00f,0.00f,-0.05f);
			sleep(.2);
			drawSquareHorizontal();
			getWriteReady("grab");
			sleep(.4);
			forceFeltDrop();
		}
		else if(ansChoice == "3"){
			double inputDistance = 0;
			cout << "Please enter a distance in cm: ";
			cin >> inputDistance;
			getWriteReady("grab");
			receivedUtensil = openFingersAndWait();
			getWriteReady("horizontal");
			establish_contact(0.00f,0.00f,-0.10f);
			sleep(.3);
			establish_contact(0.00f,0.00f,-0.06f);
			inputDistance /= 100; //conversion from cm to meters
			drawDistanceLine(inputDistance);
			getWriteReady("grab");
			sleep(.3);
			forceFeltDrop();
		}
		else if(ansChoice == "4"){
			double inputDistance = 0;
			cout << "Please enter a distance in cm: ";
			cin >> inputDistance;
			inputDistance /= 100; //conversion from cm to meters
			drawDistanceLine(inputDistance);
		}
		else if(ansChoice == "5"){
			double rad = .035;
			cout << endl <<"This function will establish a new reference frame." << endl;
			cout << "If the drawing doesn't come out as planned, try moving the arm to the center of its range of motion and run the function again." << endl << endl;
			cout << endl << "Please enter a radius in METERS (ex: .035): ";
			cin >> rad;
			establish_contact("double");
			poc = true;
			setOrigin = true;
			ros::Time start = ros::Time::now();
			ros::Duration timeout = ros::Duration(1.0);
			ros::Rate r2(50);
			//clears out old effort msgs
			while( (ros::Time::now() - start) < timeout){
				ros::spinOnce();
				r2.sleep();
			}
			if(!trackFromOrigin){
				cout << "This function requires that the reference frame be set." << endl;
				cout << "You can set the reference frame in the Subfunctions menu!" << endl;
			}
			else{
				drawCircle(rad);
			}
		}
		else if(ansChoice == "6"){
			cout << "This function will establish a new reference frame." << endl;
			cout << "If the drawing doesn't come out as planned, try moving the arm to the center of its range of motion and run the function again." << endl << endl;
			sleep(.5);
			establish_contact("double");
			poc = true;
			setOrigin = true;

			//could possibly use the clearMsgs function
			//but should test it in this instance - might need 1.0 seconds rather than the .7 in the clearMsgs 
			ros::Time start = ros::Time::now();
			ros::Duration timeout = ros::Duration(1.0);
			ros::Rate r2(50);
			//clears out old effort msgs
			while( (ros::Time::now() - start) < timeout){
				ros::spinOnce();
				r2.sleep();
			}			
			if(!trackFromOrigin){
				cout << "This function requires that the reference frame be set." << endl;
				cout << "You can set the reference frame in the Subfunctions menu!" << endl;
			}
			else{
				drawFromCD("max");
				//output_static_efforts();
			}
		}
		else if(ansChoice == "7"){
			std::string input;
			cout << "This function will establish a new reference frame." << endl;
			cout << "If the drawing doesn't come out as planned, try moving the arm to the center of its range of motion and run the function again." << endl << endl;
			cout << "Please enter the name of the file you'd like to see: ";
			cin >> input;
			sleep(.5);
			establish_contact("double");
			poc = true;
			setOrigin = true;
			ros::Time start = ros::Time::now();
			ros::Duration timeout = ros::Duration(1.0);
			ros::Rate r2(50);
			//clears out old effort msgs
			while( (ros::Time::now() - start) < timeout){
				ros::spinOnce();
				r2.sleep();
			}			
			if(!trackFromOrigin){
				cout << "This function requires that the reference frame be set." << endl;
				cout << "You can set the reference frame in the Subfunctions menu!" << endl;
			}
			else{
				drawFromCD(input);
				//output_static_efforts();
			}
		}
		else if(ansChoice == "8"){
			//**start demo 1 - demonstrating the basics of force detection, and drawing shapes
			
			getWriteReady("grab");
			openFingersAndWait();
			establish_bounds();
			ros::spinOnce();
			drawGeometricShapes();
			clearMsgs();
			//erasing sequence
			getWriteReady("grab");
			forceFeltDrop();
			openFingersAndWait();
			getWriteReady("horizontal");
			eraseBoard();
			getWriteReady("grab");
			forceFeltDrop();
			
			//end of demo 1
			//**start of demo 2 - demonstates force detection while drawing
			//compares two squares, one drawn with force detection and one static negative pressure
			//status: base functionality done: could add a "D" and "S" in the respective squares for dynamic and static
			
			openFingersAndWait();
			getWriteReady("horizontal");	
			clearMsgs();
			ros::spinOnce();
			traveling = true;
			gotoPoint(.1418003, .02361);
			gotoPoint(.1418003, .02361);
			traveling = false;
			establish_contact("");
			clearMsgs();
			drawSquareHorizontal(.04);
			traveling = true;
			gotoPoint(.05502, .023361);
			traveling = false;
			establish_contact("");
			clearMsgs();
			drawSquareHorizontalStatic(.04);
			
			//start of demo 3
			//draws some text
			moveAndPause();
			drawFromCD("max", .0662, .0253, 1.8);
			moveAndPause();		
			drawFromCD("bwi2", .07, -.06, 1.6);
			moveAndPause();

			
			//start of demo 4
			//draws a house and a cube drawn in gimp
			drawFromCD("barn2", .01, .01, .83);
			drawFromCD("cube2",0.119, 0, .8);
			//drawFromCD("barn2", .01, -.01, 1); - OLD
			//drawFromCD("cube",0.119, 0, 1); - OLD
			moveAndPause();

			//demo 5
			//draws a longhorn
			drawFromCD("longhorn", .05, -.015, 1);
			//sleep(1);
			
			//demo 6 - mona lisa
			moveAndPause();	
			drawFromCD("mona", .04, -.02, .9);
			
		}
		else if(ansChoice == "u"){
			cout << endl << "Update List: "<< endl;
			cout << "11/26/14 - Circle function on main menu tweaked and working." << endl;
			cout << "11/26/14 - Writing from menu item added into sub-menu." << endl;
			cout << "11/26/14 - Added update list" << endl;
			cout << "11/26/14 - Added lift function" << endl;
			cout << "11/26/14 - Added a modified establish_contact" << endl;
			cout << "11/26/14 - Added a new demo to main menu" << endl;
			cout << "11/26/14 - Implemented contact-tracking -needs improvement" << endl;
			cout << "12/01/14 - Rewrote contact-tracking " << endl;
			cout << "12/01/14 - Added velocity check for small distances" << endl;
			cout << "12/03/14 - Added Debug mode" << endl;
			cout << "12/03/14 - Changed image-coordinate line detection algorithm" << endl;
			cout << "12/04/14 - Added 'final' demo" << endl;
			cout << "12/05/14 - Added drawTriangle, drawSquare, and eraseBoard functions" << endl;
			cout << "12/05/14 - Continued to refine image-coordinate alg. (getting better!)" << endl;
			cout << "12/05/14 - Made draw from file partial demo on main menu" << endl;
			cout << "12/06/14 - Establish_contact function faster, more reliable" << endl;
			cout << "12/06/14 - Contact tracking improved" << endl;
			cout << "12/06/14 - Additional alg tweaking. Works well for perfectly straight lines and smaller perfect curves" << endl;
			cout << "12/07/14 - Drawing velocity now slows as its approaching its target point. Noticible increase in accuracy" << endl;
			cout << "12/07/14 - The arm now checks if it has successfully grabbed anything in the openFingersAndWait function" << endl;
			cout << "12/08/14 - Edited force feedback for drawing" << endl;
			cout << "12/10/14 - Rehaul of demo" << endl;
			cout << "12/23/14 - Added additional comments. General code cleanup.";
			cout << "1/14/15  - Consolidated drawing code into folder, updated code to reflect changes.";
			cout << endl;
		}
		else if(ansChoice == "d" || ansChoice == "v"){
			if(debug){
				cout << "Debug/verbose mode disabled!" << endl;
				debug = false;
			}
			else{
				cout << "Debug/verbose mode enabled!" << endl;
				debug = true;
			}
		}
		else{
			cout << "Input is either invalid or the function is not implemented yet." << endl;
		}
	}
	
	
    return 0;
}
