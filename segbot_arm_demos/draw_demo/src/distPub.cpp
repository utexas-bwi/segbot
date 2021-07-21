/*
* A publisher that publishes a floating point number that represents the distance (in meters)
* that the end effector has gone since its z coordinate was [effectively] zero 
* ie the point of contact. This message will hang after z increases, but will be reset
* with another POC.
* 
*/



#include <ros/ros.h>
#include <math.h>
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseStamped.h"


ros::Publisher joint_state_pub;
ros::Publisher c_vel_pub_;

//should make these a message
bool poc = false;
double poc_x, poc_y, poc_z;
double cur_x, cur_y, cur_z;



void toolpos_cb(const geometry_msgs::PoseStamped &msg){
	ros::Rate r(100);
	geometry_msgs::PoseStamped current = msg;
	cur_x = current.pose.position.x;
	cur_y = current.pose.position.y;
	cur_z = current.pose.position.z;
	if(current.pose.position.z >= 0.045 && current.pose.position.z <= 0.052 && !poc){
		ROS_INFO("Detected POC!");
		poc_x = current.pose.position.x;
		poc_y = current.pose.position.y;
		poc_z = current.pose.position.z;
		poc = true;
	}
	if(current.pose.position.z < 0.045 || current.pose.position.z > 0.052){
		//ROS_INFO("Detected lift!");
		poc = false;
	}
	

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "distancePublisher");
    ros::NodeHandle n;
    double distance = 0;

	//Most important to track. Will use this value directly and potentially exclusively to stay within the reference frame of the board
	//subscriber for cart velocity
	//ros::Subscriber vel_sub = n.subscribe("/m1n6s200_driver/in/cartesian_velocity", 100, velocity_cb);

	//Subscriber for the arm pose. Coordinate frame is essentially base_frame
	//Possibly will need to take pose movement into account, but likely not. Will really only be using this for rotating end effector for lift compensation
	ros::Subscriber sub = n.subscribe("/m1n6s200_driver/out/tool_pose", 10, toolpos_cb);

	ros::Publisher distance_pub = n.advertise<std_msgs::Float32>("poc_distance", 100);
	std_msgs::Float32 msg;
	ros::Rate loop_rate(10);
	
	
	while(ros::ok()){
		ros::spinOnce();
		//if(!poc) //end effector is above z threshold
		//	msg.data = distance;
		if(poc){	//end effector is currently within threshold
			float xDis = cur_x - poc_x;
			float yDis = cur_y - poc_y;
			xDis *= xDis;
			yDis *= yDis;
			distance = sqrt(xDis + yDis);
			msg.data = distance;
		}
		//ROS_INFO("curx cury pocx pocy %f %f %f %f", cur_x, cur_y, poc_x, poc_y);
		//ROS_INFO("Distance: %f", distance);
		distance_pub.publish(msg);
		loop_rate.sleep();
	}

	//Used for joint positions. Likely won't be tracked. In fact, should probably use this to rotate the end effector to compensate for lift
	//create subscriber to joint angles
	//ros::Subscriber sub = n.subscribe ("/m1n6s200_driver/out/joint_state", 1, joint_state_cb);
	

    return 0;
}
