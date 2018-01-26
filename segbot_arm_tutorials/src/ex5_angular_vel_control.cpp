#include <ros/ros.h>
#include <ros/package.h>

//used for registering Ctrl-C events
#include <signal.h>

//ROS messages
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

//ROS action client used for calling actions
#include <actionlib/client/simple_action_client.h>

//JACO messages and actions
#include <kinova_msgs/FingerPosition.h>
#include <kinova_msgs/JointAngles.h>

//our own arm library 
#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/MicoManager.h>

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
	g_caught_sigint = true;
	ROS_INFO("caught sigint, init shutdown sequence...");
	ros::shutdown();
	exit(1);
};


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
	ros::init(argc, argv, "ex5_angular_vel_control");
	
	ros::NodeHandle n;
	MicoManager mico(n);
	 
	//publish cartesian tool velocities
	ros::Publisher pub_angular_velocity = n.advertise<kinova_msgs::JointAngles>("/m1n6s200_driver/in/joint_velocity", 10);

	//register ctrl-c
	signal(SIGINT, sig_handler);

	pressEnter("Press [Enter] to rotate the wrist");
	
	kinova_msgs::JointAngles angular_vel;
	angular_vel.joint1 = 0.0;
	angular_vel.joint2 = 0.0;
	angular_vel.joint3 = 0.0;
	angular_vel.joint4 = 0.0;
	angular_vel.joint5 = 0.0;
	angular_vel.joint6 = 45;

	double duration = 5.0; //5 seconds
	double elapsed_time = 0.0;
	
	double pub_rate = 100.0;
	ros::Rate r(pub_rate);
	
	while (ros::ok()){
		//collect messages
		ros::spinOnce();
		
		//publish velocity message
		pub_angular_velocity.publish(angular_vel);
		
		r.sleep();
		
		elapsed_time += (1.0/pub_rate);
		if (elapsed_time > duration)
			break;
	}
	
	//publish 0 velocity command -- otherwise arm will continue moving with the last command for 0.25 seconds
	angular_vel.joint6 = 0;
	pub_angular_velocity.publish(angular_vel);

	//With less code...
	angular_vel.joint6 = -45;
	mico.move_with_angular_velocities(angular_vel, 5);

	ros::shutdown();
}
