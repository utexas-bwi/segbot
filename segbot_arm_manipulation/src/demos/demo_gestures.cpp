#include <ros/ros.h>
#include <ros/package.h>

#include <signal.h>

#include <sensor_msgs/JointState.h>

//srv for talking to table_object_detection_node.cpp
#include <bwi_perception/PerceiveTabletopScene.h>

//actions
#include "segbot_arm_manipulation/TabletopGraspAction.h"
#include "segbot_arm_manipulation/TabletopApproachAction.h"
#include "segbot_arm_manipulation/LiftVerifyAction.h"

#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/Mico.h>

#include <bwi_perception/bwi_perception.h>
#include <bwi_manipulation/GraspCartesianCommand.h>

#include <move_base_msgs/MoveBaseAction.h>


#include <segbot_arm_manipulation/HandoverAction.h>


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

#define NUM_JOINTS 8 //6+2 for the arm

using namespace std;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

segbot_arm_manipulation::Mico *mico;


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
	ros::init(argc, argv, "demo_gestures");

	ros::NodeHandle n;

	mico = new segbot_arm_manipulation::Mico(n);
	//register ctrl-c
	signal(SIGINT, sig_handler);


	ROS_INFO("Starting pointing demo");
	ROS_INFO("Moving arm out of view");
	mico->open_hand();
	mico->close_hand();



	mico->move_to_side_view();
	mico->move_to_side_view();
	//   mico->move_home();

	pressEnter("Press [ENTER] when safe to point straight");

	mico->point_straight();
	pressEnter("Press [ENTER] when safe to point right");
   	mico->point_right();
	pressEnter("Press [ENTER] when safe to point left");
    	mico->point_left();
	pressEnter("Press [ENTER] when safe to proceed");

	mico->move_to_side_view_approach();
	mico->move_to_side_view();
	mico->move_to_side_view();
	mico->open_hand();
	mico->close_hand();


	return 0;
}
