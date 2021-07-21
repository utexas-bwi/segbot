#include <ros/ros.h>

#include <signal.h> 

#include <iostream>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include "kinova_msgs/JointAngles.h"
#include "kinova_msgs/ArmPoseAction.h"
#include <math.h> 



//file manipulation
//file manip
#include <cstdlib>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <cstring>

#define PI 3.14159265

/*cartesian position message example:
 * 
 * header: 
  seq: 77830
  stamp: 
    secs: 0
    nsecs: 0
  frame_id: 'base_footprint'
twist:
	linear:
		x: 0.12313
		y: -0.4123
		z: 0.2342
	angular
		x: 1.6234
		y: 0.7134
		z: 6.2168


*/



using namespace std;

static std::vector<geometry_msgs::PoseStamped> armPoses;


//true if Ctrl-C is pressed
bool g_caught_sigint=false;

bool recording = false;
int state = 1;
int max_num_points = 100;
vector<double> coord;

void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

void callBack(const geometry_msgs::PoseStamped &msg)
{
	armPoses.push_back(msg);
	ROS_INFO("Got current pose.");	
}


//current issues: string -> double. Need to either cast string to double or
//don't stuff into two vectors of strings, and decode them into x and y vectors
//and then read them sequentially
void drawFromCD()
{
	vector <vector <string> > data;
	ifstream infile( "src/robot_arm/w_text.cdcode" );
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
	while( i < 1){//data.size()){
		vector <string> line = data.at(i);
		double x, y;
		if(line.at(0) == "P2P"){
			//cout << line.at(0) << line.at(1) << line.at(2) << endl;
			for(int g = 1; g < line.size(); g+=2){
				std::string in1 = line.at(g).substr(0,1);
				std::string coord1 = (line.at(g).substr(2,6));
				std::string in2 = line.at(g + 1).substr(0,1);
				std::string coord2 = (line.at(g + 1).substr(2,6));
				if(in1 == "X"){
					//gotoPoint(coord1, coord2);
					stringstream ss(coord1);
					ss >> x;
					stringstream dd(coord2);
					dd >> y;
					//cout << endl << y;
					coord.push_back(x);
					coord.push_back(y);
					//cin >> input;
				}
				else
					//gotoPoint(coord2, coord1);
					cout << line.at(g+1) << line.at(g);
				r.sleep();
			}
		}
		
		i++;
	}
}

void playback(){

		
		
    /*

     Python cartesian command example:

     """Send a cartesian goal to the action server."""
    action_address = '/' + str(sys.argv[1]) + '_arm_driver/arm_pose/arm_pose'
    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=(str(sys.argv[1]) + '_api_origin'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    client.send_goal(goal)*/



}

void drawFile(){
		actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ac("/m1n6s200_driver/pose_action/tool_pose", true);
		

		
		kinova_msgs::ArmPoseGoal goalPose;
		geometry_msgs::PoseStamped curPosition = armPoses.back(); //reasonably assumes that the last pose is the current position of the arm.

		ROS_INFO("Current pose in x y z, x y z w : %f %f %f %f %f %f %f", curPosition.pose.position.x, curPosition.pose.position.y, curPosition.pose.position.z, 
							curPosition.pose.orientation.x, curPosition.pose.orientation.y, curPosition.pose.orientation.z, curPosition.pose.orientation.w);
		
		//Only moving in 2d space, orientation stays
		goalPose.pose.header.frame_id = "m1n6s200_link_base";	
		goalPose.pose.pose.position.x = curPosition.pose.position.x;
		goalPose.pose.pose.position.y = curPosition.pose.position.y;		
		goalPose.pose.pose.position.z = curPosition.pose.position.z;
		goalPose.pose.pose.orientation.x = curPosition.pose.orientation.x;
		goalPose.pose.pose.orientation.y = curPosition.pose.orientation.y;
		goalPose.pose.pose.orientation.z = curPosition.pose.orientation.z;
		goalPose.pose.pose.orientation.w = curPosition.pose.orientation.w;


		//wait for server to come up. only needed once.
		ac.waitForServer();
		
		double oldX = curPosition.pose.position.x;
		double oldY = curPosition.pose.position.y;
		
		for(int i = 0; i < coord.size(); i+=2){
			goalPose.pose.pose.position.x = coord.at(i) + oldX;
			goalPose.pose.pose.position.y = coord.at(i+1) + oldY;
			//goalPose.pose.pose.position.z = 0.0474442467093;
			ac.sendGoalAndWait(goalPose);
		}
}


//draws a circle using cartesian action
//maybe could specify the board size, and a scale factor
void drawCircle(){
		actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ac("/m1n6s200_driver/pose_action/tool_pose", true);
		

		
		kinova_msgs::ArmPoseGoal goalPose;
		geometry_msgs::PoseStamped curPosition = armPoses.back(); //reasonably assumes that the last pose is the current position of the arm.

		ROS_INFO("Current pose in x y z, x y z w : %f %f %f %f %f %f %f", curPosition.pose.position.x, curPosition.pose.position.y, curPosition.pose.position.z, 
							curPosition.pose.orientation.x, curPosition.pose.orientation.y, curPosition.pose.orientation.z, curPosition.pose.orientation.w);
		
		//Only moving in 2d space, orientation stays
		goalPose.pose.header.frame_id = "m1n6s200_link_base";			
		goalPose.pose.pose.position.z = curPosition.pose.position.z;
		goalPose.pose.pose.orientation.x = curPosition.pose.orientation.x;
		goalPose.pose.pose.orientation.y = curPosition.pose.orientation.y;
		goalPose.pose.pose.orientation.z = curPosition.pose.orientation.z;
		goalPose.pose.pose.orientation.w = curPosition.pose.orientation.w;


		//wait for server to come up. only needed once.
		ac.waitForServer();

		//draw circle for one revolution
		float x,y;
		float oldX = curPosition.pose.position.x;
		float oldY = curPosition.pose.position.y;
		float circleLength = .01;
		float angle = 0.0;
		float step = .2;
		while(angle < 2*PI){


			x = (circleLength * cos(angle))+oldX;
			y = (circleLength * sin(angle))+oldY;
			goalPose.pose.pose.position.x = x;
			goalPose.pose.pose.position.y = y;
			//goalPose.pose.pose.position.z = curPosition.pose.position.z;
			angle += step;
			/*x += oldX;
			y += oldY;
			oldX = x;
			oldY = y;*/


			//finally, send goal and wait
			ac.sendGoalAndWait(goalPose);
			//ac.waitForResult();
			//ROS_INFO("The current angle is: %f", angle);
			ROS_INFO("%f %f", x, y);
		}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "shapes");
	ros::NodeHandle n("~");
	
	
	ros::Rate r(100);
	
	//register ctrl-c
	signal(SIGINT, sig_handler);	
	
	
	
	//subscribers	
	ros::Subscriber sub = n.subscribe("/m1n6s200_driver/out/tool_pose", 10, callBack);
	
	//wait for first position
	while (true){
		ros::spinOnce();
		if(armPoses.size() > 0)
			break;
	}
	
	

	//draw a circle (hopefully)
	drawCircle();
	//drawFromCD();
	//drawFile();
	

	return 0;
}
