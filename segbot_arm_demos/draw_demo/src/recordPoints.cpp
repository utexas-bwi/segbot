#include <ros/ros.h>

#include <signal.h> 

#include <iostream>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include "kinova_msgs/JointAngles.h"
#include "kinova_msgs/ArmJointAnglesAction.h"

//file manip
#include <cstdlib>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <cstring>

/*joint state message example:
 * 
 * header: 
  seq: 77830
  stamp: 
    secs: 0
    nsecs: 0
  frame_id: ''
name: ['jaco_joint_1', 'jaco_joint_2', 'jaco_joint_3', 'jaco_joint_4', 'jaco_joint_5', 'jaco_joint_6']
position: [-2.001995130476374, -1.1357473911256981, 0.328211701166913, -1.0269676469242055, -5.246697924623197, -0.20666289848681654]
velocity: []
effort: []


*/



using namespace std;

static std::vector<vector<float> > trajectory;


//true if Ctrl-C is pressed
bool g_caught_sigint=false;

bool recording = false;
int state = 1;
int max_num_points = 100;


void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

void callBack(const sensor_msgs::JointStateConstPtr &msg)
{
	ros::Rate r(100);
	if (recording){
		std::vector<float> temp;
		temp.push_back(msg->position[0]);
		temp.push_back(msg->position[1]);
		temp.push_back(msg->position[2]);
		temp.push_back(msg->position[3]);
		temp.push_back(msg->position[4]);
		temp.push_back(msg->position[5]);
		//ROS_INFO("Got joint 1: %f joint 2: %f joint 3: %f joint 4: %f joint 5: %f joint 6: %f", msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5]);
		trajectory.push_back(temp);
	}
	r.sleep();
}

void playback(){
		actionlib::SimpleActionClient<kinova_msgs::ArmJointAnglesAction> ac("/m1n6s200_driver/joint_angles/arm_joint_angles", true);
		
		kinova_msgs::ArmJointAnglesGoal goal;
		std::vector<float> last = trajectory.at(0);
		
		ROS_INFO("Target position: %f, %f, %f, %f, %f, %f",last[0],last[1],last[2],last[3],last[4],last[5]);
		
		goal.angles.joint1 = last[0];
		goal.angles.joint2 = last[1];
		goal.angles.joint3 = last[2];
		goal.angles.joint4 = last[3];
		goal.angles.joint5 = last[4];
		goal.angles.joint6 = last[5];
		ac.waitForServer();
		ac.sendGoal(goal);
		while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_INFO("Still trying");
		}
		ac.waitForResult();
}

void threadCallback(){
	ros::NodeHandle n;
	
	//subscribers	
	ros::Subscriber sub = n.subscribe("m1n6s200_driver/out/joint_angles", 10, callBack);
  		
}

void writeToFile(vector<float> angles, string pointName){
	fstream filestr;

 	 filestr.open("src/robot_arm/config/common_angles.txt", fstream::in | fstream::out | fstream::app);
 	 filestr << pointName << ",";
 	 for(int i = 0; i < angles.size() - 1; i++){
 	 	filestr << angles.at(i) << ",";
 	 }
 	 filestr << angles.back() << endl; //adds the last element without a space at the end.
 	 filestr.close();
}

//not implemented yet
void readFile(vector<float> angles, string pointName){
	std::ifstream  data("plop.csv");

    std::string line;
    while(std::getline(data,line))
    {
        std::stringstream  lineStream(line);
        std::string        cell;
        while(std::getline(lineStream,cell,','))
        {
            //stuff
        }
    }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "recordPoints");
	
	
	//register ctrl-c
	signal(SIGINT, sig_handler);	
		
	//Control loop
	bool done = false;
	
	char input;
	ros::NodeHandle n;
	
	//subscribers	
	ros::Subscriber sub = n.subscribe("/m1n6s200_driver/out/joint_state", 10, callBack);
	
	cout << "I am a joint angle recorder program." << endl;
	
	while (ros::ok()){
		signal(SIGINT, sig_handler);
		cout << "Enter 1 to record the current position of the arm to file." << endl;
		cin >> input;
		recording = true;
		ros::spinOnce();
		std::vector<float> first = trajectory.at(0);
		//ROS_INFO("I've recieved the following joint angles: %f, %f, %f, %f, %f, %f",first[0],first[1],first[2],first[3],first[4],first[5]);
		std::string name;
		//std::vector<float> first;
		//first.push_back(1);first.push_back(1);first.push_back(1);first.push_back(1);first.push_back(1);first.push_back(1);
		ROS_INFO("Please input a name for the point: ");
		cin >> name;
		writeToFile(first, name);
	return 0;
	}
}
