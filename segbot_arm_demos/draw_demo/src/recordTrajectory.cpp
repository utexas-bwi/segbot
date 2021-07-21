#include <ros/ros.h>

#include <signal.h> 

#include <iostream>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include "kinova_msgs/JointAngles.h"
#include "kinova_msgs/ArmJointAnglesAction.h"
#include "kinova_msgs/ArmPoseAction.h"


#include <iostream>
#include <fstream>


std::ofstream je;
std::ofstream js;
std::ofstream ja;

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
int state = 3;
int max_num_points = 100;


void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

/*void callBack(const sensor_msgs::JointStateConstPtr &msg)
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
		ROS_INFO("Got joint 1: %f joint 2: %f joint 3: %f joint 4: %f joint 5: %f joint 6: %f", msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5]);
		trajectory.push_back(temp);
	}
	r.sleep();
}*/

void playback(){
		actionlib::SimpleActionClient<kinova_msgs::ArmJointAnglesAction> ac("/m1n6s200_driver/joint_angles/arm_joint_angles", true);
		
		kinova_msgs::ArmJointAnglesGoal goal;
		std::vector<float> last = trajectory.at(0);
		
		ROS_INFO("Target position: %f, %f, %f, %f, %f, %f",last[0],last[1],last[2],last[3],last[4],last[5]);
		
		
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
	//ros::Subscriber sub = n.subscribe("m1n6s200_driver/out/joint_angles", 10, callBack);
  		
}
void je_callBack(const sensor_msgs::JointStateConstPtr &msg){
		std::cout << "Efforts: " << msg->effort[0] << std::endl;//<< msg->effort[1] << msg->effort[2] << msg->effort[3] << msg->effort[4] << msg->effort[5] << std::endl;
}
/*void js_callBack(const kinova_msgs::JointAnglesPtr &msg){
		std::string data;
		for (int i = 0; i < 6; i ++){
			data += msg->velocity[i];
			data += " ";
		}		
		js << data;
}*/
void ja_callBack(const kinova_msgs::JointAnglesPtr &msg){
		//std::cout << "Angles: " << msg->joint1 << std::endl; //<< msg->joint2 << msg->joint3 << msg->joint4 << msg->joint5 << msg->joint6 << std::endl;
}
//quick function to send a jointstate goal and note the efforts
//used for verification of kinematic model
void recordEfforts(){

		const char *je_path="/home/bwi/max_arm_exp/je.txt";
		const char *js_path="/home/bwi/max_arm_exp/js.txt";
		const char *ja_path="/home/bwi/max_arm_exp/ja.txt";

		std::ofstream je(je_path); //open in constructor
		//std::ofstream js(js_path); //open in constructor
		std::ofstream ja(ja_path); //open in constructor


		actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ac("/m1n6s200_driver/pose_action/tool_pose", true);
	
		kinova_msgs::ArmPoseGoal goalPose;
		ros::spinOnce();
		//Only moving in 2d space, orientation stays
		goalPose.pose.header.frame_id = "m1n6s200_link_base";
		goalPose.pose.pose.position.x = 0.151496844;	
		goalPose.pose.pose.position.y = -0.3651103675;				
		goalPose.pose.pose.position.z = 0.3085489869;			
		goalPose.pose.pose.orientation.x = 0.20792265145;		
		goalPose.pose.pose.orientation.y = 0.68410680388;		
		goalPose.pose.pose.orientation.z = -0.232979253411;		
		goalPose.pose.pose.orientation.w = 0.659156066027;
		ac.waitForServer();
		ROS_DEBUG("Waiting for server.");
		//finally, send goal and wait
		//ROS_INFO("Sending goal.");
		ac.sendGoal(goalPose);
		je.close();
		ja.close();
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "recordTrajectory");
	
	
	//register ctrl-c
	signal(SIGINT, sig_handler);	
		
	//Control loop
	bool done = false;
	
	char input;
	ros::NodeHandle n;
	
	//subscribers	
	ros::Subscriber je_sub = n.subscribe("m1n6s200_driver/out/joint_state", 10, je_callBack);

	ros::Subscriber ja_sub = n.subscribe("m1n6s200_driver/out/joint_angles", 10, ja_callBack);
	
	/*cout << endl << "1 - Record Tracjectories" << endl;
	cin >> input;*/

	//create multithreaded spinner, spinning on the subscription callback
	//ros::MultiThreadedSpinner spinner(2); //use two threads
    //spinner.spin(&threadCallback());
	
	
	while (ros::ok()){
		signal(SIGINT, sig_handler);	
		if (state == 1){
			cout << "Enter 1 to output joint positions for 5 seconds." << endl;
			cin >> input;
			if(input == '1'){
				ros::Time start = ros::Time::now();
				ros::Duration cutoff = ros::Duration(5.0);
				recording = true;
				while(ros::Time::now() - start < cutoff){
					ros::spinOnce();
				}
				state = 2;
			}
		}
		if (state == 2){
			cout << "Enter 1 to play back the last trajectory. 0 to exit." << endl;
			cin >> input;
			if(input == '1')
				playback();
			else
				return 0;
			/*if((int)trajectory.size() > max_num_points){
				recording = false;
				state = 3;
				cout << "Trajectory recorded" << endl;
			}*/
		}
		if (state == 3){
			recordEfforts();
		}
	}
			
	/*if(input == '1'){
		cout << "Beginning the recording of trajectories in 2 seconds..." << endl;
		cout << "Press Enter to start recording." << endl;
		cin >> input;
		recording=true;
		
		cout << "Press Enter to stop recording." << endl;
		cin >> input;
		recording=false;
		
		cout << "Recorded a trajectory with " << trajectory.size() << " points\n";
		
  		
  		//should make a control. 10 seconds, or a certain number of joint positions (which is basically seconds * 100 or whatever the frequency is * 6 joint positions)
  		std::string response = "";
  		while(response.compare("") == 0){
  			cin >> response;
  			ros::spinOnce();
  		}
  		
  		done = true;
			cout << "Recording ended. I should print out contents of recording and then play it back to you." << endl;
  			playback();
		}
		else
			cout << endl << "Invalid, or something went wrong. Try again." << endl;
	}		*/
	

	return 0;
}
