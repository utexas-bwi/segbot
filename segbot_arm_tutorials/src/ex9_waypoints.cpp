#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinova_msgs/FingerPosition.h>

//our own arm library 
#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/Mico.h>


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
    ros::init(argc, argv, "ex9_waypoints");
    
    ros::NodeHandle n;

    //register ctrl-c
    signal(SIGINT, sig_handler);

    bool success = false;
    segbot_arm_manipulation::Mico mico(n);

    mico.move_home();

    pressEnter("Press enter to move through waypoints with Kinova firmware...");


    geometry_msgs::Pose pose;

    tf::Quaternion quat;
    quat.setX(0.58);
    quat.setY(0.39);
    quat.setZ(0.37);
    quat.setW(0.61);
    quat.normalize();
    tf::quaternionTFToMsg(quat, pose.orientation);

    geometry_msgs::Pose to_add;
    std::vector<geometry_msgs::Pose> waypoints;
    pose.position.x = 0.2;
    pose.position.y = -0.25;
    pose.position.z = 0.5;
    to_add = pose;
    waypoints.push_back(to_add);
    pose.position.x = 0.4;
    pose.position.y = -0.25;
    pose.position.z = 0.5;
    to_add = pose;
    waypoints.push_back(to_add);
    pose.position.x = 0.2;
    pose.position.y = -0.25;
    pose.position.z = 0.5;
    to_add = pose;
    waypoints.push_back(to_add);
    mico.move_through_waypoints(waypoints);
    mico.move_home();

    pressEnter("Press enter to move through waypoints with MoveIt...");
    success = mico.move_through_waypoints_moveit(waypoints);
    ROS_INFO("Waypoint traversal status: %s", success ? "true": "false");
    mico.move_home();
    ros::shutdown();
}
