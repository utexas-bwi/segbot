#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinova_msgs/FingerPosition.h>

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
    ros::init(argc, argv, "ex7_cartesian_pose_control");
    
    ros::NodeHandle n;

    //register ctrl-c
    signal(SIGINT, sig_handler);
        
    MicoManager mico(n);

    pressEnter("Press enter to move to candle pose with MoveIt");

    geometry_msgs::PoseStamped p_target;
    // Pose with the end effector pointing straight up, with palm parallel with the frame of the robot
    p_target.header.frame_id = "m1n6s200_link_base";
    p_target.pose.position.x = 0.00;
    p_target.pose.position.y = -0.05;
    p_target.pose.position.z = 0.97;
    p_target.pose.orientation.x = 0.0;
    p_target.pose.orientation.y = 0.00;
    p_target.pose.orientation.z = -0.73;
    p_target.pose.orientation.w = 0.68;
    mico.move_to_pose_moveit(p_target);
    mico.move_home();

    pressEnter("Press enter to move to candle pose with Kinova firmware");
    mico.move_to_pose(p_target);
    mico.move_home();

    // Pose with end effector pointing straight left, with palm parallel with the frame of the robot
    pressEnter("Press enter to point left with MoveIt");
    p_target.pose.position.x = 0.00;
    p_target.pose.position.y = 0.40;
    p_target.pose.position.z = 0.50;
    p_target.pose.orientation.x = -0.50;
    p_target.pose.orientation.y = -0.50;
    p_target.pose.orientation.z = -0.50;
    p_target.pose.orientation.w = 0.5;
    mico.move_to_pose_moveit(p_target);
    mico.move_home();

    pressEnter("Press enter to point left with Kinova firmware");
    mico.move_to_pose(p_target);
    mico.move_home();


    // Pose with end effector pointing straight right, with palm parallel with the frame of the robot
    pressEnter("Press enter to point right with MoveIt");
    p_target.pose.position.x = 0.00;
    p_target.pose.position.y = -0.40;
    p_target.pose.position.z = 0.50;
    p_target.pose.orientation.x = 0.00;
    p_target.pose.orientation.y = 0.50;
    p_target.pose.orientation.z = -0.50;
    p_target.pose.orientation.w = 0.5;
    mico.move_to_pose_moveit(p_target);
    mico.move_home();

    pressEnter("Press enter to point right with Kinova firmware");
    mico.move_to_pose(p_target);
    mico.move_home();
    
    ros::shutdown();
}
