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

void log_joint_diff(const kinova_msgs::JointAngles &target, const kinova_msgs::JointAngles &actual) {
    std::vector<double> delta = segbot_arm_manipulation::getJointAngleDifferences(target, actual);
    cout << "Target: " << target.joint1 << " Actual: " << actual.joint1 << " Delta: " << delta.at(0) << endl;
    cout << "Target: " << target.joint2 << " Actual: " << actual.joint2 << " Delta: " << delta.at(1) << endl;
    cout << "Target: " << target.joint3 << " Actual: " << actual.joint3 << " Delta: " << delta.at(2) << endl;
    cout << "Target: " << target.joint4 << " Actual: " << actual.joint4 << " Delta: " << delta.at(3) << endl;
    cout << "Target: " << target.joint5 << " Actual: " << actual.joint5 << " Delta: " << delta.at(4) << endl;
    cout << "Target: " << target.joint6 << " Actual: " << actual.joint6 << " Delta: " << delta.at(5) << endl;
}


int main(int argc, char **argv) {
    // Intialize ROS with this node name
    ros::init(argc, argv, "ex8_joint_angles_pose_control");
    
    ros::NodeHandle n;

    //register ctrl-c
    signal(SIGINT, sig_handler);

    bool success = false;
    MicoManager mico(n);

    kinova_msgs::JointAngles angles;
    angles.joint1 = 3;
    angles.joint2 = 3;
    angles.joint3 = 3;
    angles.joint4 = .2;
    angles.joint5 = 6.28;
    angles.joint6 = 0;

    pressEnter("Press enter to move to candle pose with MoveIt");
    success = mico.move_to_joint_state_moveit(angles);
    ROS_INFO("Movement status: %s", success ? "true": "false");
    mico.wait_for_data();
    kinova_msgs::JointAngles result_angles = segbot_arm_manipulation::state_to_angles(mico.current_state);
    log_joint_diff(angles, result_angles);
    mico.move_home();

    pressEnter("Press enter to move to candle pose with Kinova firmware");
    success = mico.move_to_joint_state(angles);
    mico.wait_for_data();
    result_angles = segbot_arm_manipulation::state_to_angles(mico.current_state);
    log_joint_diff(angles, result_angles);
    ROS_INFO("Movement status: %s", success ? "true": "false");
    mico.move_home();
    
    ros::shutdown();
}
