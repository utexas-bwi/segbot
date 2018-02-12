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

void log_pose_diff(const geometry_msgs::PoseStamped &target, const geometry_msgs::PoseStamped &actual) {
    double angle_diff = segbot_arm_manipulation::quat_angular_difference(target.pose.orientation,
                                                                         actual.pose.orientation);
    cout << "Orientation X Target: " << target.pose.orientation.x << " Actual: " << actual.pose.orientation.x << endl;
    cout << "Orientation Y Target: " << target.pose.orientation.y << " Actual: " << actual.pose.orientation.y << endl;
    cout << "Orientation Z Target: " << target.pose.orientation.z << " Actual: " << actual.pose.orientation.z << endl;
    cout << "Orientation W Target: " << target.pose.orientation.w << " Actual: " << actual.pose.orientation.w << endl;
    cout << "Orientation Delta: " << angle_diff << endl;

    cout << "Position X Target: " << target.pose.position.x << " Actual: " << actual.pose.position.x << " Delta: "
         << fabs(target.pose.position.x - actual.pose.position.x) << endl;
    cout << "Position Y Target: " << target.pose.position.y << " Actual: " << actual.pose.position.y << " Delta: "
         << fabs(target.pose.position.y - actual.pose.position.y) << endl;
    cout << "Position Z Target: " << target.pose.position.z << " Actual: " << actual.pose.position.z << " Delta: "
         << fabs(target.pose.position.z - actual.pose.position.z) << endl;
}


int main(int argc, char **argv) {
    // Intialize ROS with this node name
    ros::init(argc, argv, "ex7_cartesian_pose_control");
    
    ros::NodeHandle n;
    //register ctrl-c
    signal(SIGINT, sig_handler);
        
    segbot_arm_manipulation::Mico mico(n);

    pressEnter("Press enter to move to candle pose with MoveIt");

    geometry_msgs::PoseStamped p_target;
    // Pose with the end effector pointing straight up, with palm parallel with the frame of the robot
    p_target.header.frame_id = "m1n6s200_link_base";
    p_target.pose.position.x = 0.00;
    p_target.pose.position.y = -0.05;
    p_target.pose.position.z = 0.97;
    tf::Quaternion quat;
    quat.setX(0.00);
    quat.setY(0.00);
    quat.setZ(-0.73);
    quat.setW(0.68);
    quat.normalize();
    tf::quaternionTFToMsg(quat, p_target.pose.orientation);
    mico.move_to_pose_moveit(p_target);
    mico.wait_for_data();
    log_pose_diff(p_target, mico.current_pose);
    mico.move_home();

    pressEnter("Press enter to move to candle pose with Kinova firmware");
    mico.move_to_pose(p_target);
    mico.move_home();

    // Pose with end effector pointing straight left, with palm parallel with the frame of the robot
    pressEnter("Press enter to point left with MoveIt");
    p_target.pose.position.x = 0.00;
    p_target.pose.position.y = 0.40;
    p_target.pose.position.z = 0.50;
    quat.setX(-0.50);
    quat.setY(-0.50);
    quat.setZ(-0.50);
    quat.setW(0.50);
    quat.normalize();
    tf::quaternionTFToMsg(quat, p_target.pose.orientation);
    mico.move_to_pose_moveit(p_target);
    mico.wait_for_data();
    log_pose_diff(p_target, mico.current_pose);
    mico.move_home();

    pressEnter("Press enter to point left with Kinova firmware");
    mico.move_to_pose(p_target);
    mico.move_home();


    // Pose with end effector pointing straight right, with palm parallel with the frame of the robot
    pressEnter("Press enter to point right with MoveIt");
    p_target.pose.position.x = 0.00;
    p_target.pose.position.y = -0.40;
    p_target.pose.position.z = 0.50;
    quat.setX(0.70);
    quat.setY(0.00);
    quat.setZ(0.00);
    quat.setW(0.70);
    quat.normalize();
    tf::quaternionTFToMsg(quat, p_target.pose.orientation);
    mico.move_to_pose_moveit(p_target);
    mico.wait_for_data();
    log_pose_diff(p_target, mico.current_pose);
    mico.move_home();

    pressEnter("Press enter to point right with Kinova firmware");
    mico.move_to_pose(p_target);
    mico.move_home();
    
    ros::shutdown();
}
