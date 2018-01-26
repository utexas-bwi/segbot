#ifndef SEGBOT_ARM_MANIPULATION_MICOMANAGER_H
#define SEGBOT_ARM_MANIPULATION_MICOMANAGER_H

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinova_msgs/FingerPosition.h>
#include <ros/node_handle.h>
#include <kinova_msgs/ArmPoseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <kinova_msgs/ArmJointAnglesAction.h>
#include <kinova_msgs/HomeArm.h>
#include <moveit_utils/MicoNavSafety.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <kinova_msgs/PoseVelocity.h>
#include "MicoManager.h"

#define OPEN_FINGER_VALUE 100
#define CLOSED_FINGER_VALUE 7200

class ArmPositionDB;

const std::string moveit_cartesian_pose_service = "";

const std::string j_pos_filename = ros::package::getPath("segbot_arm_manipulation")+"/data/jointspace_position_db.txt";
const std::string c_pos_filename = ros::package::getPath("segbot_arm_manipulation")+"/data/toolspace_position_db.txt";

class MicoManager {
    ros::Subscriber joint_state_sub;
    ros::Subscriber tool_sub;
    ros::Subscriber finger_sub;
    ArmPositionDB *position_db;

public:
    actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> pose_action;
    actionlib::SimpleActionClient<kinova_msgs::ArmJointAnglesAction> joint_state_action;
    actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> fingers_action;
    ros::ServiceClient home_client;
    ros::ServiceClient safety_client;
    ros::ServiceClient pose_moveit_client;
    ros::ServiceClient joint_angles_moveit_client;
    ros::ServiceClient joint_pose_client_old;
    ros::ServiceClient ik_client;
    ros::ServiceClient add_waypoint_client;
    ros::ServiceClient clear_waypoints_client;
    ros::Publisher angular_velocity_pub;
    ros::Publisher cartesian_velocity_pub;
    sensor_msgs::JointState current_state;
    geometry_msgs::PoseStamped current_pose;
    kinova_msgs::FingerPosition current_finger;

    moveit::planning_interface::MoveGroupInterface *group;

    bool heardJointState;
    bool heardTool;
    bool heardFingers;

    MicoManager(ros::NodeHandle n);

    ~MicoManager();

    void joint_state_cb(const sensor_msgs::JointStateConstPtr &msg);

    void toolpose_cb(const geometry_msgs::PoseStampedConstPtr &msg);

    void fingers_cb(const kinova_msgs::FingerPositionConstPtr &msg);

    void wait_for_data();

    bool move_to_joint_state(const sensor_msgs::JointState &target);

    bool move_to_joint_state(const kinova_msgs::JointAngles &target);

    bool move_to_pose(const geometry_msgs::PoseStamped &pose);

    bool move_through_waypoints(const std::vector<geometry_msgs::Pose> &waypoints);

    void move_with_angular_velocities(const kinova_msgs::JointAngles &velocities, const double duration);

    void move_with_cartesian_velocities(const kinova_msgs::PoseVelocity &velocities, const double duration);

    bool move_fingers(int finger_value);

    bool move_fingers(int finger1_value, int finger2_value);

    bool move_to_joint_state_moveit(const sensor_msgs::JointState &target, const std::vector<sensor_msgs::PointCloud2> &obstacles = std::vector<sensor_msgs::PointCloud2>(), const moveit_msgs::Constraints &constraints = moveit_msgs::Constraints());

    bool
    move_to_joint_state_moveit(const kinova_msgs::JointAngles &target, const std::vector<sensor_msgs::PointCloud2> &obstacles = std::vector<sensor_msgs::PointCloud2>(), const moveit_msgs::Constraints &constraints = moveit_msgs::Constraints());

    bool move_through_waypoints_moveit(const std::vector<geometry_msgs::Pose> &waypoints,
                                       const std::vector<sensor_msgs::PointCloud2> &obstacles = std::vector<sensor_msgs::PointCloud2>(),
                                       const moveit_msgs::Constraints &constraints = moveit_msgs::Constraints());

    bool move_to_pose_moveit(const geometry_msgs::PoseStamped &target,
                             const std::vector<sensor_msgs::PointCloud2> &obstacles = std::vector<sensor_msgs::PointCloud2>(), const moveit_msgs::Constraints &constraints = moveit_msgs::Constraints());

    moveit_msgs::GetPositionIK::Response compute_ik(const geometry_msgs::PoseStamped &p);



    bool open_hand();

    bool close_hand();

    bool make_safe_for_travel();

    bool move_home();

    bool move_to_side_view();

    bool move_to_handover();

    bool move_to_joint_state_old(const sensor_msgs::JointState &target);

};

#endif //SEGBOT_ARM_MANIPULATION_MICOMANAGER_H
