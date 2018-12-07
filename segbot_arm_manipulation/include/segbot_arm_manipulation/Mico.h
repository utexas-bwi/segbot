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
#include <moveit_msgs/GetPositionIK.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <kinova_msgs/PoseVelocity.h>
#include <geometry_msgs/WrenchStamped.h>
#include <bwi_manipulation/arm.h>
#include <moveit_msgs/GetPositionFK.h>
#include <bwi_manipulation/ArmPositionDB.h>
#include "Mico.h"


namespace segbot_arm_manipulation {

    class Mico : bwi_manipulation::Arm {
        ros::Subscriber joint_state_sub;
        ros::Subscriber tool_sub;
        ros::Subscriber finger_sub;
        ros::Subscriber wrench_sub;


    public:
        actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> pose_action;
        actionlib::SimpleActionClient<kinova_msgs::ArmJointAnglesAction> joint_state_action;
        actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> fingers_action;
        ros::ServiceClient home_client;
        ros::ServiceClient safety_client;
        ros::ServiceClient pose_moveit_client;
        ros::ServiceClient joint_angles_moveit_client;
        ros::ServiceClient waypoint_moveit_client;
        ros::ServiceClient joint_pose_client_old;
        ros::ServiceClient ik_client;
        ros::ServiceClient fk_client;
        ros::ServiceClient add_waypoint_client;
        ros::ServiceClient clear_waypoints_client;
        ros::Publisher angular_velocity_pub;
        ros::Publisher cartesian_velocity_pub;
        sensor_msgs::JointState current_state;
        geometry_msgs::PoseStamped current_pose;
        kinova_msgs::FingerPosition current_finger;
        geometry_msgs::WrenchStamped current_wrench;
        std::unique_ptr<bwi_manipulation::ArmPositionDB> position_db;

        bool heard_joint_state;
        bool heard_tool;
        bool heard_fingers;
        bool heard_wrench;

        static const std::string jointNames[];
        static const std::string finger_action_topic;
        static const std::string pose_action_topic;
        static const std::string joint_state_action_topic;
        static const std::string joint_state_topic;
        static const std::string tool_pose_topic;
        static const std::string finger_position_topic;
        static const std::string home_arm_service;

        static const std::string j_pos_filename;
        static const std::string c_pos_filename;
        static const double arm_poll_rate;
        static const std::string side_view_position_name;
        static const std::string side_view_approach_position_name;
        static const std::string handover_position_name;

        static const uint OPEN_FINGER_VALUE;
        static const uint CLOSED_FINGER_VALUE;

        explicit Mico(ros::NodeHandle n);

        ~Mico() = default;

        void joint_state_cb(const sensor_msgs::JointStateConstPtr &msg);

        void toolpose_cb(const geometry_msgs::PoseStampedConstPtr &msg);

        void fingers_cb(const kinova_msgs::FingerPositionConstPtr &msg);

        void wrench_cb(const geometry_msgs::WrenchStampedConstPtr &msg);

        bool wait_for_data(double timeout = -1.0);

        bool wait_for_force(double force_threshold, double timeout = -1.0);

        bool move_to_joint_state(const sensor_msgs::JointState &target);

        bool move_to_joint_state(const kinova_msgs::JointAngles &target);

        bool move_to_pose(const geometry_msgs::PoseStamped &pose);

        bool move_through_waypoints(const std::vector<geometry_msgs::Pose> &waypoints);

        void move_with_angular_velocities(const kinova_msgs::JointAngles &velocities, const double duration);

        void move_with_cartesian_velocities(const kinova_msgs::PoseVelocity &velocities, const double duration);

        bool move_fingers(int finger_value);

        bool move_fingers(int finger1_value, int finger2_value);

        bool move_to_joint_state_moveit(const sensor_msgs::JointState &target,
                                        const std::vector<sensor_msgs::PointCloud2> &obstacles = std::vector<sensor_msgs::PointCloud2>(),
                                        const moveit_msgs::Constraints &constraints = moveit_msgs::Constraints());

        bool
        move_to_joint_state_moveit(const kinova_msgs::JointAngles &target,
                                   const std::vector<sensor_msgs::PointCloud2> &obstacles = std::vector<sensor_msgs::PointCloud2>(),
                                   const moveit_msgs::Constraints &constraints = moveit_msgs::Constraints());

        bool move_through_waypoints_moveit(const std::vector<geometry_msgs::Pose> &waypoints,
                                           const std::vector<sensor_msgs::PointCloud2> &obstacles = std::vector<sensor_msgs::PointCloud2>(),
                                           const moveit_msgs::Constraints &constraints = moveit_msgs::Constraints());

        bool move_to_pose_moveit(const geometry_msgs::PoseStamped &target,
                                 const std::vector<sensor_msgs::PointCloud2> &obstacles = std::vector<sensor_msgs::PointCloud2>(),
                                 const moveit_msgs::Constraints &constraints = moveit_msgs::Constraints());

        moveit_msgs::GetPositionIK::Response compute_ik(const geometry_msgs::PoseStamped &p);

        moveit_msgs::GetPositionFK::Response compute_fk();

        bool open_hand();

        bool close_hand();

        bool make_safe_for_travel();

        bool move_home();

        bool move_to_named_joint_position(const std::string &name);

        bool move_to_named_tool_position(const std::string &name);

        bool move_to_side_view();

        bool move_to_side_view_approach();

        bool move_to_handover();

    };
}
#endif //SEGBOT_ARM_MANIPULATION_MICOMANAGER_H
