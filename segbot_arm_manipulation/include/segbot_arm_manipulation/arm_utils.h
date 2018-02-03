#ifndef ARM_UTILS_H
#define ARM_UTILS_H

#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>

//actions
#include <actionlib/client/simple_action_client.h>
#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"
#include "kinova_msgs/ArmJointAnglesAction.h"
#include "kinova_msgs/HomeArm.h"

#include <moveit_msgs/GetPositionIK.h>

#include <bwi_moveit_utils/AngularVelCtrl.h>
#include <bwi_moveit_utils/MicoMoveitJointPose.h>
#include <bwi_moveit_utils/MicoMoveitCartesianPose.h>
#include <bwi_moveit_utils/MicoNavSafety.h>


#include <sensor_msgs/PointCloud2.h>

#include "bwi_perception/SetObstacles.h"

#include <moveit_msgs/CollisionObject.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/common.h>

#include "arm_positions_db.h"


#define OPEN_FINGER_VALUE 100
#define CLOSED_FINGER_VALUE 7200
#define NUM_JOINTS 8


#define PI 3.14159265

using namespace std;

const string jointNames[] = {"m1n6s200_joint_1", "m1n6s200_joint_2", "m1n6s200_joint_3", "m1n6s200_joint_4",
                       "m1n6s200_joint_5", "m1n6s200_joint_6", "m1n6s200_joint_finger_1",
                       "m1n6s200_joint_finger_2"};
const string finger_action_topic = "/m1n6s200_driver/fingers_action/finger_positions";
const string pose_action_topic = "/m1n6s200_driver/pose_action/tool_pose";
const string joint_state_action_topic = "/m1n6s200_driver/joints_action/joint_angles";
const string joint_state_topic = "/m1n6s200_driver/out/joint_state";
const string tool_pose_topic = "/m1n6s200_driver/out/tool_pose";
const string finger_position_topic = "/m1n6s200_driver/out/finger_position";
const string home_arm_service = "/m1n6s200_driver/in/home_arm";


namespace segbot_arm_manipulation {
    using namespace pcl;
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;


    sensor_msgs::JointState valuesToJointState(std::vector<float> joint_values);

    std::vector<double> getJointAngleDifferences(sensor_msgs::JointState A, sensor_msgs::JointState B);

    bool makeSafeForTravel(ros::NodeHandle n);

    void homeArm(ros::NodeHandle n);

    bool setArmObstacles(ros::NodeHandle n, std::vector<sensor_msgs::PointCloud2> clouds);

    bwi_moveit_utils::MicoMoveitCartesianPose::Response
    moveToPoseMoveIt(ros::NodeHandle n, geometry_msgs::PoseStamped p_target);

    // put the boxes around the collision objects
    std::vector<moveit_msgs::CollisionObject>
    get_collision_boxes(std::vector<sensor_msgs::PointCloud2> obstacles);


    void moveToJointState(ros::NodeHandle n, sensor_msgs::JointState target);

    void moveToPoseJaco(geometry_msgs::PoseStamped g);

    void openHand();

    void closeHand();

    void arm_side_view(ros::NodeHandle n);

    void arm_handover_view(ros::NodeHandle n);

    moveit_msgs::GetPositionIK::Response computeIK(ros::NodeHandle n, geometry_msgs::PoseStamped p);
}
#endif
