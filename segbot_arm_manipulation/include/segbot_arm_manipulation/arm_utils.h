#ifndef ARM_UTILS_H
#define ARM_UTILS_H

#include <ros/ros.h>


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


    sensor_msgs::JointState values_to_joint_state(std::vector<float> joint_values);

    std::vector<double> getJointAngleDifferences(sensor_msgs::JointState A, sensor_msgs::JointState B);

    std::vector<double> getJointAngleDifferences(kinova_msgs::JointAngles A, kinova_msgs::JointAngles B);

    // put the boxes around the collision objects
    std::vector<moveit_msgs::CollisionObject>
    get_collision_boxes(std::vector<sensor_msgs::PointCloud2> obstacles);

    kinova_msgs::JointAngles state_to_angles(const sensor_msgs::JointState &state);


    double quat_angular_difference(const geometry_msgs::Quaternion &a, const geometry_msgs::Quaternion &b);
}
#endif
