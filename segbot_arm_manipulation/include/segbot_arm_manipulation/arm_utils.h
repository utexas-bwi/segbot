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

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>
#include <moveit_utils/MicoNavSafety.h>


#include <sensor_msgs/PointCloud2.h>

#include "segbot_arm_perception/SetObstacles.h"
#include "segbot_arm_perception/TabletopPerception.h"

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



    sensor_msgs::JointState valuesToJointState(std::vector<float> joint_values) {
        sensor_msgs::JointState js;

        for (unsigned int i = 0; i < 8; i++) {
            js.name.push_back(jointNames[i]);

            if (i < 6) {
                js.position.push_back(joint_values.at(i));
            } else
                js.position.push_back(0.0);

            js.velocity.push_back(0.0);
            js.effort.push_back(0.0);
        }

        return js;
    }

    std::vector<double> getJointAngleDifferences(sensor_msgs::JointState A, sensor_msgs::JointState B) {
        std::vector<double> result;

        for (unsigned int i = 0; i < A.position.size(); i++) {
            //check if this is a m1n6s200 arm joint or not
            bool is_arm_joint = false;
            for (int k = 0; k < NUM_JOINTS; k++) {
                if (A.name[i] == jointNames[k]) {
                    is_arm_joint = true;
                    break;
                }
            }

            if (is_arm_joint) {

                if (A.name[i] == "m1n6s200_joint_2" || A.name[i] == "m1n6s200_joint_3")
                    result.push_back(fabs(A.position[i] - B.position[i]));
                else {
                    if (B.position[i] > A.position[i]) {
                        if (B.position[i] - A.position[i] < A.position[i] + 2 * PI - B.position[i])
                            result.push_back(fabs(B.position[i] - A.position[i]));
                        else
                            result.push_back(fabs(2 * PI - B.position[i] + A.position[i]));
                    } else {
                        if (A.position[i] - B.position[i] > B.position[i] + 2 * PI - A.position[i])
                            result.push_back(fabs(B.position[i] + 2 * PI - A.position[i]));
                        else
                            result.push_back(fabs(A.position[i] - B.position[i]));
                    }

                }


            }
        }

        return result;
    }

    bool makeSafeForTravel(ros::NodeHandle n) {
        ros::ServiceClient safety_client = n.serviceClient<moveit_utils::MicoNavSafety>("/mico_nav_safety");
        safety_client.waitForExistence();
        moveit_utils::MicoNavSafety srv_safety;
        srv_safety.request.getSafe = true;

        if (safety_client.call(srv_safety)) {

            return srv_safety.response.safe;
        } else {
            ROS_ERROR("Failed to call safety service....aborting");
            return false;
        }
    }

    void homeArm(ros::NodeHandle n) {
        ros::ServiceClient home_client = n.serviceClient<kinova_msgs::HomeArm>("/m1n6s200_driver/in/home_arm");

        kinova_msgs::HomeArm srv;
        if (home_client.call(srv))
            ROS_INFO("Homing arm");
        else
            ROS_INFO("Cannot contact homing service. Is it running?");
    }

    segbot_arm_perception::TabletopPerception::Response getTabletopScene(ros::NodeHandle n) {

        ros::ServiceClient client_tabletop_perception = n.serviceClient<segbot_arm_perception::TabletopPerception>(
                "tabletop_object_detection_service");

        segbot_arm_perception::TabletopPerception srv;
        if (client_tabletop_perception.call(srv)) {
            return srv.response;
        } else {
            ROS_ERROR("Failed to call service tabletop_object_detection_service");
            return srv.response;
        }
    }

    moveit_msgs::GetPositionIK::Response computeIK(ros::NodeHandle n, geometry_msgs::PoseStamped p) {

        ros::ServiceClient ikine_client = n.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");


        moveit_msgs::GetPositionIK::Request ikine_request;
        moveit_msgs::GetPositionIK::Response ikine_response;
        ikine_request.ik_request.group_name = "arm";
        ikine_request.ik_request.pose_stamped = p;

        /* Call the service */
        if (ikine_client.call(ikine_request, ikine_response)) {
            return ikine_response;
        } else {
            ROS_ERROR("IK service call FAILED. Exiting");
            return ikine_response;
        }
    };

    bool setArmObstacles(ros::NodeHandle n, std::vector<sensor_msgs::PointCloud2> clouds) {
        ros::ServiceClient client_set_obstalces = n.serviceClient<segbot_arm_perception::SetObstacles>(
                "segbot_arm_perception/set_obstacles");

        segbot_arm_perception::SetObstacles srv_obstacles;
        for (unsigned int i = 0; i < clouds.size(); i++) {
            srv_obstacles.request.clouds.push_back(clouds.at(i));
        }

        if (client_set_obstalces.call(srv_obstacles)) {
            ROS_INFO("[demo_obstacle_avoidance.cpp] Obstacles set");
            return true;
        } else {
            ROS_ERROR("Failed to call service segbot_arm_perception/set_obstacles");
            return false;
        }
    }

    moveit_utils::MicoMoveitCartesianPose::Response
    moveToPoseMoveIt(ros::NodeHandle n, geometry_msgs::PoseStamped p_target) {
        moveit_utils::MicoMoveitCartesianPose::Request req;
        moveit_utils::MicoMoveitCartesianPose::Response res;

        req.target = p_target;

        ros::ServiceClient client = n.serviceClient<moveit_utils::MicoMoveitCartesianPose>(
                "/mico_cartesianpose_service");
        if (client.call(req, res)) {
            ROS_INFO("MoveToPoseMoveIt Call successful. Response:");
        } else {
            ROS_ERROR("MoveToPoseMoveIt Call failed. Terminating.");
        }

        return res;
    }

    // put the boxes around the collision objects
    std::vector<moveit_msgs::CollisionObject>
    get_collision_boxes(std::vector<sensor_msgs::PointCloud2> obstacles) {
        //wait for transform and perform it

        std::vector<moveit_msgs::CollisionObject> collision_objects;
        for (int i = 0; i < obstacles.size(); i++) {

            // transform
            tf::TransformListener tf_listener;
            tf_listener.waitForTransform(obstacles[i].header.frame_id, "base_link",
                                         ros::Time(0), ros::Duration(3.0));

            sensor_msgs::PointCloud2 object_cloud;
            pcl_ros::transformPointCloud("base_link", obstacles[i], object_cloud, tf_listener);

            // convert to PCL
            PointCloudT::Ptr object_i(new PointCloudT);
            pcl::PCLPointCloud2 pc_i;
            pcl_conversions::toPCL(object_cloud, pc_i);
            pcl::fromPCLPointCloud2(pc_i, *object_i);

            // get the min and max
            PointT min_pt;
            PointT max_pt;

            //3D Min/Max
            pcl::getMinMax3D(*object_i, min_pt, max_pt);
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*object_i, centroid);

            // create a bounding box
            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = "base_link";

            //Id of object used to identify it
            collision_object.id = "box";

            //Define a box to add to the world
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);

            primitive.dimensions[0] = max_pt.x - min_pt.x;
            primitive.dimensions[1] = max_pt.y - min_pt.y;
            primitive.dimensions[2] = max_pt.z - min_pt.z;

            geometry_msgs::Pose box_pose;
            box_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
            box_pose.position.x = centroid[0];
            box_pose.position.y = centroid[1];
            box_pose.position.z = centroid[2];

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;
            collision_objects.push_back(collision_object);

        }

        return collision_objects;
    }


    void moveToJointState(ros::NodeHandle n, sensor_msgs::JointState target) {
        //check if this is specified just for the arm
        sensor_msgs::JointState q_target;
        if (target.position.size() > NUM_JOINTS) {
            //in this case, the first four values are for the base joints
            for (int i = 4; i < target.position.size(); i++) {
                q_target.position.push_back(target.position.at(i));
                q_target.name.push_back(target.name.at(i));
            }
            q_target.header = target.header;
        } else
            q_target = target;

        /*ROS_INFO("Target joint state:");
        ROS_INFO_STREAM(q_target);
        pressEnter();*/

        moveit_utils::AngularVelCtrl::Request req;
        moveit_utils::AngularVelCtrl::Response resp;

        ros::ServiceClient ikine_client = n.serviceClient<moveit_utils::AngularVelCtrl>("/angular_vel_control");

        req.state = q_target;


        if (ikine_client.call(req, resp)) {
            ROS_INFO("Call successful. Response:");
            ROS_INFO_STREAM(resp);
        } else {
            ROS_ERROR("Call failed. Terminating.");
            //ros::shutdown();
        }

    }

    void moveToPoseJaco(geometry_msgs::PoseStamped g) {
        actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ac(pose_action_topic, true);

        kinova_msgs::ArmPoseGoal goalPose;
        goalPose.pose = g;

        ac.waitForServer();

        //finally, send goal and wait
        ac.sendGoal(goalPose);
        ac.waitForResult();
    }

    void moveFingers(int finger_value1, int finger_value2) {
        actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> ac(finger_action_topic, true);

        kinova_msgs::SetFingersPositionGoal goalFinger;
        goalFinger.fingers.finger1 = finger_value1;
        goalFinger.fingers.finger2 = finger_value2;
        // Not used for our arm
        goalFinger.fingers.finger3 = 0;

        ac.waitForServer();
        ac.sendGoal(goalFinger);
        ac.waitForResult();
    }

    void moveFingers(int finger_value) {
        actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> ac(finger_action_topic, true);

        kinova_msgs::SetFingersPositionGoal goalFinger;
        goalFinger.fingers.finger1 = finger_value;
        goalFinger.fingers.finger2 = finger_value;
        // Not used for our arm
        goalFinger.fingers.finger3 = 0;

        ac.waitForServer();
        ac.sendGoal(goalFinger);
        ac.waitForResult();
    }

    void openHand() {
        moveFingers(OPEN_FINGER_VALUE);
    }

    void closeHand() {
        moveFingers(CLOSED_FINGER_VALUE);
    }

    void arm_side_view(ros::NodeHandle n) {
        std::string j_pos_filename =
                ros::package::getPath("segbot_arm_manipulation") + "/data/jointspace_position_db.txt";
        std::string c_pos_filename =
                ros::package::getPath("segbot_arm_manipulation") + "/data/toolspace_position_db.txt";

        ArmPositionDB *positionDB;
        positionDB = new ArmPositionDB(j_pos_filename, c_pos_filename);
        positionDB->print();

        if (positionDB->hasCarteseanPosition("side_view")) {
            ROS_INFO("Moving arm to side view...");
            geometry_msgs::PoseStamped out_of_view_pose = positionDB->getToolPositionStamped("side_view",
                                                                                             "/m1n6s200_link_base");
            segbot_arm_manipulation::moveToPoseMoveIt(n, out_of_view_pose);
        } else {
            ROS_ERROR("[arm_utils] Cannot move arm to side view!");
        }
    }

    void arm_handover_view(ros::NodeHandle n) {
        std::string j_pos_filename =
                ros::package::getPath("segbot_arm_manipulation") + "/data/jointspace_position_db.txt";
        std::string c_pos_filename =
                ros::package::getPath("segbot_arm_manipulation") + "/data/toolspace_position_db.txt";

        ArmPositionDB *positionDB;
        positionDB = new ArmPositionDB(j_pos_filename, c_pos_filename);
        positionDB->print();

        if (positionDB->hasCarteseanPosition("handover_front")) {
            geometry_msgs::PoseStamped handover_pose = positionDB->getToolPositionStamped("handover_front",
                                                                                          "m1n6s200_link_base");

            ROS_INFO("Moving to handover position");

            segbot_arm_manipulation::moveToPoseMoveIt(n, handover_pose);
        } else {
            ROS_ERROR("[arm_utils] cannot move to the handover position!");
        }

    }
}
#endif
