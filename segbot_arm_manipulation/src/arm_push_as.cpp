#include <ros/ros.h>


#include <sensor_msgs/JointState.h>
#include <kinova_msgs/PoseVelocity.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/grasp_utils.h>


//actions
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <segbot_arm_manipulation/PushAction.h>


//srv for talking to table_object_detection_node.cpp
#include "bwi_perception/TabletopPerception.h"

#include <moveit_msgs/DisplayRobotState.h>
// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>


#include <segbot_arm_manipulation/MicoManager.h>

#define MIN_DISTANCE_TO_PLANE 0.05

class PushActionServer {
protected:

    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<segbot_arm_manipulation::PushAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    segbot_arm_manipulation::PushFeedback feedback_;
    segbot_arm_manipulation::PushResult result_;

    ros::Publisher debug_pub;
    //used to compute transforms
    tf::TransformListener listener;

    MicoManager mico;

public:

    PushActionServer(std::string name) :
            as_(nh_, name, boost::bind(&PushActionServer::executeCB, this, _1), false),
            action_name_(name), mico(nh_) {


        //advertise the goal pose for debugging
        debug_pub = nh_.advertise<geometry_msgs::PoseStamped>("/m1n6s200_driver/in/debug_pose", 2);

        ROS_INFO("Starting push action server...");

        as_.start();
    }

    ~PushActionServer(void) {
    }


    // Blocking call for user input
    void pressEnter(std::string message) {
        std::cout << message;
        while (true) {
            char c = std::cin.get();
            if (c == '\n')
                break;
            else if (c == 'q') {
                ros::shutdown();
                exit(1);
            } else {
                std::cout << message;
            }
        }
    }

    bool plane_distance(geometry_msgs::Pose start, Eigen::Vector4f plane_c) {
        //filter 1: if too close to the plane
        pcl::PointXYZ p_a;
        p_a.x = start.position.x;
        p_a.y = start.position.y;
        p_a.z = start.position.z;

        return pcl::pointToPlaneDistance(p_a, plane_c);
    }

    moveit_msgs::GetPositionIK::Response computeIK(ros::NodeHandle n, geometry_msgs::PoseStamped p) {
        ros::ServiceClient ikine_client = n.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");

        moveit_msgs::GetPositionIK::Request ikine_request;
        moveit_msgs::GetPositionIK::Response ikine_response;
        ikine_request.ik_request.group_name = "arm";
        ikine_request.ik_request.pose_stamped = p;


        /* Call the service */
        if (ikine_client.call(ikine_request, ikine_response)) {
            ROS_INFO("IK service call success:");
            //ROS_INFO_STREAM(ikine_response);
        } else {
            ROS_INFO("IK service call FAILED. Exiting");
        }

        return ikine_response;
    }

    //create stamped pose from point cloud
    std::vector<geometry_msgs::Pose> generate_poses(sensor_msgs::PointCloud2 pc2) {
        //transform to PCL
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(pc2, pcl_cloud);

        //find center, max, and min of point cloud
        pcl::PointXYZ min;
        pcl::PointXYZ max;
        pcl::getMinMax3D(pcl_cloud, min, max);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(pcl_cloud, centroid);

        std::vector<geometry_msgs::Pose> start_poses;

        //right
        //orientation 1 \/
        geometry_msgs::Pose pose_i;
        pose_i.position.x = centroid(0);
        pose_i.position.y = min.y - 0.03;
        pose_i.position.z = centroid(2);
        pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 3.14, 3.14 / 2);
        start_poses.push_back(pose_i);

        //orientation 2
        pose_i.position.z = centroid(2) - 0.02;
        pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 3.14 / 2, 0);
        start_poses.push_back(pose_i);

        //left
        //orientation 2
        pose_i.position.y = max.y + 0.03;
        start_poses.push_back(pose_i);

        //orientation 1
        pose_i.position.z = centroid(2);
        pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 3.14, 3.14 / 2);
        start_poses.push_back(pose_i);

        //front
        //orientation 1
        pose_i.position.y = centroid(1);
        pose_i.position.x = min.x - 0.05;
        start_poses.push_back(pose_i);

        //orientation 3 <
        pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 3.14 / 2, 3.14 / 2);
        start_poses.push_back(pose_i);

        //orientation 4 >
        pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -3.14 / 2, 3.14 / 2);
        start_poses.push_back(pose_i);

        return start_poses;
    }


    void push(int index) {
        double timeoutSeconds = 3.0;

        kinova_msgs::PoseVelocity velocityMsg;


        if (index == 0 || index == 1) {
            velocityMsg.twist_linear_x = 0.0;
            velocityMsg.twist_linear_y = 0.125;
            velocityMsg.twist_linear_z = 0.0;

            velocityMsg.twist_angular_x = 0.0;
            velocityMsg.twist_angular_y = 0.0;
            velocityMsg.twist_angular_z = 0.0;

            mico.move_with_cartesian_velocities(velocityMsg, timeoutSeconds);
        } else if (index == 2 || index == 3) {
            velocityMsg.twist_linear_x = 0.0;
            velocityMsg.twist_linear_y = -0.125;
            velocityMsg.twist_linear_z = 0.0;

            velocityMsg.twist_angular_x = 0.0;
            velocityMsg.twist_angular_y = 0.0;
            velocityMsg.twist_angular_z = 0.0;

            mico.move_with_cartesian_velocities(velocityMsg, timeoutSeconds);
        } else {
            velocityMsg.twist_linear_x = 0.125;
            velocityMsg.twist_linear_y = 0.0;
            velocityMsg.twist_linear_z = 0.0;

            velocityMsg.twist_angular_x = 0.0;
            velocityMsg.twist_angular_y = 0.0;
            velocityMsg.twist_angular_z = 0.0;

            mico.move_with_cartesian_velocities(velocityMsg, timeoutSeconds);
        }

    }

    void executeCB(const segbot_arm_manipulation::PushGoalConstPtr &goal) {

        if (goal->tgt_cloud.data.size() == 0) {
            ROS_INFO("[push_as.cpp] No object point clouds received...aborting");
            as_.setAborted(result_);
            return;
        }

        if (as_.isPreemptRequested() || !ros::ok()) {
            ROS_INFO("Press action: Preempted");
            // set the action state to preempted
            as_.setPreempted();
            result_.success = false;
            as_.setSucceeded(result_);
            return;
        }

        //get the data out of the goal
        Eigen::Vector4f plane_coef_vector;
        for (int i = 0; i < 4; i++)
            plane_coef_vector(i) = goal->cloud_plane_coef[i];
        ROS_INFO("[push_as.cpp] Received action request...proceeding.");
        mico.wait_for_data();

        mico.close_hand();

        //wait for transform and perform it
        listener.waitForTransform(goal->tgt_cloud.header.frame_id, "m1n6s200_link_base", ros::Time::now(),
                                  ros::Duration(3.0));

        //transform to base link frame of reference
        sensor_msgs::PointCloud2 obj_cloud = goal->tgt_cloud;
        pcl_ros::transformPointCloud("m1n6s200_link_base", obj_cloud, obj_cloud, listener);

        //find possible start poses
        std::vector<geometry_msgs::Pose> app_pos = generate_poses(obj_cloud);

        geometry_msgs::PoseStamped stampedPose;
        stampedPose.header.frame_id = obj_cloud.header.frame_id;
        stampedPose.header.stamp = ros::Time(0);

        //determine which poses can be reached
        int result_i;
        for (int i = 0; i < app_pos.size(); i++) {
            int dist = plane_distance(app_pos.at(i), plane_coef_vector);
            if (dist < MIN_DISTANCE_TO_PLANE) {
                int threshold = MIN_DISTANCE_TO_PLANE - dist;
                app_pos.at(i).position.z += threshold;
            }
            stampedPose.pose = app_pos.at(i);
            result_i = i;
            moveit_msgs::GetPositionIK::Response ik_response = computeIK(nh_, stampedPose);
            if (ik_response.error_code.val == 1) {
                break;
            }
        }

        debug_pub.publish(stampedPose);
        ros::spinOnce();

        //move to pose and push
        mico.move_to_pose_moveit(stampedPose);
        mico.move_to_pose_moveit(stampedPose);
        mico.move_to_pose_moveit(stampedPose);
        push(result_i);

        //move arm out of view for redetection of object
        mico.move_to_side_view();


        //set result of action
        result_.success = true;
        as_.setSucceeded(result_);

    }

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "arm_push_as");

    PushActionServer as(ros::this_node::getName());
    ros::spin();

    return 0;
}
