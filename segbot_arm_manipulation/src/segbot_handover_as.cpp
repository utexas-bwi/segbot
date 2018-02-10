#include <ros/ros.h>

#include <segbot_arm_manipulation/MicoManager.h>
#include <segbot_arm_manipulation/grasp_utils.h>
#include <segbot_arm_manipulation/arm_utils.h>

//actions
#include <actionlib/server/simple_action_server.h>


//srv for talking to table_object_detection_node.cpp
#include "bwi_perception/TabletopPerception.h"

#include <moveit_msgs/DisplayRobotState.h>


//the action definition
#include "segbot_arm_manipulation/HandoverAction.h"

//tf stuff
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>


//used to decide if someone is pulling an object from the arm
#define FORCE_HANDOVER_THRESHOLD 3.0


/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


class HandoverActionServer {
protected:

    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<segbot_arm_manipulation::HandoverAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    segbot_arm_manipulation::HandoverFeedback feedback_;
    segbot_arm_manipulation::HandoverResult result_;

    //subscribers -- in an action server, these have to be class members
    MicoManager mico;

public:

    explicit HandoverActionServer(const std::string &name) :
            as_(nh_, name, boost::bind(&HandoverActionServer::executeCB, this, _1), false),
            action_name_(name),
            mico(nh_) {

        ROS_INFO("Starting grasp action server...");

        as_.start();
    }

    ~HandoverActionServer() = default;


    void executeCB(const segbot_arm_manipulation::HandoverGoalConstPtr &goal) {

        if (goal->type == segbot_arm_manipulation::HandoverGoal::RECEIVE) {
            //open fingers
            mico.open_hand();

            //update readings
            mico.wait_for_data();

            bool threshold_met = mico.wait_for_force(FORCE_HANDOVER_THRESHOLD, goal->timeout_seconds);

            if (threshold_met) {
                mico.close_hand();
                result_.success = true;
                as_.setSucceeded(result_);
            } else {
                result_.success = false;
                as_.setAborted(result_);
            }
        } else if (goal->type == segbot_arm_manipulation::HandoverGoal::GIVE) {

            //TO DO: move to handover position

            ROS_INFO("Starting handover action...");

            //listen for haptic feedback

            mico.wait_for_data();
            sensor_msgs::JointState prev_effort_state = mico.current_state;


            double elapsed_time = 0;

            bool threshold_met = mico.wait_for_force(FORCE_HANDOVER_THRESHOLD, goal->timeout_seconds);
            if (threshold_met) {
                //now open the hand
                mico.open_hand();

                result_.success = true;
                as_.setSucceeded(result_);
                return;
            } else {

                result_.success = false;
                as_.setAborted(result_);
            }
        }
    }

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "segbot_handover_as");

    HandoverActionServer as(ros::this_node::getName());
    ros::spin();

    return 0;
}