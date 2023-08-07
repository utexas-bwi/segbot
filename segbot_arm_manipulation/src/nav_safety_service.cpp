#include <vector>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Bool.h>

//services
#include "bwi_moveit_utils/MoveitJointPose.h"

#include "segbot_arm_manipulation/NavSafety.h"
#include <moveit_msgs/GetPositionFK.h>
#include <bwi_msgs/StopBaseStatus.h>
#include <bwi_msgs/StopBase.h>


#include <boost/assign/std/vector.hpp>
#include <tf/transform_listener.h>

#include <std_srvs/SetBool.h>
#include <segbot_arm_manipulation/Mico.h>
#include <segbot_arm_manipulation/arm_utils.h>

using namespace std;

bool debug = true;
bool g_caught_sigint = false;
bool safe = false;

std::vector<float> q_safe;
ros::Publisher pub;
double inflationRad;
std_msgs::Bool pub_data;
sensor_msgs::JointState js_cur;
ros::ServiceClient stopbase_client;

segbot_arm_manipulation::Mico *mico;

bwi_msgs::StopBaseStatus sbs;
bwi_msgs::StopBaseStatus last_status;
bwi_msgs::StopBase::Request sb_req;
bwi_msgs::StopBase::Response sb_resp;

//if disabled, the node publishes "safe" regardless of arm position
bool enabled = true;

void sig_handler(int sig) {
    g_caught_sigint = true;
    ROS_INFO("caugt sigint, init shutdown seq...");
    ros::shutdown();
    exit(1);
};

bool checkIfSafe() {

    if (!enabled) {
        //if disabled, just assume safe
        pub_data.data = true;
        pub.publish(pub_data);
        return true;
    }
    moveit_msgs::GetPositionFK::Response fkine_response = mico->compute_fk();
    /*
     * Check if any joints are outside the radius
     */
    bool temp = true;
    for (auto &i : fkine_response.pose_stamped) {
        auto pose = i.pose;
        if (pose.position.x > inflationRad ||
            pose.position.y > inflationRad ||
            pose.position.x < -inflationRad ||
            pose.position.y < -inflationRad)
            temp = false;
    }
    safe = temp;
    pub_data.data = temp;
    pub.publish(pub_data);
    return temp;

}

void joint_state_cb(const sensor_msgs::JointStateConstPtr &js) {

    if (js->position.size() > 4) { //Message from the base or the arm
        js_cur = *js;

        last_status = sbs;
        if (!checkIfSafe())
            sbs.status = sbs.PAUSED;
        else
            sbs.status = sbs.RUNNING;

        sb_req.status = sbs;
        sb_req.requester = "mico_safety_node";

        if (sbs.status != last_status.status) {
            if (stopbase_client.call(sb_req, sb_resp)) {
                ROS_DEBUG("Made stop_base call");
            } else {
                ROS_ERROR("Stop base service call failed!");
                ros::shutdown();
            }
        }
    }

};

bool service_cb(segbot_arm_manipulation::NavSafety::Request &req, segbot_arm_manipulation::NavSafety::Response &res) {
    ROS_INFO("[mico_nav_safety_service.cpp] making a call to moveit client...");
    sensor_msgs::JointState target_state = segbot_arm_manipulation::values_to_joint_state(q_safe);
    if (mico->move_to_joint_state_moveit(target_state)) {
        ROS_INFO("Safety service call sent. Preparing to move arm to save location.");
        if (req.getSafe) {
            ROS_INFO("enabled checkIfSafe");
            enabled = true;
        }
        res.safe = checkIfSafe();
    } else {
        ROS_ERROR("Safety service call failed. Is the service running?");
        res.safe = false;
        return false;
    }

    return true;
}

bool set_mode_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    enabled = req.data;
    res.success = true;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "nav_safety_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    mico = new segbot_arm_manipulation::Mico(nh);
    ros::Subscriber sub_angles = nh.subscribe(segbot_arm_manipulation::Mico::joint_state_topic, 10, joint_state_cb);
    pub = nh.advertise<std_msgs::Bool>("safe_for_travel", 10);
    ros::ServiceServer srv = nh.advertiseService("make_safe_for_travel", service_cb);

    stopbase_client = nh.serviceClient<bwi_msgs::StopBase>("stop_base");

    //service for enabling and disabling the safety mode
    ros::ServiceServer srv_mode = pnh.advertiseService("set_mode", set_mode_cb);

    q_safe = mico->position_db->get_joint_position("safe");
    nh.param("inflation_radius", inflationRad, .195);
    ros::spin();
}

