#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>

#include <segbot_simulation_apps/common.h>

namespace segbot_simulation_apps {
  
  bool checkClosePoses(const geometry_msgs::Pose& p1, 
                       const geometry_msgs::Pose& p2, 
                       float threshold, 
                       bool check_yaw) {

    float dist_diff = sqrtf(pow((p1.position.x - p2.position.x), 2) + pow((p1.position.y - p2.position.y), 2));
    if (dist_diff > threshold) {
      return false;
    }
    double yaw1 = tf::getYaw(p1.orientation);
    double yaw2 = tf::getYaw(p2.orientation);
    if (check_yaw && fabs(yaw1 - yaw2) > 0.1) {
      return false;
    }

    return true;
  }

  bool teleportEntity(const std::string& entity,
                      const geometry_msgs::Pose& pose,
                      ros::ServiceClient& get_gazebo_model_client,
                      ros::ServiceClient& set_gazebo_model_client) {

    int count = 0;
    int attempts = 5;
    bool location_verified = false;
    while (count < attempts and !location_verified) {
      gazebo_msgs::GetModelState get_srv;
      get_srv.request.model_name = entity;
      get_gazebo_model_client.call(get_srv);
      location_verified = checkClosePoses(get_srv.response.pose, pose);
      if (!location_verified) {
        gazebo_msgs::SetModelState set_srv;
        set_srv.request.model_state.model_name = entity;
        set_srv.request.model_state.pose = pose;
        set_gazebo_model_client.call(set_srv);
        if (!set_srv.response.success) {
          ROS_WARN_STREAM("SetModelState service call failed for " << entity << " to " << pose);
        }
      }
      ++count;
    }

    if (!location_verified) {
      ROS_ERROR_STREAM("Unable to teleport " << entity << " to " << pose
                       << " despite " << attempts << " attempts.");
      return false;
    }

    return true;
  }

} /* segbot_simulation_apps */
