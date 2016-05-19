#include <boost/shared_ptr.hpp>
#include <stdexcept>

#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <ros/ros.h>

#include <bwi_msgs/RobotTeleporterInterface.h>
#include <segbot_simulation_apps/common.h>

std::string robot_model_name_;
boost::shared_ptr<ros::ServiceClient> get_gazebo_model_client_;
boost::shared_ptr<ros::ServiceClient> set_gazebo_model_client_;

bool execute(bwi_msgs::RobotTeleporterInterface::Request &req,
             bwi_msgs::RobotTeleporterInterface::Response &res) {

  res.success = segbot_simulation_apps::teleportEntity(robot_model_name_,
                                                       req.pose,
                                                       *get_gazebo_model_client_,
                                                       *set_gazebo_model_client_);
  return true;
}

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "gazebo_robot_teleporter");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  if (!private_nh.getParam("robotid", robot_model_name_)) {
    ROS_FATAL_STREAM("GazeboRobotTeleporter: Parameter ~robotid needs to be set to indicate the robot entity name " <<
                     "inside Gazebo.");
    return -1;
  }

  get_gazebo_model_client_.reset(new ros::ServiceClient);
  *get_gazebo_model_client_ = nh.serviceClient<gazebo_msgs::GetModelState>( "/gazebo/get_model_state");
  bool gazebo_available = get_gazebo_model_client_->waitForExistence(ros::Duration(30));

  if (!gazebo_available) {
    ROS_FATAL_STREAM("GazeboRobotTeleporter: Gazebo is NOT AVAILABLE");
    return -1;
  }

  set_gazebo_model_client_.reset(new ros::ServiceClient);
  *set_gazebo_model_client_ = nh.serviceClient<gazebo_msgs::SetModelState>( "/gazebo/set_model_state");
  set_gazebo_model_client_->waitForExistence();

  ros::ServiceServer service = nh.advertiseService("teleport_robot", execute);
  ros::spin();

  return 0;
}
