#ifndef SEGBOT_SIM_APPS_COMMON_H
#define SEGBOT_SIM_APPS_COMMON_H

#include <geometry_msgs/Pose.h>

namespace segbot_simulation_apps {
  
  bool checkClosePoses(const geometry_msgs::Pose& p1,
                       const geometry_msgs::Pose& p2, 
                       float threshold = 0.05,
                       bool check_yaw = true);

  bool teleportEntity(const std::string& entity,
                      const geometry_msgs::Pose& pose,
                      ros::ServiceClient& get_gazebo_model_client,
                      ros::ServiceClient& set_gazebo_model_client);

} /* segbot_simulation_apps */


#endif /* end of include guard: SEGBOT_SIM_APPS_COMMON_H */
