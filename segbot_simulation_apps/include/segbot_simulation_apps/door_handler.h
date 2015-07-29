#ifndef SEGBOT_SIM_APPS_DOOR_HANDLER_H
#define SEGBOT_SIM_APPS_DOOR_HANDLER_H

#include <boost/shared_ptr.hpp>
#include <bwi_planning_common/structures.h>
#include <geometry_msgs/Pose.h>
#include <multi_level_map_msgs/MultiLevelMapData.h>
#include <ros/ros.h>

namespace segbot_simulation_apps {

  class DoorHandler {

    public:

      DoorHandler ();

      geometry_msgs::Pose getDefaultLocation(bool is_door, int index);
      float getDoorWidth(int index);
      geometry_msgs::Pose getDoorLocation(int index);

      bool openDoor(const std::string& door);
      bool openDoor(int index);
      void openAllDoors();
      bool closeDoor(const std::string& door);
      bool closeDoor(int index);
      void closeAllDoors();
      bool isDoorOpen(const std::string& door);
      bool isDoorOpen(int index);

      void closeAllDoorsFarAwayFromPoint(
          const geometry_msgs::Pose& point, float distance = 2.0);
      bool checkClosePoses(const geometry_msgs::Pose& p1,
          const geometry_msgs::Pose& p2, float threshold = 0.05,
          bool check_yaw = true);
      bool teleportEntity(const std::string& entity,
          const geometry_msgs::Pose& pose);

      void spawnObject(bool is_door, int index = 0);      

    private:

      bool initialized_;
      void initialize();

      ros::Subscriber multimap_subscriber_;
      void multimapHandler(const multi_level_map_msgs::MultiLevelMapData::ConstPtr& multimap);

      std::vector<bwi_planning_common::Door> doors_;
      std::vector<bool> door_open_status_;
      std::vector<int> door_to_true_door_map_;

      std::set<int> obstacles_in_use;
      std::set<int> unused_obstacles_;
      unsigned int num_obstacles_; // obstacles + doors

      ros::ServiceClient get_gazebo_model_client_;
      ros::ServiceClient set_gazebo_model_client_;
      ros::ServiceClient spawn_model_client_;

      std::string obstacle_urdf_;
      std::string door_urdf_;
  };
}

#endif /* end of include guard: SEGBOT_SIM_APPS_DOOR_HANDLER_H */
