#include <boost/algorithm/string/join.hpp>
#include <boost/foreach.hpp>
#include <boost/regex.hpp>
#include <bwi_planning_common/utils.h>
#include <bwi_tools/resource_resolver.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <stdexcept>
#include <tf/transform_datatypes.h>

#include <bwi_tools/point.h>
#include <segbot_simulation_apps/door_handler.h>

namespace segbot_simulation_apps {

  DoorHandler::DoorHandler() {

    ros::NodeHandle nh, private_nh("~");

    std::vector<std::string> unavailable_parameters;
    if (!(private_nh.getParam("obstacle_urdf", obstacle_urdf_))) {
      unavailable_parameters.push_back("obstacle_urdf");
    }
    if (!(private_nh.getParam("door_urdf", door_urdf_))) {
      unavailable_parameters.push_back("door_urdf");
    }

    if (unavailable_parameters.size() != 0) {
      std::string message = "Following neccessary params not available: " +
        boost::algorithm::join(unavailable_parameters, ", ");
      ROS_INFO_STREAM(message);
      throw std::runtime_error(message);
    }

    get_gazebo_model_client_ =
      nh.serviceClient<gazebo_msgs::GetModelState>(
          "/gazebo/get_model_state");
    bool gazebo_available = 
      get_gazebo_model_client_.waitForExistence(ros::Duration(30));

    if (!gazebo_available) {
      ROS_FATAL_STREAM("ClingoGazeboHandler: Gazebo is NOT AVAILABLE");
      throw 
        std::runtime_error("ClingoGazeboHandler: Gazebo is NOT AVAILABLE");
    }

    set_gazebo_model_client_ =
      nh.serviceClient<gazebo_msgs::SetModelState>(
          "/gazebo/set_model_state");
    set_gazebo_model_client_.waitForExistence();

    spawn_model_client_ =
      nh.serviceClient<gazebo_msgs::SpawnModel>(
          "/gazebo/spawn_urdf_model");
    set_gazebo_model_client_.waitForExistence();

    multimap_subscriber_ = nh.subscribe("map_metadata", 1, &DoorHandler::multimapHandler, this);

    initialized_ = false;
  }

  void DoorHandler::initialize() {

    // Spawn all necessary doors
    door_open_status_.resize(doors_.size());
    door_to_true_door_map_.resize(doors_.size());
    for (unsigned i = 0; i < doors_.size(); ++i) {
      // Check if this door is too close to a previously spawned door (in which case they are probably the same door).
      int parent_door = i; // i.e. no parent
      for (unsigned j = 0; j < i; ++j) {
        if (door_to_true_door_map_[j] != -1) {
          // This is a true door. Get its location.
          float diffx = doors_[j].door_center.x - doors_[i].door_center.x;
          float diffy = doors_[j].door_center.y - doors_[i].door_center.y;
          if (sqrtf(diffx*diffx + diffy*diffy) < 0.25f) {
            parent_door = j;
            break;
          }
        }
      }

      door_to_true_door_map_[i] = parent_door;
      if (parent_door == i) {
        // This is a new door. Yay!
        spawnObject(true, i);
        door_open_status_[i] = false;
      }
    }

    // Don't use obstacles for now
    // Spawn 30 obstacle objects
    num_obstacles_ = 0;
    // for (unsigned i = 0; i < 30; ++i) {
    //   spawnObject(false, i);
    // }
    
    initialized_ = true;
  }

  void DoorHandler::multimapHandler(const multi_level_map_msgs::MultiLevelMapData::ConstPtr& multimap) {

    if (!initialized_) {
      doors_.clear();
      // Read in the objects for each level.
      BOOST_FOREACH(const multi_level_map_msgs::LevelMetaData &level, multimap->levels) {
        std::string resolved_data_directory = bwi_tools::resolveRosResource(level.data_directory);
        std::string door_file = bwi_planning_common::getDoorsFileLocationFromDataDirectory(resolved_data_directory);
        std::vector<bwi_planning_common::Door> level_doors;
        readDoorFile(door_file, level_doors);
        doors_.insert(doors_.end(), level_doors.begin(), level_doors.end());
      }

      initialize();
    }
  }

  geometry_msgs::Pose DoorHandler::getDefaultLocation(bool is_door, int index) {
    geometry_msgs::Pose retval;
    retval.position.y = 500.0f + index * 2;
    retval.position.z = 0.0f;
    retval.orientation.x = 0.0f;
    retval.orientation.y = 0.0f;
    retval.orientation.z = 0.0f;
    retval.orientation.w = 1.0f;
    if (is_door) {
      retval.position.x = 500.0f;
    } else {
      retval.position.x = 600.0f;
    }
    return retval;
  }

  float DoorHandler::getDoorWidth(int index) {
    return 0.75f * doors_[index].width;
  }

  geometry_msgs::Pose DoorHandler::getDoorLocation(int index) {
    geometry_msgs::Pose retval;

    retval.position.x = doors_[index].door_center.x;
    retval.position.y = doors_[index].door_center.y;
    retval.position.z = 0;

    bwi::Point2f diff = 
      (doors_[index].door_corners[0] -
       doors_[index].door_corners[1]);
    float door_yaw = atan2f(diff.y, diff.x) + M_PI / 2;
    retval.orientation = tf::createQuaternionMsgFromYaw(door_yaw);

    return retval;
  }

  bool DoorHandler::openDoor(const std::string& door) {
    size_t idx = bwi_planning_common::resolveDoor(door, doors_);
    if (idx == bwi_planning_common::NO_DOOR_IDX) {
      return false;
    }
    return openDoor(idx); 
  }

  bool DoorHandler::openDoor(int index) {
    if (index >= doors_.size()) {
      return false;
    }
    index = door_to_true_door_map_[index];
    if (door_open_status_[index]) 
      return true;
    std::string prefix = "auto_door_";
    std::string model_name = prefix +
      boost::lexical_cast<std::string>(index);
    geometry_msgs::Pose pose = getDefaultLocation(true, index);
    bool success = teleportEntity(model_name, pose);
    door_open_status_[index] = true;
    return success;
  }

  void DoorHandler::openAllDoors() {
    for (unsigned i = 0; i < doors_.size(); ++i) {
      openDoor(i);
    }
  }

  bool DoorHandler::closeDoor(const std::string& door) {
    size_t idx = bwi_planning_common::resolveDoor(door, doors_);
    if (idx == bwi_planning_common::NO_DOOR_IDX) {
      return false;
    }
    return closeDoor(idx); 
  }

  bool DoorHandler::closeDoor(int index) {
    if (index >= doors_.size()) {
      return false;
    }
    index = door_to_true_door_map_[index];
    if (!door_open_status_[index]) 
      return true;
    ROS_INFO_STREAM("Closing door " << index);
    std::string prefix = "auto_door_";
    std::string model_name = prefix +
      boost::lexical_cast<std::string>(index);
    geometry_msgs::Pose pose = getDoorLocation(index);
    bool success = teleportEntity(model_name, pose);
    door_open_status_[index] = false;
    return success;
  }

  void DoorHandler::closeAllDoors() {
    ROS_INFO_STREAM("Closing all doors");
    for (unsigned i = 0; i < doors_.size(); ++i) {
      closeDoor(i);
    }
  }


  bool DoorHandler::isDoorOpen(const std::string& door) {
    size_t idx = bwi_planning_common::resolveDoor(door, doors_);
    if (idx == bwi_planning_common::NO_DOOR_IDX) {
      return false;
    }
    return isDoorOpen(idx); 
  }

  bool DoorHandler::isDoorOpen(int index) {
    if (index >= doors_.size()) {
      return false;
    }
    index = door_to_true_door_map_[index];
    return door_open_status_[index];
  }

  void DoorHandler::closeAllDoorsFarAwayFromPoint(
      const geometry_msgs::Pose& point, float distance) {
    for (unsigned i = 0; i < doors_.size(); ++i) {
      if (door_to_true_door_map_[i] != i) {
        // This isn't a true door. Don't worry about it.
        continue;
      }
      if (!door_open_status_[i]) {
        continue;
      }
      bool is_door_near = checkClosePoses(point, getDoorLocation(i), distance, false);
      if (!is_door_near) {
        closeDoor(i);
      }
    }
  }

  bool DoorHandler::checkClosePoses(const geometry_msgs::Pose& p1, 
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

  bool DoorHandler::teleportEntity(const std::string& entity,
      const geometry_msgs::Pose& pose) {

    int count = 0;
    int attempts = 5;
    bool location_verified = false;
    while (count < attempts and !location_verified) {
      gazebo_msgs::GetModelState get_srv;
      get_srv.request.model_name = entity;
      get_gazebo_model_client_.call(get_srv);
      location_verified = checkClosePoses(get_srv.response.pose, pose);
      if (!location_verified) {
        gazebo_msgs::SetModelState set_srv;
        set_srv.request.model_state.model_name = entity;
        set_srv.request.model_state.pose = pose;
        set_gazebo_model_client_.call(set_srv);
        if (!set_srv.response.success) {
          ROS_WARN_STREAM("SetModelState service call failed for " << entity
              << " to " << pose);
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

  void DoorHandler::spawnObject(bool is_door, int index) {

    gazebo_msgs::SpawnModel spawn;
    std::string prefix;
    if (is_door) {
      prefix = "auto_door_";
      spawn.request.model_xml = boost::regex_replace(door_urdf_, 
          boost::regex("@WIDTH@"),
          boost::lexical_cast<std::string>(getDoorWidth(index)));
      spawn.request.initial_pose = getDoorLocation(index);
    } else {
      prefix = "auto_obs_";
      index = num_obstacles_;
      spawn.request.model_xml = obstacle_urdf_;
      spawn.request.initial_pose = getDefaultLocation(false, index);
    }

    spawn.request.model_name = prefix +
      boost::lexical_cast<std::string>(index);

    if (spawn_model_client_.call(spawn)) {
      if (spawn.response.success) {
        ++num_obstacles_;
        return;
      }
      ROS_WARN_STREAM("Received error message while spawning object: " <<
          spawn.response.status_message);
    }

    ROS_ERROR_STREAM("Unable to spawn: " << spawn.request.model_name);
  }      

}
