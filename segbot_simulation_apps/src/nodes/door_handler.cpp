#include <ros/ros.h>

#include <bwi_msgs/DoorHandlerInterface.h>
#include <segbot_simulation_apps/door_handler.h>

boost::shared_ptr<segbot_simulation_apps::DoorHandler> gh_;

bool execute(bwi_msgs::DoorHandlerInterface::Request  &req,
             bwi_msgs::DoorHandlerInterface::Response &res) {

  res.status = "";
  if (req.all_doors) {
    if (req.open) {
      gh_->openAllDoors();
    } else {
      gh_->closeAllDoors();
    }
    res.success = true;
  } else {
    if (req.open) {
      res.success = gh_->openDoor(req.door);
    } else {
      res.success = gh_->closeDoor(req.door);
    }
    if (!res.success) {
      res.status = "Unable to resolved " + req.door + "!";
    }
  }

  return true;
}

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "gazebo_door_handler");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("update_doors", execute);
  gh_.reset(new segbot_simulation_apps::DoorHandler);
  ros::spin();
  return 0;
}
