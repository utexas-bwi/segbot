#include <ros/ros.h>

#include <bwi_msgs/DoorHandlerInterface.h>
#include <segbot_simulation_apps/door_handler.h>

boost::shared_ptr<segbot_simulation_apps::DoorHandler> gh_;
std::vector<boost::shared_ptr<ros::Timer> > timeout_queue_;

void executeSingleTimeout(const ros::TimerEvent&, 
                          bwi_msgs::DoorHandlerInterface::Request req,
                          boost::shared_ptr<ros::Timer> timer) {
  if (req.all_doors) {
    gh_->closeAllDoors();
  } else {
    /* bool success = */ gh_->closeDoor(req.door);
    // TODO do something with the success result? This was an automated request.
  }
  timeout_queue_.erase(std::remove(timeout_queue_.begin(), timeout_queue_.end(), timer), timeout_queue_.end());
}

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

  if (req.open && req.open_timeout > 0) {
    boost::shared_ptr<ros::Timer> timer(new ros::Timer);
    ros::NodeHandle nh;
    *timer = nh.createTimer(ros::Duration(req.open_timeout), boost::bind(executeSingleTimeout, _1, req, timer), true);
    timeout_queue_.push_back(timer);
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
