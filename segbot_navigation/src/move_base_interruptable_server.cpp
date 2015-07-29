#include <boost/thread/thread.hpp>
#include <bwi_interruptable_action_server/interruptable_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/GetPlan.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

using namespace bwi_interruptable_action_server;

boost::shared_ptr<ros::ServiceClient> clear_costmap_service_;
boost::shared_ptr<ros::ServiceClient> make_plan_service_;
boost::shared_ptr<ros::Subscriber> location_subscriber_;
bool first_location_available_ = false;
geometry_msgs::PoseStamped current_location_;

void locationHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose) {
  current_location_.header = pose->header;
  current_location_.pose = pose->pose.pose;
  first_location_available_ = true;
}

void newGoalCallback(const InterruptableActionServer<move_base_msgs::MoveBaseAction>::GoalConstPtr& new_goal) {
  if (first_location_available_) {
    // TODO also check that our current location estimates are not too far off.
    nav_msgs::GetPlan srv;
    srv.request.start = current_location_;
    srv.request.goal = new_goal->target_pose;
    srv.request.tolerance = 0.2f;
    bool plan_found = false;
    if (make_plan_service_->call(srv)) {
      plan_found = srv.response.plan.poses.size() != 0;
    }
    if (!plan_found) {
      ROS_INFO_STREAM("New goal received, but unable to find plan to goal. Clearing costmaps before sending goal to nav stack.");
      std_srvs::Empty srv;
      if (!clear_costmap_service_->call(srv)) {
        ROS_ERROR_STREAM("Unable to clear costmaps!");
      } else {
        // Sleep for three seconds to allow costmaps to be cleared and reset properly.
        boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
        ROS_INFO_STREAM("Costmaps cleared!");
      }
    }
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "move_base_interruptable_server");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("move_base_interruptable : Waiting for navigation services...");
  clear_costmap_service_.reset(new ros::ServiceClient);
  *clear_costmap_service_ = nh.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
  clear_costmap_service_->waitForExistence();

  make_plan_service_.reset(new ros::ServiceClient);
  *make_plan_service_ = nh.serviceClient<nav_msgs::GetPlan>("move_base/NavfnROS/make_plan");
  make_plan_service_->waitForExistence();

  ROS_INFO_STREAM("move_base_interruptable :   Navigation services found!");

  location_subscriber_.reset(new ros::Subscriber);
  *location_subscriber_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1, &locationHandler);

  InterruptableActionServer<move_base_msgs::MoveBaseAction> as(nh, "move_base", 1, &newGoalCallback);
  as.spin(); // internally calls ros::spinOnce where locationHandler will also get called.
  
  return 0;
}
