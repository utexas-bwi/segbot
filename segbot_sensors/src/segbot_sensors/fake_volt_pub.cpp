#include "ros/ros.h"
#include "std_msgs/String.h"
#include "smart_battery_msgs/SmartBatteryStatus.h"
#include <sstream>

int main(int argc, char **argv){
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<smart_battery_msgs::SmartBatteryStatus>("/battery0", 1);
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()){
    smart_battery_msgs::SmartBatteryStatus stat;
    stat.voltage = 10.9;
    chatter_pub.publish(stat);
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
