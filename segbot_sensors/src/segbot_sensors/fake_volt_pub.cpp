#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/BatteryState.h"
#include <sstream>

int main(int argc, char **argv){
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<sensor_msgs::BatteryState>("/battery0", 1);
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()){
    sensor_msgs::BatteryState stat;
    stat.voltage = 10.9;
    chatter_pub.publish(stat);
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
