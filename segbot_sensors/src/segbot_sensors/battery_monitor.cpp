#include "ros/ros.h"
#include "ros/time.h"
#include <vector>
#include "std_msgs/Header.h"
#include "std_msgs/Float32.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/KeyValue.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "battery_estimator");
  ros::NodeHandle n;

  ros::Publisher battery_life_pub = n.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);

  ros::Rate loop_rate(10);
  diagnostic_msgs::DiagnosticStatus status;
  diagnostic_msgs::DiagnosticArray diagAr;
  status.level = 0;
  status.name = "battery_estimator";
  status.hardware_id = "005";
  diagnostic_msgs::KeyValue val;
  val.key = "bat_mon";
  val.value = "183.0";
  val.key = "battery";
  while (ros::ok())
  {
    diagAr.header.stamp = ros::Time::now();
    diagAr.header.frame_id = 1;
    float voltage = 12.3;
    float estimated_life = 183.0;
    if(voltage > 10.5)
	status.message = "Battery in good health";

    status.values.push_back( val);
    diagAr.status.push_back(status);
    battery_life_pub.publish(diagAr);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
