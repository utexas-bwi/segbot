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

  ros::Rate loop_rate(1);
  diagnostic_msgs::DiagnosticStatus status;
  diagnostic_msgs::DiagnosticArray diagAr;
  diagnostic_msgs::KeyValue val;
  status.hardware_id = "005";
  val.key = "battery_value";
  val.value = "183.0";

  while (ros::ok())
  {
    diagAr.header.stamp = ros::Time::now();
    diagAr.header.frame_id = 1;
    float voltage = 12.3;
    float estimated_life = 183.0;
    if(voltage > 11.0){
	status.message = "Battery in good health";
	status.level = 0; // 0 = OK
    }else if(voltage > 10 && voltage < 11){
	status.message = "Battery low, return to lab.";
	status.level = 1; // WARN
    }else{
	status.message = "Battery CRITICALLY low, or voltmeter data is inaccurate (or missing). Ensure the volt sensor is connected properly and its publisher relaying data.";
	status.level = 0; // CRITICAL
    }
    status.values.push_back(val);
    diagAr.status.push_back(status);
    battery_life_pub.publish(diagAr);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
