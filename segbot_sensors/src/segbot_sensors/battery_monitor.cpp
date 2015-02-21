#include "ros/ros.h"
#include <vector>
#include "std_msgs/Float32.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/KeyValue.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "battery_life_estimator");
  ros::NodeHandle n;

  ros::Publisher battery_life_pub = n.advertise<diagnostic_msgs::DiagnosticStatus>("/battery_estimator", 10);

  ros::Rate loop_rate(10);
  diagnostic_msgs::DiagnosticStatus status;
  status.level = 0;
  status.name = "battery_estimator";
  status.hardware_id = "005";
  int count = 0;
  diagnostic_msgs::KeyValue val;
  std::vector<diagnostic_msgs::KeyValue> kvals;
  val.key = "battery";
  while (ros::ok())
  {
    float voltage = 12.3;
    float estimated_life = 183.0;
    if(voltage > 10.5)
	status.message = "Battery in good health";
    val.value = estimated_life;
    kvals.push_back(val);
    status.values = kvals;

    battery_life_pub.publish(status);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
