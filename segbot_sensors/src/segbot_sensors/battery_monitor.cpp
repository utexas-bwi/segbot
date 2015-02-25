#include "ros/ros.h"
#include "ros/time.h"
#include <vector>
#include "std_msgs/Header.h"
#include "std_msgs/Float32.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/KeyValue.h"
#include "smart_battery_msgs/SmartBatteryStatus.h" //file found
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

void voltageCallback(const smart_battery_msgs::SmartBatteryStatus::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->voltage);
  //todo: add voltage profiler logic - write profile rates in /config
}

int sendmail()
{
    char msg[] = "Robot battery low. Please seek robot and bring to lab for charging.";
    int retval = -1;
    FILE *mailpipe = popen("/usr/lib/sendmail -t", "w");
    if (mailpipe != NULL) {
        fprintf(mailpipe, "To: maxsvetlik@gmail.com\n");
        fprintf(mailpipe, "From: bwipowerdameon@bwi.edu\n");
        fprintf(mailpipe, "Subject: There's an issue with the robot\n");
        fwrite(msg, 1, strlen(msg), mailpipe);
        fwrite(".\n", 1, 2, mailpipe);
        pclose(mailpipe);
        retval = 0;
     }
     else {
         perror("Failed to invoke sendmail");
     }
     return retval;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "battery_estimator");
  ros::NodeHandle n;
  int mail = system("/usr/lib/sendmail -t < ~/mailmessage.txt &");
  std::cout << "Mail status: " << mail << std::endl;

  ros::Publisher battery_life_pub = n.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
  ros::Subscriber voltage_sub = n.subscribe("smart_battery_msgs/SmartBatteryStatus", 10, voltageCallback);

  ros::Rate loop_rate(1); //1hz
  diagnostic_msgs::DiagnosticStatus status;
  diagnostic_msgs::DiagnosticArray diagAr;
  diagnostic_msgs::KeyValue val;
  status.hardware_id = "005";
  status.name = "battery_estimator"; //must match what the aggregator config file expects
  val.key = "battery_value";
  val.value = "183.0";

  while (ros::ok())
  {
    diagAr.header.stamp = ros::Time::now();
    diagAr.header.frame_id = 1;
    float voltage = 12.3;
//    float estimated_life = 183.0;
    if(voltage > 11.0){
		status.message = "Battery in good health";
		status.level = 0; // 0 = OK
    }
    else if(voltage > 10 && voltage < 11){
		status.message = "Battery low, return to lab.";
		status.level = 1; // WARN
    }
    else{
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
