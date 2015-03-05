#include "ros/ros.h"
#include "ros/time.h"
#include <vector>
#include "std_msgs/Header.h"
#include "std_msgs/Float32.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/KeyValue.h"
#include "smart_battery_msgs/SmartBatteryStatus.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

float voltage;
bool sentMail = false;
void voltageCallback(const smart_battery_msgs::SmartBatteryStatus::ConstPtr& msg){
  voltage = (msg->voltage);
  //todo: add voltage profiler logic - write profile rates in /config
}

int sendMail(std::string outboundList, std::string sender){
  char msg[] = "Hello,\nJust so you know, there is a robot whose battery is low. Please seek this robot and bring it to the lab for charging.\nBest,\nBuilding Wide Intelligence\n\nPS: Please don't email me!";
  int retval = -1;
  FILE *mailpipe = popen("/usr/lib/sendmail -t", "w");
  if(mailpipe != NULL){
    std::string to = "To: " + outboundList + "\n";
    std::string from = "From: " + sender + "\n";
    fputs(to.c_str(), mailpipe);
    fputs(from.c_str(), mailpipe);
    fputs("Subject: Segbot Low Battery Alert\n", mailpipe);
    fwrite(msg, 1, strlen(msg), mailpipe);
    fwrite(".\n", 1, 2, mailpipe);
    pclose(mailpipe);
    retval = 0;
  }else {
    perror("Failed to invoke sendmail");
  }
  return retval;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "battery_estimator");
  ros::NodeHandle n;
  ros::Publisher battery_life_pub = n.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
  ros::Subscriber voltage_sub = n.subscribe("/battery0", 10, voltageCallback);
  ros::Rate loop_rate(1); //1hz
  diagnostic_msgs::DiagnosticStatus voltages;
  diagnostic_msgs::DiagnosticStatus status;
  diagnostic_msgs::DiagnosticArray diagAr;
  diagnostic_msgs::KeyValue status_val;
  diagnostic_msgs::KeyValue voltages_val;
  voltages.hardware_id = "005";
  voltages.name = "voltage";
  voltages_val.value = voltage;
  voltages_val.key = "Current voltage: ";
  status.hardware_id = "005";
  status.name = "battery_estimator"; //must match what the aggregator config file expects
  status_val.key = "battery_value";
  status_val.value = "183.0"; //dummy value
  std::ostringstream ss;

  while (ros::ok()){
    voltages_val.value = boost::lexical_cast<std::string>(voltage);
    diagAr.header.stamp = ros::Time::now();
    diagAr.header.frame_id = 1;

    if(voltage > 11.0){
      status.message = "Battery in good health";
      status.level = 0; // 0 = OK
      voltages.level = 0;
      voltages.message = "Voltage level OK";
    }else if(voltage > 10 && voltage < 11){
      status.message = "Battery low, return to lab.";
      status.level = 1; // WARN
      voltages.level = 1;
      voltages.message = "Voltage dropping";
      if(!sentMail){
        //To maintain steady diag readings, run sendmail on thread
        bool paramSuccess;
        std::string outbound;
        paramSuccess = n.getParam("email_alert_outbound", outbound);
        std::string send;
        paramSuccess &= n.getParam("email_alert_sender", send);
        if(!paramSuccess){
          std::cout << "Error reading send mail parameters. Check launchfile. Emails not sent." << std::endl;
        }else{
          boost::thread mailThread(sendMail, outbound, send);
        }
      }
      sentMail = true;
    }else if(!voltage){
      voltages.message = "No voltage data";
      voltages.level = 2;
      status.message = "Cannot extrapolate battery charge. No voltage data.";
      status.level = 2;
    }else{
      status.message = "Battery CRITICALLY low, or voltmeter data is inaccurate (or missing). Ensure the volt sensor is connected properly and its publisher relaying data.";
      status.level = 2; // CRITICAL
      voltages.level = 2;
      voltages.message = "ERROR: Ensure accurate volt readings!";
    }
    //Clearing messages reduces memory overhead, but limits the perception of changes in state in the diagnostics viewer.
    status.values.clear();
    status.values.push_back(status_val);
    voltages.values.clear();
    voltages.values.push_back(voltages_val);
    diagAr.status.clear();
    diagAr.status.push_back(status);
    diagAr.status.push_back(voltages);
    battery_life_pub.publish(diagAr);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
