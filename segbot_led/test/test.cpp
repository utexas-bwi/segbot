/*******************************************************
*                    ROS Headers                       *
********************************************************/
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

/*******************************************************
*                   Service Headers                    *
********************************************************/
#include "bwi_msgs/LEDClear.h"
#include "bwi_msgs/LEDSetCamera.h"
#include "bwi_msgs/LEDTestStrip.h"

/*******************************************************
*                   Action Headers                     *
********************************************************/
#include "bwi_msgs/LEDControlAction.h"

/*******************************************************
*                   Message Headers                    *
********************************************************/
#include "bwi_msgs/LEDActionResult.h"
#include "bwi_msgs/LEDAnimations.h"
#include "bwi_msgs/LEDCameraStatus.h"
#include "bwi_msgs/LEDTestType.h"

using namespace std;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_ledcom_server");

  ros::NodeHandle privateNode("~");

  int type;
  privateNode.param<int>("type",type,2);

  int timeout;
  privateNode.param<int>("timeout",timeout,10);

  actionlib::SimpleActionClient<bwi_msgs::LEDControlAction> ac("led_control_server", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

  ROS_INFO_STREAM("Action server started, sending goal of type " << type << " and with a timeout of " << timeout << " seconds.");
  bwi_msgs::LEDControlGoal goal;


  switch(type){

    case 1: { 
              goal.type.led_animations = bwi_msgs::LEDAnimations::LEFT_TURN;
              break;
            }
    case 2: { 
              goal.type.led_animations = bwi_msgs::LEDAnimations::RIGHT_TURN;
              break;
            }
    case 3: { 
              goal.type.led_animations = bwi_msgs::LEDAnimations::REVERSE;
              break;
            }
    case 4: { 
              goal.type.led_animations = bwi_msgs::LEDAnimations::BLOCKED;
              break;
            }
    case 5: { 
              goal.type.led_animations = bwi_msgs::LEDAnimations::UP;
              break;
            }
    case 6: { 
              goal.type.led_animations = bwi_msgs::LEDAnimations::DOWN;
              break;
            }
    case 7: { 
              goal.type.led_animations = bwi_msgs::LEDAnimations::NEED_ASSIST;
              break;
            }
  }

  goal.timeout = ros::Duration(timeout);
  ac.sendGoal(goal);

  ROS_INFO("Action received goal: %s",ac.getState().toString().c_str());

  // Prvents values from being cached
  privateNode.deleteParam("type");
  privateNode.deleteParam("timeout");

  return 0;
}
