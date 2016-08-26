/*******************************************************
*                    C Headers                         *
********************************************************/
#include <unistd.h>
#include <signal.h>

/*******************************************************
*                    ROS Headers                       *
********************************************************/
#include <actionlib/server/simple_action_server.h>

/*******************************************************
*                   segbot_led Headers                    *
********************************************************/
#include "ledcom.h"

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

/*******************************************************
*                 Global Variables                     *
********************************************************/
LedCOM leds;
int led_count;
string serial_port;
bool camera_on = false;
bool connected = false;

// LED Segments
int front_center_left_start;
int front_center_left_end;

int front_center_right_start;
int front_center_right_end;

int back_center_left_start;
int back_center_left_end;

int back_center_right_start;
int back_center_right_end;

int front_left_beam_start;
int front_left_beam_end;

int front_right_beam_start;
int front_right_beam_end ;

int back_left_beam_start;
int back_left_beam_end;

int back_right_beam_start;
int back_right_beam_end;

int camera_indicator_start;
int camera_indicator_end;

int back_center_start;
int back_center_end;

int circular_u_start;
int circular_u_end;

int top_of_u_start;
int top_of_u_end;

/*******************************************************
*                                                      *
*                 Helper Functions                     *
*                                                      *
********************************************************/
void check_camera_status()
{
  if(camera_on)
  {
    leds.clear();
    // Microseconds
    usleep(100000);

    for (int i = camera_indicator_start; i <= camera_indicator_end; i++)
    {
      leds.setHSV(i, 240, 1, .1);
    }

    leds.flush();
    // Microseconds
    usleep(100000);

    ROS_INFO("Set camera indicator on strip");
  }
}

void connect(string port, int baud)
{
  while(!connected)
  {
    try
    {
        leds.connect(port, baud);
        connected = true;
        // Need to sleep to wait for connection to take full effect
        sleep(3);
    }
    catch(const serial::IOException &e)
    {
        ROS_ERROR("EXCEPTION CAUGHT: serial::IOException could not open a connection.");
        ROS_ERROR_STREAM("Original exception: " << e.what());
        ROS_ERROR_STREAM("Ensure device is connected and using port, " << port << ", with baud setting, " << baud << ".");
        ROS_ERROR("Retrying to open connection after waiting 2 seconds.");
        sleep(1);
    }
  }

  try
  {
    leds.setLEDCount(led_count);
    // Ensures strip is clear on initial start up
    leds.clear();
    sleep(1);
  }
  catch(const serial::SerialException &e)
  {
    connected = false;
  }

  ROS_INFO("Ready to control LED strip with %d leds.", led_count);

}

void ledSigintHandler(int sig)
{
  camera_on = false;
  ros::shutdown();
}

/*******************************************************
*                                                      *
*                   Action Server                      *
*                                                      *
********************************************************/
class LEDControlAction
{
protected:

  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<bwi_msgs::LEDControlAction> as_;
  std::string action_name_;

  bwi_msgs::LEDControlFeedback feedback_;
  bwi_msgs::LEDControlResult result_;

public:

  ros::Timer timeout_timer;

  bool timeout = false;
  bool success = true;

  void timerCallback(const ros::TimerEvent& event)
  {
    timeout = true;
    success = true;
    ROS_INFO("Goal has reached timeout.");
  }

  LEDControlAction(std::string name) :
    as_(nh_, name, boost::bind(&LEDControlAction::executeCB, this, _1), false),
    action_name_(name)
  {
    timeout_timer = nh_.createTimer(ros::Duration(100), boost::bind(&LEDControlAction::timerCallback, this, _1));
    timeout_timer.stop();
    as_.start();
  }

  ~LEDControlAction(void)
  {
  }

  void executeCB(const bwi_msgs::LEDControlGoalConstPtr &goal)
  {
    ROS_INFO_STREAM(action_name_ << " : Executing LED Action: " << goal->type);

    leds.clear();
    // Microseconds
    usleep(100000);

    timeout_timer.stop();

    ros::Time start = ros::Time::now();

    if (goal->timeout > ros::Duration(0))
    {
      timeout = false;
      ROS_INFO_STREAM("Creating timeout for: " << goal->timeout);
      timeout_timer.setPeriod(goal->timeout);
      timeout_timer.start();
    }

    try
    {
      // Will run as long as goal is active
      while(as_.isActive())
      {
        // Preempted Execution Logic
        if (as_.isPreemptRequested())
        {
          ROS_INFO("%s: Preempted", action_name_.c_str());
          result_.result = bwi_msgs::LEDActionResult::PREEMPTED;
          std::ostringstream stringStream;
          stringStream << "Action Preempted. " << goal->type;
          result_.status = stringStream.str();
          as_.setPreempted(result_);
          timeout_timer.stop();
          leds.clear();
          ROS_INFO("Cleared LED Strip");
          success = false;
          timeout = false;
          check_camera_status();
          break;
        }

        // ROS Failure Logic
        if (!ros::ok())
        {
          ROS_INFO("%s: Preempted due to ROS failure", action_name_.c_str());
          result_.result = bwi_msgs::LEDActionResult::SHUTDOWN;
          std::ostringstream stringStream;
          stringStream << "Action terminated due to ROS Shutdown. " << goal->type;
          result_.status = stringStream.str();
          as_.setPreempted(result_);
          timeout_timer.stop();
          leds.clear();
          ROS_INFO("Cleared LED Strip");
          check_camera_status();
          success = false;
          timeout = false;
          break;
        }

        // Determines which animation to execute based on goal->type value
        switch(goal->type.led_animations)
        {
          // Left Turn Sequential Animation
/*          case bwi_msgs::LEDAnimations::LEFT_TURN:
            {
              // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK
              while(!as_.isPreemptRequested() && !timeout && ros::ok())
              {
                // Creates an animation of leds which travels left along the strip

                ros::Duration time_running = ros::Time::now() - start;
                feedback_.time_running = time_running;
                as_.publishFeedback(feedback_);

                int j = back_center_left_start+1;

                for (int i = front_center_left_start-1; i < front_center_left_end+1;)
                {
                  // Terminate goal if preempted, timeout is reached, or ros fails
                  if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                  if (i == front_center_left_start-1)
                  {
                    leds.setHSV(i, 22, 1, .1);
                    leds.setHSV(i+1, 22, 1, .1);

                    leds.setHSV(j, 22, 1, .1);
                    leds.setHSV(j-1, 22, 1, .1);

                    i+=2;
                    j-=2;
                  }
                  else
                  {
                    leds.setHSV(i, 22, 1, .1);
                    leds.setHSV(j, 22, 1, .1);
                    i+=1;
                    j-=1;
                  }

                  if (i > front_center_left_start-1)
                  {
                    leds.setRGB(i-2, 0, 0, 0);
                    leds.setRGB(j+2, 0, 0, 0);
                  }

                  leds.flush();
                  // Microseconds
                  usleep(100000);

                  if (i == front_center_left_end+1)
                  {
                    leds.setRGB(i, 0, 0, 0);
                    leds.setRGB(i-1, 0, 0, 0);

                    leds.setRGB(j, 0, 0, 0);
                    leds.setRGB(j+1, 0, 0, 0);

                    i+=1;
                    j-=1;

                    leds.flush();
                    // Microseconds
                    usleep(100000);
                  }
                }
              }
              break;
            }

          // Right Turn Sequential Animation
          case bwi_msgs::LEDAnimations::RIGHT_TURN:
            {
              // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK
              while(!as_.isPreemptRequested() && !timeout && ros::ok())
              {
                // Creates an animation of leds which travels right along the strip

                ros::Duration time_running = ros::Time::now() - start;
                feedback_.time_running = time_running;
                as_.publishFeedback(feedback_);

                int j = front_center_right_start+1;

                for (int i = back_center_right_start-1; i < back_center_right_end+1;)
                {
                  // Terminate goal if preempted, timeout is reached, or ros fails
                  if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                  if (i == back_center_right_start-1)
                  {
                    leds.setHSV(i, 22, 1, .1);
                    leds.setHSV(i+1, 22, 1, .1);

                    leds.setHSV(j, 22, 1, .1);
                    leds.setHSV(j-1, 22, 1, .1);

                    i+=2;
                    j-=2;
                  }
                  else
                  {
                    leds.setHSV(i, 22, 1, .1);
                    leds.setHSV(j, 22, 1, .1);
                    i+=1;
                    j-=1;
                  }

                  if (i > back_center_right_start-1)
                  {
                    leds.setRGB(i-2, 0, 0, 0);
                    leds.setRGB(j+2, 0, 0, 0);
                  }

                  leds.flush();
                  // Microseconds
                  usleep(100000);

                  if (i == back_center_right_end+1)
                  {
                    leds.setRGB(i, 0, 0, 0);
                    leds.setRGB(i-1, 0, 0, 0);

                    leds.setRGB(j, 0, 0, 0);
                    leds.setRGB(j+1, 0, 0, 0);

                    i+=1;
                    j-=1;

                    leds.flush();
                    // Microseconds
                    usleep(100000);
                  }
                }
              }
              break;
            }*/

          // Blinking Versions of Turn Signals

/*          // Left Turn Blinking Animation
          case bwi_msgs::LEDAnimations::LEFT_TURN:
            {
              // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK
              while(!as_.isPreemptRequested() && !timeout && ros::ok())
              {
                // Creates an animation of leds which travels left along the strip

                ros::Duration time_running = ros::Time::now() - start;
                feedback_.time_running = time_running;
                as_.publishFeedback(feedback_);

                int j = back_center_left_start;

                for (int i = front_center_left_start; i <= front_center_left_end;)
                {
                  // Terminate goal if preempted, timeout is reached, or ros fails
                  if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                  leds.setHSV(i, 22, 1, .1);
                  leds.setHSV(j, 22, 1, .1);

                  i++;
                  j--;
                }

                leds.flush();
                // Microseconds
                usleep(400000);

                leds.clear();
                // Microseconds
                usleep(100000);
              }
              break;
            }

          // Right Turn Blinking Animation
          case bwi_msgs::LEDAnimations::RIGHT_TURN:
            {
              // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK
              while(!as_.isPreemptRequested() && !timeout && ros::ok())
              {
                // Creates an animation of leds which travels right along the strip

                ros::Duration time_running = ros::Time::now() - start;
                feedback_.time_running = time_running;
                as_.publishFeedback(feedback_);

                int j = front_center_right_start;

                for (int i = back_center_right_start; i <= back_center_right_end;)
                {
                  // Terminate goal if preempted, timeout is reached, or ros fails
                  if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                  leds.setHSV(i, 22, 1, .1);
                  leds.setHSV(j, 22, 1, .1);

                  i++;
                  j--;
                }

                leds.flush();
                // Microseconds
                usleep(400000);

                leds.clear();
                // Microseconds
                usleep(100000);
              }
              break;
            }*/

          // Circular Versions of Turn Signals [One Set]

/*          // Left Turn Circular Animation [One Set]
          case bwi_msgs::LEDAnimations::LEFT_TURN:
            {
              // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK
              while(!as_.isPreemptRequested() && !timeout && ros::ok())
              {
                // Creates an animation of leds which travels left along the strip

                ros::Duration time_running = ros::Time::now() - start;
                feedback_.time_running = time_running;
                as_.publishFeedback(feedback_);

                int k = circular_u_end+(1+(top_of_u_end-top_of_u_start));
                int l = top_of_u_start-1;
                int m = top_of_u_end+1;

                for (int i = circular_u_start-1; i < k+3;)
                {
                  // Terminate goal if preempted, timeout is reached, or ros fails
                  if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                  if(i < circular_u_end+1)
                  {
                    if (i == circular_u_start-1)
                    {
                      leds.setHSV(i, 22, 1, .1);
                      leds.setHSV(i+1, 22, 1, .1);
                      leds.setHSV(i+2, 22, 1, .1);
                      leds.setHSV(i+3, 22, 1, .1);

                      i+=4;
                    }
                    else
                    {
                      leds.setHSV(i, 22, 1, .1);
                      i+=1;
                    }

                    if (i > circular_u_start-1)
                    {
                      leds.setRGB(i-4, 0, 0, 0);
                    }

                    leds.flush();
                    // Microseconds
                    usleep(100000);
                  }
                  else
                  {
                    if (i == circular_u_end+1)
                    {
                      leds.setRGB(i, 0, 0, 0);
                      leds.setRGB(i-1, 0, 0, 0);
                      leds.setRGB(i-2, 0, 0, 0);
                      leds.setRGB(i-3, 0, 0, 0);

                      i+=1;

                      leds.flush();
                      // Microseconds
                      usleep(100000);
                    }

                    if (i == circular_u_end+2)
                    {
                      leds.setHSV(l, 22, 1, .1);
                      leds.setHSV(l+1, 22, 1, .1);
                      leds.setHSV(l+2, 22, 1, .1);
                      leds.setHSV(l+3, 22, 1, .1);

                      i+=4;
                      l+=4;
                    }
                    else
                    {
                      leds.setHSV(l, 22, 1, .1);
                      i+=1;
                      l+=1;
                    }

                    if (i > circular_u_end+2)
                    {
                      leds.setRGB(l-4, 0, 0, 0);
                    }

                    leds.flush();
                    // Microseconds
                    usleep(100000);

                    if (i == k+3)
                    {
                      leds.setRGB(l, 0, 0, 0);
                      leds.setRGB(l-1, 0, 0, 0);
                      leds.setRGB(l-2, 0, 0, 0);
                      leds.setRGB(l-3, 0, 0, 0);

                      i+=1;
                      l+=1;

                      leds.flush();
                      // Microseconds
                      usleep(100000);
                    }
                  }
                }
              }
              break;
            }

          // Right Turn Circular Animation [One Set]
          case bwi_msgs::LEDAnimations::RIGHT_TURN:
            {
              // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK
              while(!as_.isPreemptRequested() && !timeout && ros::ok())
              {
                // Creates an animation of leds which travels left along the strip

                ros::Duration time_running = ros::Time::now() - start;
                feedback_.time_running = time_running;
                as_.publishFeedback(feedback_);

                int k = circular_u_start-(1+(top_of_u_end-top_of_u_start));
                int l = top_of_u_start-1;
                int m = top_of_u_end+1;

                for (int i = circular_u_end+1; i > k-3;)
                {
                  // Terminate goal if preempted, timeout is reached, or ros fails
                  if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                  if(i > circular_u_start-1)
                  {
                    if (i == circular_u_end+1)
                    {
                      leds.setHSV(i, 22, 1, .1);
                      leds.setHSV(i-1, 22, 1, .1);
                      leds.setHSV(i-2, 22, 1, .1);
                      leds.setHSV(i-3, 22, 1, .1);

                      i-=4;
                    }
                    else
                    {
                      leds.setHSV(i, 22, 1, .1);
                      i-=1;
                    }

                    if (i < circular_u_end+1)
                    {
                      leds.setRGB(i+4, 0, 0, 0);
                    }

                    leds.flush();
                    // Microseconds
                    usleep(100000);
                  }
                  else
                  {
                    if (i == circular_u_start-1)
                    {
                      leds.setRGB(i, 0, 0, 0);
                      leds.setRGB(i+1, 0, 0, 0);
                      leds.setRGB(i+2, 0, 0, 0);
                      leds.setRGB(i+3, 0, 0, 0);

                      i-=1;

                      leds.flush();
                      // Microseconds
                      usleep(100000);
                    }

                    if (i == circular_u_start-2)
                    {
                      leds.setHSV(m, 22, 1, .1);
                      leds.setHSV(m-1, 22, 1, .1);
                      leds.setHSV(m-2, 22, 1, .1);
                      leds.setHSV(m-3, 22, 1, .1);

                      i-=4;
                      m-=4;
                    }
                    else
                    {
                      leds.setHSV(m, 22, 1, .1);
                      i-=1;
                      m-=1;
                    }

                    if (i < circular_u_start-2)
                    {
                      leds.setRGB(m+4, 0, 0, 0);
                    }

                    leds.flush();
                    // Microseconds
                    usleep(100000);

                    if (i == k-3)
                    {
                      leds.setRGB(m, 0, 0, 0);
                      leds.setRGB(m+1, 0, 0, 0);
                      leds.setRGB(m+2, 0, 0, 0);
                      leds.setRGB(m+3, 0, 0, 0);

                      i-=1;
                      m-=1;

                      leds.flush();
                      // Microseconds
                      usleep(100000);
                    }
                  }
                }
              }
              break;
            }*/

          // Circular Versions of Turn Signals [Four Sets]

          // Left Turn Circular Animation [Four  Sets]
          case bwi_msgs::LEDAnimations::LEFT_TURN:
            {
              // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK
              while(!as_.isPreemptRequested() && !timeout && ros::ok())
              {
                // Creates an animation of leds which travels left along the strip

                ros::Duration time_running = ros::Time::now() - start;
                feedback_.time_running = time_running;
                as_.publishFeedback(feedback_);

                int j = circular_u_start-1;
                int k = circular_u_end+(1+(top_of_u_end-top_of_u_start));
                int l = top_of_u_start-1;
                int m = top_of_u_end+1;

                for (int i = 1; i <= 12;)
                {
                  // Terminate goal if preempted, timeout is reached, or ros fails
                  if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                  if (i == 1)
                  {
                    leds.setHSV(j, 22, 1, .1);
                    leds.setHSV(j+1, 22, 1, .1);
                    leds.setHSV(j+2, 22, 1, .1);
                    leds.setHSV(j+3, 22, 1, .1);

                    leds.setHSV(j+10, 22, 1, .1);
                    leds.setHSV(j+11, 22, 1, .1);
                    leds.setHSV(j+12, 22, 1, .1);
                    leds.setHSV(j+13, 22, 1, .1);

                    leds.setHSV(j+20, 22, 1, .1);
                    leds.setHSV(j+21, 22, 1, .1);
                    leds.setHSV(j+22, 22, 1, .1);
                    leds.setHSV(j+23, 22, 1, .1);

                    leds.setHSV(l, 22, 1, .1);
                    leds.setHSV(l+1, 22, 1, .1);
                    leds.setHSV(l+2, 22, 1, .1);
                    leds.setHSV(l+3, 22, 1, .1);

                    i+=4;
                    j+=4;
                    l+=4;
                  }
                  else
                  {
                    leds.setHSV(j, 22, 1, .1);
                    leds.setHSV(j+10, 22, 1, .1);
                    leds.setHSV(j+20, 22, 1, .1);
                    leds.setHSV(l, 22, 1, .1);
                    i+=1;
                    j+=1;
                    l+=1;
                  }

                  if (i > 1)
                  {
                    leds.setRGB(j-4, 0, 0, 0);
                    leds.setRGB((j+10)-4, 0, 0, 0);
                    leds.setRGB((j+20)-4, 0, 0, 0);
                    leds.setRGB(l-4, 0, 0, 0);
                  }

                  leds.flush();
                  // Microseconds
                  usleep(100000);

                   if (i == 12)
                  {
                    leds.setRGB(j, 0, 0, 0);
                    leds.setRGB(j-1, 0, 0, 0);
                    leds.setRGB(j-2, 0, 0, 0);
                    leds.setRGB(j-3, 0, 0, 0);

                    leds.setRGB(j+10, 0, 0, 0);
                    leds.setRGB((j+10)-1, 0, 0, 0);
                    leds.setRGB((j+10)-2, 0, 0, 0);
                    leds.setRGB((j+10)-3, 0, 0, 0);


                    leds.setRGB(j+20, 0, 0, 0);
                    leds.setRGB((j+20)-1, 0, 0, 0);
                    leds.setRGB((j+20)-2, 0, 0, 0);
                    leds.setRGB((j+20)-3, 0, 0, 0);

                    leds.setRGB(l, 0, 0, 0);
                    leds.setRGB(l-1, 0, 0, 0);
                    leds.setRGB(l-2, 0, 0, 0);
                    leds.setRGB(l-3, 0, 0, 0);


                    i+=1;
                    j+=1;
                    l+=1;

                    leds.flush();
                    // Microseconds
                    usleep(100000);
                  }
                }
              }
              break;
            }

          // Right Turn Circular Animation [Four  Sets]
          case bwi_msgs::LEDAnimations::RIGHT_TURN:
            {
              // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK
              while(!as_.isPreemptRequested() && !timeout && ros::ok())
              {
                // Creates an animation of leds which travels left along the strip

                ros::Duration time_running = ros::Time::now() - start;
                feedback_.time_running = time_running;
                as_.publishFeedback(feedback_);

                int j = circular_u_end+1;
                int k = circular_u_start-(1+(top_of_u_end-top_of_u_start));
                int l = top_of_u_end+1;
                int m = top_of_u_start-1;

                for (int i = 1; i <= 12;)
                {
                  // Terminate goal if preempted, timeout is reached, or ros fails
                  if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                  if (i == 1)
                  {
                    leds.setHSV(j, 22, 1, .1);
                    leds.setHSV(j-1, 22, 1, .1);
                    leds.setHSV(j-2, 22, 1, .1);
                    leds.setHSV(j-3, 22, 1, .1);

                    leds.setHSV(j-10, 22, 1, .1);
                    leds.setHSV(j-11, 22, 1, .1);
                    leds.setHSV(j-12, 22, 1, .1);
                    leds.setHSV(j-13, 22, 1, .1);

                    leds.setHSV(j-20, 22, 1, .1);
                    leds.setHSV(j-21, 22, 1, .1);
                    leds.setHSV(j-22, 22, 1, .1);
                    leds.setHSV(j-23, 22, 1, .1);

                    leds.setHSV(l, 22, 1, .1);
                    leds.setHSV(l-1, 22, 1, .1);
                    leds.setHSV(l-2, 22, 1, .1);
                    leds.setHSV(l-3, 22, 1, .1);

                    i+=4;
                    j-=4;
                    l-=4;
                  }
                  else
                  {
                    leds.setHSV(j, 22, 1, .1);
                    leds.setHSV(j-10, 22, 1, .1);
                    leds.setHSV(j-20, 22, 1, .1);
                    leds.setHSV(l, 22, 1, .1);
                    i+=1;
                    j-=1;
                    l-=1;
                  }

                  if (i > 1)
                  {
                    leds.setRGB(j+4, 0, 0, 0);
                    leds.setRGB((j-10)+4, 0, 0, 0);
                    leds.setRGB((j-20)+4, 0, 0, 0);
                    leds.setRGB(l+4, 0, 0, 0);
                  }

                  leds.flush();
                  // Microseconds
                  usleep(100000);

                   if (i == 12)
                  {
                    leds.setRGB(j, 0, 0, 0);
                    leds.setRGB(j+1, 0, 0, 0);
                    leds.setRGB(j+2, 0, 0, 0);
                    leds.setRGB(j+3, 0, 0, 0);

                    leds.setRGB(j+10, 0, 0, 0);
                    leds.setRGB((j-10)+1, 0, 0, 0);
                    leds.setRGB((j-10)+2, 0, 0, 0);
                    leds.setRGB((j-10)+3, 0, 0, 0);


                    leds.setRGB(j-20, 0, 0, 0);
                    leds.setRGB((j-20)+1, 0, 0, 0);
                    leds.setRGB((j-20)+2, 0, 0, 0);
                    leds.setRGB((j-20)+3, 0, 0, 0);

                    leds.setRGB(l, 0, 0, 0);
                    leds.setRGB(l+1, 0, 0, 0);
                    leds.setRGB(l+2, 0, 0, 0);
                    leds.setRGB(l+3, 0, 0, 0);


                    i+=1;
                    j-=1;
                    l-=1;

                    leds.flush();
                    // Microseconds
                    usleep(100000);
                  }
                }
              }
              break;
            }

          // Reverse Animtion
          case bwi_msgs::LEDAnimations::REVERSE:
            {
              // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK
              while(!as_.isPreemptRequested() && !timeout && ros::ok())
              {
                // Creates a pulsing animation

                ros::Duration time_running = ros::Time::now() - start;
                feedback_.time_running = time_running;
                as_.publishFeedback(feedback_);

                // Increase brightness
                for (float b = 0.0; b < 0.5; b += 0.02)
                {
                  // Terminate goal if preempted, timeout is reached, or ros fails
                  if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                  int j = back_left_beam_start;
                  int k = back_center_end;

                  for (int i = back_right_beam_end; i >= back_right_beam_start; i--)
                  {
                    leds.setHSV(i, 360, 1, b);
                    leds.setHSV(j, 360, 1, b);

                    if (k >= back_center_start)
                    {
                      leds.setHSV(k, 360, 1, b);
                      k--;
                    }

                    j--;
                  }

                  leds.flush();
                  // Microseconds
                  usleep(22500);
                }
                // Microseconds
                usleep(500000);

                // Decreases Brightness
                for (float b = 0.5; b >= 0.0; b -= 0.02)
                {
                  // Terminate goal if preempted, timeout is reached, or ros fails
                  if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                  int j = back_left_beam_start;
                  int k = back_center_end;

                  for (int i = back_right_beam_end; i >= back_right_beam_start; i--)
                  {
                    leds.setHSV(i, 360, 1, b);
                    leds.setHSV(j, 360, 1, b);

                    if (k >= back_center_start)
                    {
                      leds.setHSV(k, 360, 1, b);
                      k--;
                    }

                    j--;
                  }

                  leds.flush();
                  // Microseconds
                  usleep(22500);
                }
                // Microseconds
                usleep(500000);
              }
              break;
            }

          // Blocked Animation
          case bwi_msgs::LEDAnimations::BLOCKED:
            {
              // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK
              while(!as_.isPreemptRequested() && !timeout && ros::ok())
              {
                // Creates a pulsing animation

                ros::Duration time_running = ros::Time::now() - start;
                feedback_.time_running = time_running;
                as_.publishFeedback(feedback_);

                // Increase brightness
                for (float b = 0.0; b <= 0.5; b += 0.02)
                {
                  // Terminate goal if preempted, timeout is reached, or ros fails
                  if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                  for (int i = led_count; i >= 0; i--)
                  {
                    leds.setHSV(i, 360, 1, b);
                  }

                  leds.flush();
                  // Microseconds
                  if (b != 0.5) { usleep(60000); }
                }
                // Microseconds
                usleep(45000);

                // Decreases Brightness
                for (float b = 0.5; b >= 0.0; b -= 0.02)
                {
                  // Terminate goal if preempted, timeout is reached, or ros fails
                  if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                  for (int i = led_count; i >= 0; i--)
                  {
                    leds.setHSV(i, 360, 1, b);
                  }

                  leds.flush();
                  // Microseconds
                  if (b != 0.0) { usleep(60000); }
                }
                // Microseconds
                usleep(45000);
              }
              break;
            }
          // Up
          case bwi_msgs::LEDAnimations::UP:
            {
              // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK
              while(!as_.isPreemptRequested() && !timeout && ros::ok())
              {
                // Creates an animation of leds which travels up along the strip

                ros::Duration time_running = ros::Time::now() - start;
                feedback_.time_running = time_running;
                as_.publishFeedback(feedback_);

                int l = front_left_beam_start;
                int k = front_right_beam_start;
                int j = back_left_beam_start;

                for (int i = back_right_beam_start; i <= back_right_beam_end;)
                {
                  // Terminate goal if preempted, timeout is reached, or ros fails
                  if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                  if (i == back_right_beam_start)
                  {
                    leds.setHSV(i, 240, 1, .1);
                    leds.setHSV(i+1, 240, 1, .1);
                    leds.setHSV(i+2, 240, 1, .1);
                    leds.setHSV(i+3, 240, 1, .1);

                    leds.setHSV(l, 240, 1, .1);
                    leds.setHSV(l+1, 240, 1, .1);
                    leds.setHSV(l+2, 240, 1, .1);
                    leds.setHSV(l+3, 240, 1, .1);

                    leds.setHSV(k, 240, 1, .1);
                    leds.setHSV(k-1, 240, 1, .1);
                    leds.setHSV(k-2, 240, 1, .1);
                    leds.setHSV(k-3, 240, 1, .1);

                    leds.setHSV(j, 240, 1, .1);
                    leds.setHSV(j-1, 240, 1, .1);
                    leds.setHSV(j-2, 240, 1, .1);
                    leds.setHSV(j-3, 240, 1, .1);

                    i+=4;
                    l+=4;
                    k-=4;
                    j-=4;
                  }
                  else
                  {
                    leds.setHSV(i, 240, 1, .1);
                    leds.setHSV(l, 240, 1, .1);
                    leds.setHSV(k, 240, 1, .1);
                    leds.setHSV(j, 240, 1, .1);
                    i+=1;
                    l+=1;
                    k-=1;
                    j-=1;
                  }

                  if (i > back_right_beam_start)
                  {
                    leds.setRGB(i-4, 0, 0, 0);
                    leds.setRGB(l-4, 0, 0, 0);
                    leds.setRGB(k+4, 0, 0, 0);
                    leds.setRGB(j+4, 0, 0, 0);
                  }

                  leds.flush();
                  // Microseconds
                  usleep(100000);

                  if (i == back_right_beam_end)
                  {
                    leds.setRGB(i, 0, 0, 0);
                    leds.setRGB(i-1, 0, 0, 0);
                    leds.setRGB(i-2, 0, 0, 0);
                    leds.setRGB(i-3, 0, 0, 0);

                    leds.setRGB(l, 0, 0, 0);
                    leds.setRGB(l-1, 0, 0, 0);
                    leds.setRGB(l-2, 0, 0, 0);
                    leds.setRGB(l-3, 0, 0, 0);

                    leds.setRGB(k, 0, 0, 0);
                    leds.setRGB(k+1, 0, 0, 0);
                    leds.setRGB(k+2, 0, 0, 0);
                    leds.setRGB(k+3, 0, 0, 0);

                    leds.setRGB(j, 0, 0, 0);
                    leds.setRGB(j+1, 0, 0, 0);
                    leds.setRGB(j+2, 0, 0, 0);
                    leds.setRGB(j+3, 0, 0, 0);

                    i+=1;
                    l+=1;
                    k-=1;
                    j-=1;

                    leds.flush();
                    // Microseconds
                    usleep(100000);
                  }
                }
              }
              break;
            }
          // Down
          case bwi_msgs::LEDAnimations::DOWN:
            {
              // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK
              while(!as_.isPreemptRequested() && !timeout && ros::ok())
              {
                // Creates an animation of leds which travels down along the strip

                ros::Duration time_running = ros::Time::now() - start;
                feedback_.time_running = time_running;
                as_.publishFeedback(feedback_);

                int l = front_left_beam_end;
                int k = front_right_beam_end;
                int j = back_left_beam_end;

                for (int i = back_right_beam_end; i >= back_right_beam_start;)
                {
                  // Terminate goal if preempted, timeout is reached, or ros fails
                  if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                  if (i == back_right_beam_end)
                  {
                    leds.setHSV(i, 240, 1, .1);
                    leds.setHSV(i-1, 240, 1, .1);
                    leds.setHSV(i-2, 240, 1, .1);
                    leds.setHSV(i-3, 240, 1, .1);

                    leds.setHSV(l, 240, 1, .1);
                    leds.setHSV(l-1, 240, 1, .1);
                    leds.setHSV(l-2, 240, 1, .1);
                    leds.setHSV(l-3, 240, 1, .1);

                    leds.setHSV(k, 240, 1, .1);
                    leds.setHSV(k+1, 240, 1, .1);
                    leds.setHSV(k+2, 240, 1, .1);
                    leds.setHSV(k+3, 240, 1, .1);

                    leds.setHSV(j, 240, 1, .1);
                    leds.setHSV(j+1, 240, 1, .1);
                    leds.setHSV(j+2, 240, 1, .1);
                    leds.setHSV(j+3, 240, 1, .1);

                    i-=4;
                    l-=4;
                    k+=4;
                    j+=4;

                  }
                  else
                  {
                    leds.setHSV(i, 240, 1, .1);
                    leds.setHSV(l, 240, 1, .1);
                    leds.setHSV(k, 240, 1, .1);
                    leds.setHSV(j, 240, 1, .1);
                    i-=1;
                    l-=1;
                    k+=1;
                    j+=1;
                  }

                  if (i < back_right_beam_end)
                  {
                    leds.setRGB(i+4, 0, 0, 0);
                    leds.setRGB(l+4, 0, 0, 0);
                    leds.setRGB(k-4, 0, 0, 0);
                    leds.setRGB(j-4, 0, 0, 0);
                  }

                  leds.flush();
                  // Microseconds
                  usleep(100000);

                  if (i == back_right_beam_start)
                  {
                    leds.setRGB(i, 0, 0, 0);
                    leds.setRGB(i+1, 0, 0, 0);
                    leds.setRGB(i+2, 0, 0, 0);
                    leds.setRGB(i+3, 0, 0, 0);

                    leds.setRGB(l, 0, 0, 0);
                    leds.setRGB(l+1, 0, 0, 0);
                    leds.setRGB(l+2, 0, 0, 0);
                    leds.setRGB(l+3, 0, 0, 0);

                    leds.setRGB(k, 0, 0, 0);
                    leds.setRGB(k-1, 0, 0, 0);
                    leds.setRGB(k-2, 0, 0, 0);
                    leds.setRGB(k-3, 0, 0, 0);

                    leds.setRGB(j, 0, 0, 0);
                    leds.setRGB(j-1, 0, 0, 0);
                    leds.setRGB(j-2, 0, 0, 0);
                    leds.setRGB(j-3, 0, 0, 0);

                    i-=1;
                    l-=1;
                    k+=1;
                    j+=1;

                    leds.flush();
                    // Microseconds
                    usleep(100000);
                  }
                }
              }
              break;
            }

          // Need Assistance Animation
          case bwi_msgs::LEDAnimations::NEED_ASSIST:
            {
              // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK
              while(!as_.isPreemptRequested() && !timeout && ros::ok())
              {
                // Creates a pulsing animation

                ros::Duration time_running = ros::Time::now() - start;
                feedback_.time_running = time_running;
                as_.publishFeedback(feedback_);

                // Increase brightness
                for (float b = 0.0; b <= 0.3; b += 0.02)
                {
                  // Terminate goal if preempted, timeout is reached, or ros fails
                  if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                  for (int i = led_count; i >= 0; i--)
                  {
                    leds.setHSV(i, 240, 1, b);
                  }

                  leds.flush();
                  // Microseconds
                  if (b != 0.3) { usleep(70000); }
                }
                // Microseconds
                usleep(75000);

                // Decreases Brightness
                for (float b = 0.3; b >= 0.0; b -= 0.02)
                {
                  // Terminate goal if preempted, timeout is reached, or ros fails
                  if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                  for (int i = led_count; i >= 0; i--)
                  {
                    leds.setHSV(i, 240, 1, b);
                  }

                  leds.flush();
                  // Microseconds
                  if (b != 0.0) { usleep(70000); }
                }
                // Microseconds
                usleep(75000);
              }
              break;
            }
        }

        // Successful Execution Logic
        if(success || timeout)
        {
          ROS_INFO("%s: Succeeded", action_name_.c_str());
          result_.result = bwi_msgs::LEDActionResult::SUCCESS;
          std::ostringstream stringStream;
          stringStream << "Action completed successfully. " << goal->type;
          result_.status = stringStream.str();
          as_.setSucceeded(result_);
          timeout_timer.stop();
          leds.clear();
          ROS_INFO("Cleared LED Strip");
          check_camera_status();
          timeout = false;
        }
      }
    }
    catch(const serial::SerialException &e)
    {
      result_.result = bwi_msgs::LEDActionResult::FAILURE;
      std::ostringstream stringStream;
      stringStream << "Action, " << goal->type << ", failed due failure with serial communication to LED microcontroller.";
      result_.status = stringStream.str();
      ROS_ERROR("Action execution failed, unable to write to microcontroller,");
      timeout_timer.stop();
      connected = false;
      as_.setPreempted(result_);
      success = false;
      timeout = false;
      ROS_ERROR("Ensure LED microcontroller is connected.");
      ROS_ERROR("Attempting to reconnect to LED microcontroller.");
    }
  }
};

/*******************************************************
*                                                      *
*                Service Callbacks                     *
*                                                      *
********************************************************/
bool clear_strip(bwi_msgs::LEDClear::Request  &req,
                 bwi_msgs::LEDClear::Response &res)
{
  try
  {
    leds.clear();
    ROS_INFO("Cleared LED Strip");
    check_camera_status();
    res.success = true;
    return true;
  }
  catch(const serial::SerialException &e)
  {
    res.success = false;
    res.status = "Failure with serial communication to LED microcontroller";
    ROS_ERROR("Service execution failed, unable to write to microcontroller,");
    ROS_ERROR("Ensure LED microcontroller is connected.");
    ROS_ERROR("Attempting to reconnect to LED microcontroller.");
    connected = false;
  }
}

bool test_strip(bwi_msgs::LEDTestStrip::Request  &req,
                bwi_msgs::LEDTestStrip::Response &res)
{
  try
  {
    switch(req.type.test_type)
    {
      // Set Every Fifth LED
      case bwi_msgs::LEDTestType::SET_EVERY_FIFTH:
        {
          leds.clear();
          sleep(1);

          for (int l=5; l < led_count; l+=5)
          {
            if (l%10 == 0)
            {
              leds.setHSV(l, 120, 1, .1);
            }
            else
            {
              leds.setHSV(l, 250, 1, .1);
            }
          }
          leds.flush();
          sleep(1);

          ROS_INFO("Set every fifth LED on strip");
          res.success = true;
          return true;
        }
      // Set First Five LEDs
      case bwi_msgs::LEDTestType::SET_FIRST_FIVE:
        {
          leds.clear();
          sleep(1);

          leds.setHSV(0, 240, 1, .1);
          leds.setHSV(1, 120, 1, .1);
          leds.setHSV(2, 240, 1, .1);
          leds.setHSV(3, 120, 1, .1);
          leds.setHSV(4, 240, 1, .1);

          leds.flush();
          sleep(1);

          ROS_INFO("Set first five LEDs on strip");
          res.success = true;
          return true;
        }
      // Test LEDs on Strip
      case bwi_msgs::LEDTestType::TEST_STRIP:
        {
          ROS_INFO("Testing LED Strip, will take about 30 seconds to complete.");

          leds.clear();
          sleep(1);

          for (int i=0; i < 360; i+=15)
          {
            for (int l=0; l < led_count; l++)
            {
              leds.setHSV(l, i, 1, .1);
            }
            leds.flush();
            sleep(1);
          }

          leds.clear();
          ROS_INFO("Cleared LED Strip");
          ROS_INFO("Tested Colors on LED strip");
          res.success = true;
          return true;
        }
      default:
        {
          res.success = false;
          std::ostringstream stringStream;
          stringStream << "Unknown type requested: " << req.type.test_type;
          res.success = false;
          res.status = stringStream.str();
          return false;
        }
    }
  }
  catch(const serial::SerialException &e)
  {
    res.success = false;
    res.status = "Failure with serial communication to LED microcontroller";
    ROS_ERROR("Service execution failed, unable to write to microcontroller,");
    ROS_ERROR("Ensure LED microcontroller is connected.");
    ROS_ERROR("Attempting to reconnect to LED microcontroller.");
    connected = false;
  }
}

bool set_camera(bwi_msgs::LEDSetCamera::Request  &req,
                bwi_msgs::LEDSetCamera::Response &res)
{
  try
  {
    switch(req.type.camera_status)
    {
      // Camera On
      case bwi_msgs::LEDCameraStatus::CAMERA_ON:
        {
          camera_on = true;
          check_camera_status();
          res.success = true;
          return true;
        }
      // Camera Off
      case bwi_msgs::LEDCameraStatus::CAMERA_OFF:
        {
          camera_on = false;
          leds.clear();
          ROS_INFO("Set camera indicator off and cleared LED strip");
          res.success = true;
          return true;
        }
      default:
        {
          res.success = false;
          std::ostringstream stringStream;
          stringStream << "Unknown type requested: " << req.type.camera_status;
          res.success = false;
          res.status = stringStream.str();
          return false;
        }
    }
  }
  catch(const serial::SerialException &e)
  {
    res.success = false;
    res.status = "Failure with serial communication to LED microcontroller";
    ROS_ERROR("Service execution failed, unable to write to microcontroller,");
    ROS_ERROR("Ensure LED microcontroller is connected.");
    ROS_ERROR("Attempting to reconnect to LED microcontroller.");
    connected = false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "led_control_server");
  ros::NodeHandle n;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, ledSigintHandler);

  ros::NodeHandle privateNode("~");

  // Node Parameters
  // LED and Microcontroller Initialization
  privateNode.param<int>("led_count",led_count,60);
  privateNode.param<string>("serial_port",serial_port,"/dev/metromini");

  // LED Segments
  privateNode.param<int>("camera_indicator_start",camera_indicator_start,29);
  privateNode.param<int>("camera_indicator_end",camera_indicator_end,30);

  privateNode.param<int>("back_center_start",back_center_start,85);
  privateNode.param<int>("back_center_end",back_center_end,94);

  // For center segments, start is in the center and end is the left/right most led of the segment
  // Point of reference is from facing the segment
  privateNode.param<int>("front_center_left_start",front_center_left_start,30);
  privateNode.param<int>("front_center_left_end",front_center_left_end,34);

  privateNode.param<int>("front_center_right_start",front_center_right_start,29);
  privateNode.param<int>("front_center_right_end",front_center_right_end,15);

  privateNode.param<int>("back_center_left_start",back_center_left_start,89);
  privateNode.param<int>("back_center_left_end",back_center_left_end,75);

  privateNode.param<int>("back_center_right_start",back_center_right_start,90);
  privateNode.param<int>("back_center_right_end",back_center_right_end,94);

  // For beam segments, start is bottom most led and end is top most led of the segment
  // Point of reference is from facing the segment
  privateNode.param<int>("front_left_beam_start",front_left_beam_start,60);
  privateNode.param<int>("front_left_beam_end",front_left_beam_end,73);

  privateNode.param<int>("front_right_beam_start",front_right_beam_start,119);
  privateNode.param<int>("front_right_beam_end",front_right_beam_end,106);

  privateNode.param<int>("back_left_beam_start",back_left_beam_start,59);
  privateNode.param<int>("back_left_beam_end",back_left_beam_end,46);

  privateNode.param<int>("back_right_beam_start",back_right_beam_start,0);
  privateNode.param<int>("back_right_beam_end",back_right_beam_end,13);

  // For Circular segments, start is left most led of the "U" shaped segment and end is right most led of the "U" shaped segment
  // Point of reference is from facing the segment
  privateNode.param<int>("circular_u_start",circular_u_start,75);
  privateNode.param<int>("circular_u_end",circular_u_end,105);

  // For top segment, start is left most led and end is right most led of the segment
  // Point of reference is from facing the segment
  privateNode.param<int>("top_of_u_start",top_of_u_start,25);
  privateNode.param<int>("top_of_u_end",top_of_u_end,34);

  // Initial connection to microcontroller

  // Service Server advertisers
  ros::ServiceServer clear_service = n.advertiseService("led_clear", clear_strip);
  ros::ServiceServer test_strip_service = n.advertiseService("led_test", test_strip);
  ros::ServiceServer set_camera_service = n.advertiseService("led_set_camera", set_camera);

  // Action Server advertisers
  LEDControlAction led_control_server(ros::this_node::getName());

  ros::Rate r(10); // 10 hz

  while(ros::ok())
  {
      if(!connected)
      {
        connect(serial_port, 115200);
      }
      ros::spinOnce();
      r.sleep();
  }

  return 0;
}
