#include <ros/ros.h>
#include <ros/package.h>

//used for registering Ctrl-C events
#include <signal.h>

//ROS messages
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinova_msgs/PoseVelocity.h>

//ROS action client used for calling actions
#include <actionlib/client/simple_action_client.h>

//JACO messages and actions
#include <kinova_msgs/FingerPosition.h>
#include <kinova_msgs/HomeArm.h>

//our own arm library 
#include <segbot_arm_manipulation/arm_utils.h>

#include <sensor_msgs/Joy.h>
#include <bwi_services/SpeakMessage.h>

#include <iostream>
using namespace std;

#define NUM_JOINTS 8 //6+2 for the arm

//global variables for joystick values
float linear_x;
float linear_y;
float linear_z;
float angular_x;
float angular_y;
float angular_z;
float finger_1 = 7200;
float finger_2 = 7200;

bool fingers_opening;
bool fingers_closing;
bool fingers_opening_fully;
bool fingers_closing_fully;
bool fingers_changed;
bool homingArm;

bool emergency_brake;
float turn_multiplier = 0.4;
float speed_multiplier = 0.4;

bool mode_changed;
bool mode_requested;

bool ARM_MODE = false;
bool BASE_MODE = true;

bool mode = ARM_MODE;

bool base_allowed = true;


//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

void printArmControls() {
  cout << "Arm controls:\n";
  cout << "Linear velocity x - Left axis stick left/right\n";
  cout << "Linear velocity y - Left axis stick up/down\n";
  cout << "Linear velocity z - Left trigger/Right trigger\n";
  cout << "Angular velocity x - Right axis stick left/right\n";
  cout << "Angular velocity y - Right axis stick up/down\n";
  cout << "Angular velocity z - Left back button/Right back button\n";
  cout << "Fingers:\n";
  cout << "Open slowly - 4-direction pad LEFT\n";
  cout << "Close slowly - 4-direction pad RIGHT\n";
  cout << "Open fully - 4-direction pad UP\n";
  cout << "Close fully - 4-direction pad DOWN\n";
  cout << "Home arm - Center button\n";
  cout << "Switch modes - Back + Start buttons\n";
}

void printBaseControls() {
  cout << "Base controls:\n";
  cout << "Forward/Backward - Left axis stick up/down\n";
  cout << "Turn - Right axis stick left/right\n";
  cout << "Increase/Decrease speed - Y/A\n";
  cout << "Increase/Decrease turn speed - X/B\n";
}

// Call back function when joy stick message recieved
void joy_cb(const sensor_msgs::Joy::ConstPtr& joy) {

  int fully_open_button = joy->buttons[13]; // UP (4-Direction pad)
  int fully_close_button = joy->buttons[14]; // DOWN (4-Direction pad)
  int slowly_open_button = joy->buttons[11]; // LEFT (4-Direction pad)
  int slowly_close_button = joy->buttons[12]; // RIGHT (4-Direction pad)

  int increase_speed_button = joy->buttons[3]; // Y
  int decrease_speed_button = joy->buttons[0]; // A
  int increase_turn_button = joy->buttons[2]; // X
  int decrease_turn_button = joy->buttons[1]; // B

  // To switch between arm and base
  int switch_mode_button1 = joy->buttons[6]; 
  int switch_mode_button2 = joy->buttons[7];

  int home_button = joy->buttons[8]; // home button - for homing the arm

  float positive_multiplier = 0.6;
  float negative_multiplier = -0.6;
  float lower_negative_multiplier = -0.8;

  float linear_x_input = joy->axes[1]; //left axis stick L/R
  float linear_y_input = joy->axes[0]; //left axis stick U/D
  float linear_z_input = joy->axes[2] - joy->axes[5]; //left trigger (up) - right trigger (down)

  float angular_x_input = joy->axes[4]; //right axis stick L/R
  float angular_y_input = joy->axes[3]; //right axis stick U/D
  float angular_z_input = joy->buttons[4] - joy->buttons[5]; //left back button (up) - right back button (down)

  if(switch_mode_button1 != 0 && switch_mode_button2 != 0){
    mode_changed = true;
    return;
  }
  else if (switch_mode_button1 != 0 || switch_mode_button2 != 0) {
    mode_requested = true;
    return;
  }

  //remove all noise
  if(linear_x_input < 0.2 && linear_x_input > -0.2) linear_x = 0; //make it 0
    else linear_x = positive_multiplier * linear_x_input; 

  if(linear_y_input < 0.2 && linear_y_input > -0.2) linear_y = 0; //make it 0
    else linear_y = positive_multiplier * linear_y_input;

  if(linear_z_input < 0.2 && linear_z_input > -0.2) linear_z = 0;  //make it 0
    else linear_z = negative_multiplier * linear_z_input;

  if(angular_x_input < 0.2 && angular_x_input > -0.2) angular_x = 0; //make it 0
    else angular_x = positive_multiplier * angular_x_input;

  if(angular_y_input < 0.2 && angular_y_input > -0.2) angular_y = 0; //make it 0
    else angular_y = positive_multiplier * angular_y_input;

  if(angular_z_input < 0.2 && angular_z_input > -0.2) angular_z = 0;  //make it 0
    else angular_z = lower_negative_multiplier * angular_z_input;


  //100 is open, 7200 is closed
  // if any of the finger buttons is pressed
  if (slowly_open_button != 0 || slowly_close_button != 0 || fully_open_button != 0 || fully_close_button != 0)
    fingers_changed = true;
  else 
    fingers_changed = false;
    
  if (fully_open_button != fully_close_button && (slowly_close_button == 0 && slowly_open_button == 0)) {
    if (fully_open_button != 0) 
      fingers_opening_fully = true;
    else 
      fingers_closing_fully = true;
  } else {
    fingers_opening_fully = false;
    fingers_closing_fully = false;
  }

  //if only one button pressed
  if ((slowly_close_button != slowly_open_button) && (fully_open_button == 0 && fully_close_button == 0)) { 
    if (slowly_close_button != 0) {
      fingers_opening = true;
      fingers_closing = false;
    } else {
      fingers_opening = false;
      fingers_closing = true;
    }
  } else {
    fingers_opening = false;
    fingers_closing = false;
  }

  if (home_button != 0) {
    homingArm = true;
    emergency_brake = true;
  } else {
    homingArm = false;
    emergency_brake = false;
  }

  if (increase_speed_button) {
    if (speed_multiplier < 1)
      speed_multiplier += 0.2;
    ROS_INFO("Speed multiplier increased to %f", speed_multiplier);
  }
  else if (decrease_speed_button) {
    if (speed_multiplier > 0.2)
      speed_multiplier -= 0.2;
      ROS_INFO("Speed multiplier decreased to %f", speed_multiplier);
  }

  if (increase_turn_button) {
    if (turn_multiplier < 1)
      turn_multiplier += 0.2;
    ROS_INFO("Turn multiplier increased to %f", turn_multiplier);
  }
  else if (decrease_turn_button) {
    if (turn_multiplier > 0.2)
      turn_multiplier -= 0.2;
    ROS_INFO("Turn multiplier decreased to %f", turn_multiplier);
  }

}

// Blocking call for user input
void pressEnter(std::string message){
  std::cout << message;
  while (true){
    char c = std::cin.get();
    if (c == '\n')
      break;
    else if (c == 'q'){
      ros::shutdown();
      exit(1);
    }
    else {
      std::cout <<  message;
    }
  }
}

bool allZeros(kinova_msgs::PoseVelocity velocityMsg) {
  return (velocityMsg.twist_linear_x == 0 && velocityMsg.twist_linear_y == 0 
        && velocityMsg.twist_linear_z == 0
      && velocityMsg.twist_angular_x == 0 && velocityMsg.twist_angular_y == 0 
        && velocityMsg.twist_angular_z == 0);
}

void publishArm(ros::Publisher pub_arm) {
  kinova_msgs::PoseVelocity velocityMsg;
  //construct message
  velocityMsg.twist_linear_x = linear_x;
  velocityMsg.twist_linear_y = linear_y;
  velocityMsg.twist_linear_z = linear_z; 
  velocityMsg.twist_angular_x = angular_x;
  velocityMsg.twist_angular_y = angular_y;
  velocityMsg.twist_angular_z = angular_z;

  //ROS_INFO("Linear x: %f\n", linear_x);
  //ROS_INFO("Linear y: %f\n", linear_y);
  //ROS_INFO("Linear z: %f\n", linear_z);
  //ROS_INFO("Angular x: %f\n", angular_x);
  //ROS_INFO("Angular y: %f\n", angular_y);
  //ROS_INFO("Angular z: %f\n", angular_z);

  if (allZeros(velocityMsg))
    return;
    
  //publish velocity message
  //ROS_INFO("Publishing Velocity Message");
  pub_arm.publish(velocityMsg);
}

void publishFingers(ros::Rate r) {
  if (fingers_closing_fully) {
    finger_1 = 7200;
    finger_2 = 7200;
    segbot_arm_manipulation::moveFingers(finger_1, finger_2);
    fingers_closing_fully = false;
    return;
  }
  
  if (fingers_opening_fully) {
    finger_1 = 100;
    finger_2 = 100;
    segbot_arm_manipulation::moveFingers(finger_1, finger_2);
    fingers_opening_fully = false;
    return;
  }
  
  // send only if buttons are pressed
  while(ros::ok() && fingers_opening) {
    if (finger_1 >= 700 && finger_2 >= 700) {
      finger_1 -= 600;
      finger_2 -= 600;
      segbot_arm_manipulation::moveFingers(finger_1, finger_2);
    }
  
    ros::spinOnce();
    r.sleep();
  }
   
  // send only if buttons are pressed
  while(ros::ok() && fingers_closing) {
    ROS_INFO("Fingers closed\n"); 
    ROS_INFO("Finger1->%f\n", finger_1);
    ROS_INFO("Finger2->%f\n", finger_2);
    if (finger_1 <= 6600 && finger_2 <= 6600) {
      ROS_INFO("Closing fingers\n");
      finger_1 += 600;
      finger_2 += 600;
      segbot_arm_manipulation::moveFingers(finger_1, finger_2);
    }
  
    ros::spinOnce();
    r.sleep();
  }
}

void homeArm(ros::NodeHandle n) {
  ROS_INFO("Homing arm\n");
  segbot_arm_manipulation::homeArm(n);
  finger_1 = 7200;
  finger_2 = 7200;
  segbot_arm_manipulation::moveFingers(finger_1, finger_2);
  homingArm = false;
}

bool allZeros(geometry_msgs::Twist base_msg) {
  return (base_msg.linear.x == 0 && base_msg.linear.y == 0 
        && base_msg.linear.z == 0
      && base_msg.angular.x == 0 && base_msg.angular.y == 0 
        && base_msg.angular.z == 0);
}

void publishBase(ros::Publisher pub_base) {
  geometry_msgs::Twist base_msg;
  base_msg.linear.x = speed_multiplier * linear_x;
  base_msg.angular.z = turn_multiplier * angular_y;
  
  if (allZeros(base_msg))
    return;

  pub_base.publish(base_msg);
}

void emergency_braking(ros::Publisher pub_base) {
  ROS_INFO("Braking.");
  geometry_msgs::Twist base_msg;
  base_msg.linear.x = 0;
  base_msg.angular.z = 0;

  pub_base.publish(base_msg);
  emergency_brake = false;
}

void switchMode(ros::ServiceClient speak_message_client, ros::NodeHandle n) {
  if (!base_allowed) {
    mode_changed = false;
    return;
  }
  bwi_services::SpeakMessage speak_srv1;
  speak_srv1.request.message = "Switching mode.";
  speak_message_client.call(speak_srv1);

  bwi_services::SpeakMessage speak_srv2;
  if (mode == ARM_MODE) {
    bool safe = segbot_arm_manipulation::makeSafeForTravel(n);
    if (!safe) {
      ROS_INFO("Could not switch to base mode. Arm not safe for travel.");
      mode_changed = false;
      return;
    }
    mode = BASE_MODE;
    ROS_INFO("Now in BASE Mode");
    speak_srv2.request.message = "Now in base mode.";
    printBaseControls();
  } else {
    mode = ARM_MODE;
    homeArm(n);
    ROS_INFO("Now in ARM Mode");
    speak_srv2.request.message = "Now in arm mode.";
    printArmControls();
  }
  mode_changed = false;
  speak_message_client.call(speak_srv2);
}

void modeRequested(ros::ServiceClient speak_message_client) {
  bwi_services::SpeakMessage speak_srv;

  if (mode == ARM_MODE) {
    speak_srv.request.message = "Arm mode.";
  } else {
    speak_srv.request.message = "Base mode.";
  }

  mode_requested = false;
  speak_message_client.call(speak_srv);
}

int main(int argc, char * *argv) {
  // Intialize ROS with this node name
  ros::init(argc, argv, "segbot_arm_joystick_node");

  ros::NodeHandle nh("~");
  nh.param<bool>("base", base_allowed, true);

  ros::NodeHandle n;
  ros::Subscriber joy_sub;

  ros::Publisher pub_arm;
  ros::Publisher pub_base;

  //construction the action request
  kinova_msgs::SetFingersPositionGoal goalFinger;

  // joy is the name of the topic the joystick publishes to
  joy_sub  = n.subscribe<sensor_msgs::Joy>("joy", 10, joy_cb);
  
  pub_arm = n.advertise<kinova_msgs::PoseVelocity>("/m1n6s200_driver/in/cartesian_velocity", 10);
  pub_base = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::ServiceClient speak_message_client = n.serviceClient<bwi_services::SpeakMessage>("/speak_message_service/speak_message");

  //register ctrl-c
  signal(SIGINT, sig_handler);
  
  segbot_arm_manipulation::homeArm(n);
  segbot_arm_manipulation::moveFingers(finger_1, finger_2); //inital position = 7200
  
  //close fingers and "home" the arm
  pressEnter("Press [Enter] to start");
  
  printArmControls();

  double pub_rate = 40.0; //we publish at 40 hz
  ros::Rate r(pub_rate);
  
  while (ros::ok()) {
    while (ros::ok() && (!fingers_changed) && (!homingArm) && (!mode_changed) && (!mode_requested)) {
      ros::spinOnce();
      r.sleep();
	
      if (mode == ARM_MODE) 
        publishArm(pub_arm);
      if (mode == BASE_MODE)
        publishBase(pub_base);
    }

    if (ros::ok()) {
      if (mode_changed) {
        switchMode(speak_message_client, n);
        ros::spinOnce();
        r.sleep();
        continue; 
      }
      if (mode_requested) {
        modeRequested(speak_message_client);
        ros::spinOnce();
        r.sleep();
        continue; 
      } 
    }

    if (ros::ok() && mode == ARM_MODE) {
      if (fingers_closing_fully || fingers_opening_fully || fingers_opening || fingers_closing)
        publishFingers(r);
      else if (homingArm)
        homeArm(n);

      ros::spinOnce();
      r.sleep();
      continue;
    }

    if (ros::ok() && mode == BASE_MODE) {
      if (emergency_brake)
        emergency_braking(pub_base);

      ros::spinOnce();
      r.sleep();
      continue; 
    }
  }
  
  
  //publish 0 velocity command -- otherwise arm/base will continue moving with the last command for 0.25 seconds
  ROS_INFO("Ending");
  kinova_msgs::PoseVelocity arm_msg;
  arm_msg.twist_linear_x = 0;
  arm_msg.twist_linear_y = 0;
  arm_msg.twist_linear_z = 0; 
  arm_msg.twist_angular_x = 0;
  arm_msg.twist_angular_y = 0;
  arm_msg.twist_angular_z = 0;
  pub_arm.publish(arm_msg);

  geometry_msgs::Twist base_msg;
  base_msg.linear.x = 0;
  base_msg.linear.y = 0;
  base_msg.linear.z = 0;
  base_msg.angular.x = 0;
  base_msg.angular.y = 0;
  base_msg.angular.z = 0;
  pub_base.publish(base_msg);

  //the end
  ros::shutdown();
}
