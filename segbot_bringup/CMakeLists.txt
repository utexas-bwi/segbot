# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(segbot_bringup)
find_package(catkin REQUIRED)
catkin_package()

# Install all udev files
install(DIRECTORY udev/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/udev)

# Install all launch files
install(DIRECTORY launch/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Install the teleoperation script for rosrun 
install(PROGRAMS 
  scripts/get_addr
  scripts/teleop_twist_keyboard
  scripts/udev_install
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# unit tests are enabled selectively
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/includes)

  ## launch/segbot_kinect*.launch files fail, see:
  ## ros-drivers/openni_launch#10.
  ##
  ## Test the individual files that do work, restore testing of the
  ## rest when openni_launch has been fixed.
  #roslaunch_add_file_check(launch) 
  roslaunch_add_file_check(launch/segbot_hokuyo.launch)
  roslaunch_add_file_check(launch/segbot.launch)
  roslaunch_add_file_check(launch/segway_base.launch)
endif()
