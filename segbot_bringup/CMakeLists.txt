# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(segbot_bringup)
find_package(catkin REQUIRED)
catkin_package()

# Install all launch files
install(DIRECTORY launch/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Install the teleoperation script for rosrun 
install(PROGRAMS scripts/teleop_twist_keyboard
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
