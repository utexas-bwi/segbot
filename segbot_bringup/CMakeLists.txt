# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(segbot_bringup)
find_package(catkin REQUIRED robot_state_publisher)

catkin_package(
  LIBRARIES segbot_state_publisher_solver
  INCLUDE_DIRS include
  CATKIN_DEPENDS robot_state_publisher
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(segbot_state_publisher_solver
  src/robot_state_publisher.cpp
)
target_link_libraries(segbot_state_publisher_solver ${catkin_LIBRARIES})

add_executable(segbot_state_publisher src/joint_state_listener.cpp)
target_link_libraries(segbot_state_publisher segbot_state_publisher_solver log4cxx)

install(TARGETS 
  segbot_state_publisher_solver
  segbot_state_publisher
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install all launch files
install(DIRECTORY launch/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Install the teleoperation script for rosrun 
install(PROGRAMS 
  scripts/teleop_twist_keyboard
  scripts/get_addr
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
endif (CATKIN_ENABLE_TESTING)
