cmake_minimum_required(VERSION 2.8.3)
project(segbot_arm_joystick)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  bwi_msgs
  bwi_services
  kinova_msgs
  move_base_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_utils
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  segbot_arm_manipulation
  segbot_arm_perception
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
  CATKIN_DEPENDS bwi_msgs
  CATKIN_DEPENDS bwi_services
  CATKIN_DEPENDS kinova_msgs
  CATKIN_DEPENDS move_base_msgs
  CATKIN_DEPENDS moveit_msgs
  CATKIN_DEPENDS std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
