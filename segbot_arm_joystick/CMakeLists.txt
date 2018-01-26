cmake_minimum_required(VERSION 2.8.3)
project(segbot_arm_joystick)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  bwi_msgs
  bwi_services
  kinova_msgs
  roscpp
  rospy
  std_msgs
  segbot_arm_manipulation
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
   bwi_msgs
   bwi_services
   kinova_msgs
   segbot_arm_manipulation
   std_msgs
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
