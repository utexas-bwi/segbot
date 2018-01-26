cmake_minimum_required(VERSION 2.8.3)
project(obstacle_avoidance)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
        agile_grasp
  geometry_msgs
  kinova_msgs
  message_generation
  moveit_msgs
  moveit_ros_planning_interface
  moveit_utils
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  segbot_arm_manipulation
  segbot_arm_perception
  sensor_msgs
  std_msgs
  tf_conversions
)

catkin_package(
  CATKIN_DEPENDS kinova_msgs
  CATKIN_DEPENDS std_msgs
)

###########
## Build ##
###########

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(collision_demo src/collision_demo.cpp)
target_link_libraries(collision_demo ${catkin_LIBRARIES} )

target_link_libraries(collision_demo
   ${catkin_LIBRARIES}
 )
