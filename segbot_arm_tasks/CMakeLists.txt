cmake_minimum_required(VERSION 2.8.3)
project(segbot_arm_tasks)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    bwi_msgs
    bwi_kr_execution
    bwi_services
    kinova_msgs
    pcl_ros
    pcl_conversions
    roscpp
    rospy
    segbot_arm_perception
    segbot_arm_manipulation
    std_msgs
    move_base_msgs
    moveit_ros_planning_interface
    moveit_utils
    moveit_msgs
    nav_msgs

)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES segbot_arm_tasks
#  CATKIN_DEPENDS kinova_msgs moveit_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
   CATKIN_DEPENDS actionlib_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## example task 
add_executable(object_to_office_task src/object_to_office_task.cpp)
target_link_libraries(object_to_office_task ${catkin_LIBRARIES} )

## example task 
add_executable(object_handover_delivery_task src/object_handover_delivery_task.cpp)
target_link_libraries(object_handover_delivery_task ${catkin_LIBRARIES} )

add_executable(object_table_delivery_task src/object_table_delivery_task.cpp)
target_link_libraries(object_table_delivery_task ${catkin_LIBRARIES} )
