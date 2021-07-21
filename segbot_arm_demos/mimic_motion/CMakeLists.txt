cmake_minimum_required(VERSION 2.8.3)
project(mimic_motion)

add_compile_options(-std=c++11)

if( $ENV{SEGBOT_HAS_ARM} )
find_package(catkin REQUIRED COMPONENTS
        agile_grasp
  bwi_moveit_utils
  bwi_perception
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  kinova_msgs
  actionlib
  pcl_ros
  pcl_conversions
  tf_conversions
  moveit_ros_planning_interface
  moveit_msgs
  segbot_arm_manipulation

)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
        DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(push_button_demo src/push_button_demo.cpp)
target_link_libraries(push_button_demo ${catkin_LIBRARIES} )

add_executable(candy_demo src/candy_demo.cpp)
target_link_libraries(candy_demo ${catkin_LIBRARIES} )

add_executable(touch_tabletop_object_demo src/touch_tabletop_object_demo.cpp)
target_link_libraries(touch_tabletop_object_demo ${catkin_LIBRARIES} )

add_executable(touch_detect_tabletop_demo src/touch_detect_tabletop_demo.cpp)
target_link_libraries(touch_detect_tabletop_demo ${catkin_LIBRARIES} )
endif()
