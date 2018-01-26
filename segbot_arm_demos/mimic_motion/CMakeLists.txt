cmake_minimum_required(VERSION 2.8.3)
project(mimic_motion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  kinova_msgs
  actionlib
  pcl_ros
  pcl_conversions
  agile_grasp
  tf_conversions
  moveit_ros_planning_interface
  moveit_utils
  moveit_msgs
  segbot_arm_manipulation
  segbot_arm_perception
)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  DEPENDS segbot_arm_perception
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(copy src/copy.cpp)
target_link_libraries(copy ${catkin_LIBRARIES} )

add_executable(push_button_demo src/push_button_demo.cpp)
target_link_libraries(push_button_demo ${catkin_LIBRARIES} )

add_executable(angular_vel_control_test src/angular_vel_control_test.cpp)
target_link_libraries(angular_vel_control_test ${catkin_LIBRARIES} )

add_executable(agile_grasp_demo src/agile_grasp_demo.cpp)
target_link_libraries(agile_grasp_demo ${catkin_LIBRARIES} )

add_executable(candy_demo src/candy_demo.cpp)
target_link_libraries(candy_demo ${catkin_LIBRARIES} )

add_executable(approach_table_demo src/approach_table_demo.cpp)
target_link_libraries(approach_table_demo ${catkin_LIBRARIES} )


add_executable(touch_tabletop_object_demo src/touch_tabletop_object_demo.cpp)
target_link_libraries(touch_tabletop_object_demo ${catkin_LIBRARIES} )

add_executable(touch_detect_tabletop_demo src/touch_detect_tabletop_demo.cpp)
target_link_libraries(touch_detect_tabletop_demo ${catkin_LIBRARIES} )

add_executable(grasp_and_verify src/grasp_and_verify.cpp)
target_link_libraries(grasp_and_verify ${catkin_LIBRARIES})


## Declare a cpp executable
#add_executable(press_object_demo src/press_object_demo.cpp)
#target_link_libraries(press_object_demo ${catkin_LIBRARIES} )

## Declare a cpp executable
#add_executable(side_grasp_demo src/side_grasp_demo.cpp)
#target_link_libraries(side_grasp_demo ${catkin_LIBRARIES} )

## Declare a cpp executable
#add_executable(top_grasp_demo src/top_grasp_demo.cpp)
#target_link_libraries(top_grasp_demo ${catkin_LIBRARIES} )

