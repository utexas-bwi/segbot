cmake_minimum_required(VERSION 2.8.3)
project(push_button)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  message_generation
)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(push_button src/push_button.cpp)
target_link_libraries(   push_button ${CATKIN_LIBRARIES} ${PCL_LIBRARIES} )

add_executable(analyse_red_button src/analyze_red_button.cpp)
target_link_libraries(   analyse_red_button ${CATKIN_LIBRARIES} ${PCL_LIBRARIES} )
