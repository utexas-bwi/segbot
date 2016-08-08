cmake_minimum_required(VERSION 2.8.3)
project(segbot_led)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  bwi_msgs
  roscpp
  serial
  std_msgs
)

add_definitions(-std=c++11)

catkin_package(
  INCLUDE_DIRS ledcomm
  CATKIN_DEPENDS actionlib_msgs bwi_msgs roscpp serial std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS} ledcomm
)

add_executable(led_control_server src/led_control_server.cpp)
add_dependencies(led_control_server ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(led_control_server ${catkin_LIBRARIES})

install(TARGETS
          led_control_server
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})