# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(segbot_sensors)

find_package(catkin REQUIRED
  COMPONENTS
    laser_filters
    message_generation
    nodelet
    pcl_ros
    pluginlib
    roscpp
    roslint
    sensor_msgs
    tf
    std_msgs
    diagnostic_msgs
    smart_battery_msgs
    roscpp)

catkin_python_setup()

# define RangeArray message (ONLY used for segbot version 1)
add_message_files(DIRECTORY msg FILES RangeArray.msg)
generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(
  CATKIN_DEPENDS
    hokuyo_node
    laser_filters
    message_runtime
    nodelet
    pcl_ros
    sensor_msgs
    tf
  INCLUDE_DIRS
    include
  LIBRARIES
    segbot_sensors_filters
    range_to_cloud
    ranges_to_cloud
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_subdirectory(src/sonar)

add_library(segbot_sensors_filters src/segbot_sensors_filters.cpp)
target_link_libraries(segbot_sensors_filters 
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
)

# Install the plugin and the plugin description
install (DIRECTORY include/segbot_sensors/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install (TARGETS segbot_sensors_filters
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES nodelets.xml segbot_sensors_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install executable Python nodes
install(DIRECTORY nodes/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS)

# Install runtime launch and configuration files
foreach(dir config launch rviz)
  install(DIRECTORY ${dir}/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

# Lint Python modules for PEP8 compatibility
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SCRIPTS
     RELATIVE ${PROJECT_SOURCE_DIR} nodes/* src/${PROJECT_NAME}/*.py)
roslint_python(${${PROJECT_NAME}_PY_SCRIPTS})

# unit tests are enabled selectively
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
  roslaunch_add_file_check(launch/arduino)
  roslaunch_add_file_check(launch/hokuyo)
  roslaunch_add_file_check(launch/kinect)
  roslaunch_add_file_check(launch/ptgrey)
endif()

add_executable(batteryLife src/segbot_sensors/battery_monitor.cpp)
target_link_libraries(batteryLife ${catkin_LIBRARIES})
