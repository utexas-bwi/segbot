# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(segbot_sensors)

find_package(catkin REQUIRED
  COMPONENTS
    diagnostic_msgs
    geometry_msgs
    laser_filters
    message_generation
    nodelet
    pcl_ros
    pluginlib
    roscpp
    roslint
    sensor_msgs
    smart_battery_msgs
    std_msgs
    tf)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    diagnostic_msgs
    geometry_msgs
    laser_filters
    message_runtime
    nodelet
    pcl_ros
    sensor_msgs
    smart_battery_msgs
    std_msgs
    tf
  INCLUDE_DIRS
    include
  LIBRARIES
    segbot_sensors_filters
    range_to_cloud
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_subdirectory(src/sonar)

add_library(segbot_sensors_filters src/segbot_sensors_filters.cpp)
target_link_libraries(segbot_sensors_filters 
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
)

add_executable(battery_diagnostics src/segbot_sensors/battery_diagnostics.cpp)
target_link_libraries(battery_diagnostics ${catkin_LIBRARIES})

add_executable(fake_volt_pub src/segbot_sensors/fake_volt_pub.cpp)
target_link_libraries(fake_volt_pub ${catkin_LIBRARIES})

# Install the plugin and the plugin description
install (DIRECTORY include/segbot_sensors/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install (TARGETS segbot_sensors_filters battery_diagnostics fake_volt_pub
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

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
  roslaunch_add_file_check(launch/diagnostic)
  roslaunch_add_file_check(launch/hokuyo)
  roslaunch_add_file_check(launch/kinect)
  roslaunch_add_file_check(launch/ptgrey)
endif()
