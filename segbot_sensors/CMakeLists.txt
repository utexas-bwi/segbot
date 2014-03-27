# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(segbot_sensors)

find_package(catkin REQUIRED
  COMPONENTS laser_filters message_generation pcl_ros roslint sensor_msgs tf)

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
    ranges_to_cloud
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(ranges_to_cloud src/ranges_nodelet.cpp src/ranges_to_cloud.cpp)
add_dependencies(ranges_to_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(ranges_to_cloud ${catkin_LIBRARIES})

add_library(segbot_sensors_filters src/segbot_sensors_filters.cpp)
target_link_libraries(segbot_sensors_filters 
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
)

# Install the plugin and the plugin description
install (DIRECTORY include/segbot_sensors/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install (TARGETS ranges_to_cloud segbot_sensors_filters
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES segbot_sensors_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install executable Python scripts
install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS)

# Install runtime launch and configuration files
foreach(dir config launch rviz)
  install(DIRECTORY ${dir}/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Lint Python modules for PEP8 compatibility
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SCRIPTS
     RELATIVE ${PROJECT_SOURCE_DIR} scripts/*)
roslint_python(${${PROJECT_NAME}_PY_SCRIPTS})

# unit tests are enabled selectively
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
  roslaunch_add_file_check(launch/hokuyo)
  roslaunch_add_file_check(launch/ptgrey)

  ## launch/kinect/kinect.launch fails, because of
  ## ros-drivers/openni_launch#10.
  ##
  ## Test the individual files that do work, restore testing of them
  ## all when openni_launch has been fixed.
  #roslaunch_add_file_check(launch/kinect)
  roslaunch_add_file_check(launch/kinect/kinect-filters.launch)
  roslaunch_add_file_check(launch/kinect/kinect-laserscan.launch)

endif (CATKIN_ENABLE_TESTING)
