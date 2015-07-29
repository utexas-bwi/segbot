# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(segbot_navigation)

find_package(catkin REQUIRED COMPONENTS bwi_interruptable_action_server geometry_msgs move_base_msgs nav_msgs std_srvs)
catkin_package(
  CATKIN_DEPENDS bwi_interruptable_action_server geometry_msgs move_base_msgs nav_msgs std_srvs
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(move_base_interruptable_server src/move_base_interruptable_server.cpp)
target_link_libraries(move_base_interruptable_server
  ${catkin_LIBRARIES}
)

install(TARGETS move_base_interruptable_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch rviz)
  install(DIRECTORY ${dir}/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

catkin_install_python(PROGRAMS scripts/move_base_interruptable_simple scripts/rviz_runner
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install the teleoperation script for rosrun 
install(PROGRAMS 
  scripts/rviz_runner
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# unit tests are enabled selectively
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/amcl.launch)
  roslaunch_add_file_check(launch/gmapping.launch)
  roslaunch_add_file_check(launch/move_base_eband.launch)
  roslaunch_add_file_check(launch/rviz.launch)
  ## launch/navigation.launch not tested until
  ## ros-drivers/openni_launch#10 is fixed.
endif ()
