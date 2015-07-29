cmake_minimum_required(VERSION 2.8.3)
project(segbot_simulation_apps)

find_package(catkin REQUIRED COMPONENTS 
  bwi_msgs
  bwi_planning_common
  bwi_tools
  gazebo_msgs
  geometry_msgs
  multi_level_map_msgs
  roscpp
  std_msgs
  tf
)
find_package(Boost REQUIRED)

catkin_package(
  DEPENDS
    Boost
  CATKIN_DEPENDS
    bwi_msgs
    bwi_planning_common
    bwi_tools
    gazebo_msgs
    geometry_msgs
    multi_level_map_msgs
    roscpp
    std_msgs
    tf
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/libsegbot_simulation_apps/door_handler.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(door_handler
  src/nodes/door_handler.cpp
)
target_link_libraries(door_handler
  ${PROJECT_NAME}
)
add_dependencies(door_handler segbot_simulation_apps_gencpp)

install(TARGETS ${PROJECT_NAME} door_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
 
# Install runtime launch and configuration files
foreach(dir launch urdf)
  install(DIRECTORY ${dir}/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

# unit tests are enabled selectively
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif ()
