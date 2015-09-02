cmake_minimum_required(VERSION 2.8.3)
project(segbot_logical_translator)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    bwi_mapper
    bwi_msgs
    bwi_planning_common
    map_msgs
    message_filters
    move_base_msgs
    multi_level_map_msgs
    multi_level_map_utils
    nav_msgs
    roscpp
    tf
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    bwi_mapper
    bwi_msgs
    bwi_planning_common
    map_msgs
    message_filters
    move_base_msgs
    multi_level_map_msgs
    multi_level_map_utils
    nav_msgs
    roscpp
    tf
  INCLUDE_DIRS
    include
  LIBRARIES
    segbot_logical_translator
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(segbot_logical_translator
  src/libsegbot_logical_translator/segbot_logical_translator.cpp)
target_link_libraries(segbot_logical_translator
  ${catkin_LIBRARIES}
)

add_executable(segbot_logical_navigator src/nodes/segbot_logical_navigator.cpp)
target_link_libraries(segbot_logical_navigator
  ${catkin_LIBRARIES}
  segbot_logical_translator)
add_dependencies(segbot_logical_navigator segbot_logical_translator_generate_messages_cpp)

install(TARGETS segbot_logical_translator segbot_logical_navigator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  include/segbot_logical_translator/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

