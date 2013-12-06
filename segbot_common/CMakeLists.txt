cmake_minimum_required(VERSION 2.8.3)
project(segbot_common)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

add_message_files(
  FILES
  PlannerAtom.msg
)

add_service_files(
  FILES
  PlannerInterface.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  DEPENDS 
    opencv2
    yaml-cpp
  CATKIN_DEPENDS
    message_runtime
    std_msgs
  INCLUDE_DIRS
    include
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
  LIBRARIES
    ${PROECT_NAME}
    ${OpenCV_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/structures.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
  
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
 
