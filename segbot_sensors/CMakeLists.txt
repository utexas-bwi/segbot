# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(segbot_sensors)

find_package(catkin REQUIRED COMPONENTS laser_filters)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(segbot_sensors_filters src/segbot_sensors_filters.cpp)
target_link_libraries(segbot_sensors_filters ${Boost_LIBRARIES} ${CATKIN_LIBRARIES})

catkin_package(
    CATKIN_DEPENDS hokuyo_node laser_filters tf
)
