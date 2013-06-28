# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(segbot_sensors)

find_package(catkin REQUIRED COMPONENTS laser_filters)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(segbot_sensors_filters src/segbot_sensors_filters.cpp)
target_link_libraries(segbot_sensors_filters 
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
)

catkin_package(
  CATKIN_DEPENDS hokuyo_node laser_filters tf
)

# Install the plugin and the plugin description
install (TARGETS segbot_sensors_filters
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES segbot_sensors_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install runtime launch and configuration files
foreach(dir launch config rviz)
   install(DIRECTORY ${dir}/ 
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
