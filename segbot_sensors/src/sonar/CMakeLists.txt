# Conversions for segbot_sensors/RangesArray messages
# ONLY used for segbot version 1
add_library(ranges_to_cloud ranges_nodelet.cpp ranges_to_cloud.cpp)
add_dependencies(ranges_to_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(ranges_to_cloud ${catkin_LIBRARIES})
install (TARGETS ranges_to_cloud
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Conversions for sensor_msgs/Range messages
# used for segbot version 2
add_library(range_to_cloud range_nodelet.cpp range_to_cloud.cpp)
add_dependencies(range_to_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(range_to_cloud ${catkin_LIBRARIES})
install (TARGETS range_to_cloud
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
