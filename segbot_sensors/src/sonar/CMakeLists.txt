# Conversions for sensor_msgs/Range messages
# used for segbot version 2
add_library(range_to_cloud range_nodelet.cpp range_to_cloud.cpp)
target_link_libraries(range_to_cloud ${catkin_LIBRARIES})
install (TARGETS range_to_cloud
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
