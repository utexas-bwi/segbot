cmake_minimum_required(VERSION 2.8.3)
project(segbot_arm_tutorials)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    actionlib
    kinova_msgs
    moveit_msgs
    moveit_ros_planning_interface
    moveit_utils
    moveit_msgs
    nav_msgs
    move_base_msgs
    pcl_ros
    pcl_conversions
    roscpp
    rospy
    segbot_arm_perception
    segbot_arm_manipulation
    std_msgs
)

catkin_package(
   CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ex1_subscribing_to_topics src/ex1_subscribing_to_topics.cpp)
target_link_libraries(ex1_subscribing_to_topics ${catkin_LIBRARIES} )

add_executable(ex2_gripper src/ex2_gripper_control.cpp)
target_link_libraries(ex2_gripper ${catkin_LIBRARIES} )

add_executable(ex3_home_arm src/ex3_home_arm.cpp)
target_link_libraries(ex3_home_arm ${catkin_LIBRARIES} )

add_executable(ex4_cartesian_vel_control src/ex4_cartesian_vel_control.cpp)
target_link_libraries(ex4_cartesian_vel_control ${catkin_LIBRARIES} )

add_executable(ex5_angular_vel_control src/ex5_angular_vel_control.cpp)
target_link_libraries(ex5_angular_vel_control ${catkin_LIBRARIES} )

add_executable(ex6_detect_force src/ex6_detect_force.cpp)
target_link_libraries(ex6_detect_force ${catkin_LIBRARIES} )

add_executable(ex7_cartesian_pose_control src/ex7_cartesian_pose_control.cpp)
target_link_libraries(ex7_cartesian_pose_control ${catkin_LIBRARIES} )

add_executable(ex8_joint_angles_control src/ex8_joint_angles_control.cpp)
target_link_libraries(ex8_joint_angles_control ${catkin_LIBRARIES} )

add_executable(ex9_waypoints src/ex9_waypoints.cpp)
target_link_libraries(ex9_waypoints ${catkin_LIBRARIES} )

add_executable(ex10_advanced_moveit src/ex10_advanced_moveit.cpp)
target_link_libraries(ex10_advanced_moveit ${catkin_LIBRARIES} )
