cmake_minimum_required(VERSION 2.8.3)
project(segbot_arm_manipulation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        actionlib
        actionlib_msgs
        agile_grasp
        geometry_msgs
        kinova_driver
        kinova_msgs
        message_generation
        move_base_msgs
        moveit_ros_planning_interface
        moveit_utils
        moveit_msgs
        nav_msgs
        pcl_ros
        pcl_conversions
        roscpp
        rospy
        std_msgs
        sensor_msgs
        sound_play
        segbot_arm_perception
        tf_conversions
        )


find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

add_action_files(
        FILES
        TabletopGrasp.action
        TabletopApproach.action
        LiftVerify.action
        Press.action
        Push.action
        Shake.action
        ObjReplacement.action
)

add_service_files(
        FILES
        iSpyTouch.srv
        iSpyDetectTouch.srv
        iSpyFaceTable.srv
)

generate_messages(
        DEPENDENCIES
        sensor_msgs
        actionlib_msgs
        geometry_msgs
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs kinova_msgs kinova_driver message_runtime moveit_msgs roscpp rospy std_msgs
        INCLUDE_DIRS include
        LIBRARIES arm_utilities
)


add_executable(segbot_tabletop_grasp_as src/segbot_tabletop_grasp_as.cpp)
add_dependencies(segbot_tabletop_grasp_as   segbot_arm_manipulation_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(segbot_tabletop_grasp_as ${catkin_LIBRARIES})

add_executable(arm_lift_verify_as src/arm_lift_verify_as.cpp)
target_link_libraries(arm_lift_verify_as ${catkin_LIBRARIES})
add_dependencies(arm_lift_verify_as  segbot_arm_manipulation_gencpp ${catkin_EXPORTED_TARGETS})

add_executable(arm_press_as src/arm_press_as.cpp)
add_dependencies(arm_press_as  segbot_arm_manipulation_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_press_as ${catkin_LIBRARIES})

add_executable(arm_push_as src/arm_push_as.cpp)
add_dependencies(arm_push_as  segbot_arm_manipulation_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_push_as ${catkin_LIBRARIES})

add_executable(arm_shake_as src/arm_shake_as.cpp)
add_dependencies(arm_shake_as  segbot_arm_manipulation_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_shake_as ${catkin_LIBRARIES})

add_executable(segbot_table_approach_as src/segbot_table_approach_as.cpp)
target_link_libraries(segbot_table_approach_as ${catkin_LIBRARIES})
add_dependencies(segbot_table_approach_as segbot_arm_manipulation_gencpp)

add_executable(ispy_arm_server src/ispy_arm_server.cpp)
target_link_libraries(ispy_arm_server ${catkin_LIBRARIES})
add_dependencies(ispy_arm_server  segbot_arm_manipulation_gencpp)

add_executable(demo_obstacle_avoidance src/demos/demo_obstacle_avoidance.cpp)
target_link_libraries(demo_obstacle_avoidance ${catkin_LIBRARIES})

add_executable(demo_grasp_action_client src/demos/demo_grasp_action_client.cpp)
target_link_libraries(demo_grasp_action_client ${catkin_LIBRARIES} )
add_dependencies(demo_grasp_action_client segbot_arm_manipulation_gencpp)

add_executable(demo_grasp_verify src/demos/demo_grasp_verify.cpp)
target_link_libraries(demo_grasp_verify ${catkin_LIBRARIES} )
add_dependencies(demo_grasp_verify segbot_arm_manipulation_gencpp)

add_executable(demo_explore_object src/demos/demo_explore_object.cpp)
target_link_libraries(demo_explore_object ${catkin_LIBRARIES})
add_dependencies(demo_explore_object  segbot_arm_manipulation_gencpp ${catkin_EXPORTED_TARGETS})

add_executable(test_table_approach_actions src/demos/test_table_approach_actions.cpp)
target_link_libraries(test_table_approach_actions ${catkin_LIBRARIES} )
add_dependencies(test_table_approach_actions segbot_arm_manipulation_gencpp)

add_executable(demo_hand_lead src/demos/demo_hand_lead.cpp)
target_link_libraries(demo_hand_lead ${catkin_LIBRARIES} )
add_dependencies(demo_hand_lead segbot_arm_manipulation_gencpp)

add_executable(test_grasp_replacement src/demos/test_grasp_replacement.cpp)
target_link_libraries(test_grasp_replacement ${catkin_LIBRARIES} )
add_dependencies(test_grasp_replacement segbot_arm_manipulation_gencpp)

add_executable(segbot_obj_replacement_as src/segbot_obj_replacement_as.cpp)
add_dependencies(segbot_obj_replacement_as segbot_arm_manipulation_gencpp)
target_link_libraries(segbot_obj_replacement_as ${catkin_LIBRARIES} )

add_executable(test_handover_replacement src/demos/test_handover_replacement.cpp)
target_link_libraries(test_handover_replacement ${catkin_LIBRARIES} )
add_dependencies(test_handover_replacement segbot_arm_manipulation_gencpp)

install(PROGRAMS
        scripts/arm_actions.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(test_ispy src/demos/test_ispy.cpp)
target_link_libraries(test_ispy ${catkin_LIBRARIES})
add_dependencies(test_ispy segbot_arm_manipulation_gencpp)

add_executable(demo_press_button src/demos/demo_press_button.cpp)
target_link_libraries(demo_press_button ${catkin_LIBRARIES})
add_dependencies(demo_press_button  segbot_arm_manipulation_gencpp)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

add_library(arm_utilities src/MicoManager.cpp)
target_link_libraries(arm_utilities ${catkin_LIBRARIES})