#include "kinova_msgs/ArmJointAnglesAction.h"
#include <moveit_msgs/GetPositionIK.h>

#include "segbot_arm_perception/SetObstacles.h"
#include <segbot_arm_manipulation/arm_utils.h>

#include <segbot_arm_manipulation/MicoManager.h>
#include <kinova_msgs/PoseVelocity.h>
#include <kinova_msgs/AddPoseToCartesianTrajectory.h>
#include <kinova_msgs/ClearTrajectories.h>
#include <kinova_driver/kinova_ros_types.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>
using namespace std;

MicoManager::MicoManager(ros::NodeHandle n) : pose_action(pose_action_topic, true),
                                              fingers_action(finger_action_topic, true),
                                              joint_state_action(joint_state_action_topic, true) {

    //joint positions
    joint_state_sub = n.subscribe(joint_state_topic, 1, &MicoManager::joint_state_cb, this);
    //cartesian tool position and orientation
    tool_sub = n.subscribe(tool_pose_topic, 1, &MicoManager::toolpose_cb, this);
    //finger positions
    finger_sub = n.subscribe(finger_position_topic, 1, &MicoManager::fingers_cb, this);
    home_client = n.serviceClient<kinova_msgs::HomeArm>(home_arm_service);
    safety_client = n.serviceClient<moveit_utils::MicoNavSafety>("/mico_nav_safety");
    pose_moveit_client = n.serviceClient<moveit_utils::MicoMoveitCartesianPose>("/mico_cartesianpose_service");
    joint_angles_moveit_client = n.serviceClient<moveit_utils::MicoMoveitJointPose>("/mico_jointpose_service");
    joint_pose_client_old = n.serviceClient<moveit_utils::AngularVelCtrl>("/angular_vel_control");
    ik_client = n.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
    add_waypoint_client = n.serviceClient<kinova_msgs::AddPoseToCartesianTrajectory>("/m1n6s200_driver/in/add_pose_to_Cartesian_trajectory");
    clear_waypoints_client = n.serviceClient<kinova_msgs::ClearTrajectories>("/m1n6s200_driver/in/clear_trajectories");

    angular_velocity_pub = n.advertise<kinova_msgs::JointAngles>("/m1n6s200_driver/in/joint_velocity", 10);
    cartesian_velocity_pub = n.advertise<kinova_msgs::PoseVelocity>("/m1n6s200_driver/in/cartesian_velocity", 10);

    group = new moveit::planning_interface::MoveGroupInterface("arm");
    group->setGoalTolerance(0.01);
    group->setPoseReferenceFrame("m1n6s200_end_effector");

    position_db = new ArmPositionDB(j_pos_filename, c_pos_filename);
}

MicoManager::~MicoManager(){
    delete group;
}
//Joint positions cb
void MicoManager::joint_state_cb(const sensor_msgs::JointStateConstPtr &msg) {
    if (msg->position.size() == NUM_JOINTS) {
        current_state = *msg;
        heardJointState = true;
    }
}

//tool pose cb
void MicoManager::toolpose_cb(const geometry_msgs::PoseStampedConstPtr &msg) {
    current_pose = *msg;
    heardTool = true;
}

//fingers state cb
void MicoManager::fingers_cb(const kinova_msgs::FingerPositionConstPtr &msg) {
    current_finger = *msg;
    heardFingers = true;
}

//blocking call to listen for arm data (in this case, joint states)
void MicoManager::wait_for_data() {
    heardJointState = false;
    heardTool = false;
    heardFingers = false;

    ros::Rate r(40.0);

    while (ros::ok()) {
        ros::spinOnce();

        if (heardJointState && heardTool && heardFingers)
            return;

        r.sleep();
    }
}

bool MicoManager::move_to_pose(const geometry_msgs::PoseStamped &pose) {
    kinova_msgs::ArmPoseGoal goalPose;
    goalPose.pose = pose;

    pose_action.waitForServer();

    //finally, send goal and wait
    pose_action.sendGoal(goalPose);
    return pose_action.waitForResult();

}

kinova_msgs::JointAngles state_to_angles(const sensor_msgs::JointState &state) {
    //check if this is specified just for the arm
    sensor_msgs::JointState just_arm;
    if (state.position.size() > NUM_JOINTS) {
        //in this case, the first four values are for the base joints
        for (int i = 4; i < state.position.size(); i++) {
            just_arm.position.push_back(state.position.at(i));
            just_arm.name.push_back(state.name.at(i));
        }
        just_arm.header = state.header;
    } else {
        just_arm = state;
    }
    kinova_msgs::JointAngles as_angles;

    for (int i = 0; i < NUM_JOINTS; i++) {
        as_angles.joint1 = just_arm.position.at(0);
        as_angles.joint2 = just_arm.position.at(1);
        as_angles.joint3 = just_arm.position.at(2);
        as_angles.joint4 = just_arm.position.at(3);
        as_angles.joint5 = just_arm.position.at(4);
        as_angles.joint6 = just_arm.position.at(5);
    }
    return as_angles;
}




bool MicoManager::move_to_joint_state(const sensor_msgs::JointState &target) {
    return move_to_joint_state(state_to_angles(target));
}


bool MicoManager::move_to_joint_state(const kinova_msgs::JointAngles &target) {
    kinova_msgs::ArmJointAnglesGoal goal;
    goal.angles;

    joint_state_action.waitForServer();

    //finally, send goal and wait
    joint_state_action.sendGoal(goal);
    return joint_state_action.waitForResult();

}

bool MicoManager::move_to_joint_state_old(const sensor_msgs::JointState &target) {
    //check if this is specified just for the arm
    sensor_msgs::JointState q_target;
    if (target.position.size() > NUM_JOINTS) {
        //in this case, the first four values are for the base joints
        for (int i = 4; i < target.position.size(); i++) {
            q_target.position.push_back(target.position.at(i));
            q_target.name.push_back(target.name.at(i));
        }
        q_target.header = target.header;
    } else {
        q_target = target;
    }


    moveit_utils::AngularVelCtrl::Request req;
    moveit_utils::AngularVelCtrl::Response resp;

    req.state = q_target;

    if (joint_angles_moveit_client.call(req, resp)) {
        ROS_INFO("Call successful. Response:");
        return resp.success;
    }
    return false;

}

bool MicoManager::move_fingers(const int finger1_value, const int finger2_value) {

    kinova_msgs::SetFingersPositionGoal goalFinger;
    goalFinger.fingers.finger1 = finger1_value;
    goalFinger.fingers.finger2 = finger2_value;
    // Not used for our arm
    goalFinger.fingers.finger3 = 0;

    fingers_action.waitForServer();
    fingers_action.sendGoal(goalFinger);
    return fingers_action.waitForResult();
}

bool MicoManager::move_fingers(int finger_value) {
    return move_fingers(finger_value, finger_value);
}


bool MicoManager::make_safe_for_travel() {
    safety_client.waitForExistence();
    moveit_utils::MicoNavSafety srv_safety;
    srv_safety.request.getSafe = true;

    if (safety_client.call(srv_safety)) {
        return srv_safety.response.safe;
    } else {
        return false;
    }
}

bool MicoManager::move_home() {
    kinova_msgs::HomeArm srv;
    return home_client.call(srv);
}


bool MicoManager::open_hand() {
    return move_fingers(OPEN_FINGER_VALUE);
}

bool MicoManager::close_hand() {
    return move_fingers(CLOSED_FINGER_VALUE);
}

moveit_msgs::GetPositionIK::Response MicoManager::compute_ik(const geometry_msgs::PoseStamped &p) {

    moveit_msgs::GetPositionIK::Request ik_request;
    moveit_msgs::GetPositionIK::Response ik_response;
    ik_request.ik_request.group_name = "arm";
    ik_request.ik_request.pose_stamped = p;

    /* Call the service */
    ik_client.call(ik_request, ik_response);
    return ik_response;

}


bool MicoManager::move_to_pose_moveit(const geometry_msgs::PoseStamped &target,
                                      const vector<sensor_msgs::PointCloud2> &obstacles,
                                      const moveit_msgs::Constraints &constraints
) {
    moveit_utils::MicoMoveitCartesianPose::Request req;
    moveit_utils::MicoMoveitCartesianPose::Response res;

    vector<moveit_msgs::CollisionObject> moveit_obstacles = segbot_arm_manipulation::get_collision_boxes(obstacles);

    req.target = target;
    req.collision_objects = moveit_obstacles;
    req.constraints = constraints;
    return pose_moveit_client.call(req, res);

}

bool MicoManager::move_to_joint_state_moveit(const kinova_msgs::JointAngles &target,
                                             const vector<sensor_msgs::PointCloud2> &obstacles,
                                             const moveit_msgs::Constraints &constraints) {
    moveit_utils::MicoMoveitJointPose::Request req;
    moveit_utils::MicoMoveitJointPose::Response res;

    vector<moveit_msgs::CollisionObject> moveit_obstacles = segbot_arm_manipulation::get_collision_boxes(obstacles);

    req.target = target;
    req.collision_objects = moveit_obstacles;
    req.constraints = constraints;
    return joint_angles_moveit_client.call(req, res);
}

bool MicoManager::move_to_joint_state_moveit(const sensor_msgs::JointState &target,
                                             const vector<sensor_msgs::PointCloud2> &obstacles,
                                             const moveit_msgs::Constraints &constraints) {

    return move_to_joint_state_moveit(state_to_angles(target), obstacles, constraints);
}


bool MicoManager::move_to_side_view() {
    if (!position_db->hasCarteseanPosition("side_view")) {
        return false;
    }
    geometry_msgs::PoseStamped out_of_view_pose = position_db->getToolPositionStamped("side_view",
                                                                                      "/m1n6s200_link_base");
    return move_to_pose_moveit(out_of_view_pose);

}


bool MicoManager::move_to_handover() {
    if (!position_db->hasCarteseanPosition("handover_front")) {
        return false;
    }
    geometry_msgs::PoseStamped handover_pose = position_db->getToolPositionStamped("handover_front",
                                                                                   "m1n6s200_link_base");

    return move_to_pose_moveit(handover_pose);


}

void MicoManager::move_with_angular_velocities(const kinova_msgs::JointAngles &velocities, const double seconds) {
    // Per the Kinova documentation, we have to publish at exactly 100Hz to get
    // predictable behavior
    ros::Rate r(100);


    ros::Time end = ros::Time::now() + ros::Duration(seconds);
    while (ros::ok()) {
        //collect messages
        ros::spinOnce();

        //publish velocity message
        angular_velocity_pub.publish(velocities);
        r.sleep();
        if (ros::Time::now() > end) {
            break;
        }
    }
    kinova_msgs::JointAngles zeros;
    angular_velocity_pub.publish(zeros);

}

void MicoManager::move_with_cartesian_velocities(const kinova_msgs::PoseVelocity &velocities, const double seconds) {
    // Per the Kinova documentation, we have to publish at exactly 100Hz to get
    // predictable behavior
    ros::Rate r(100);

    ros::Time end = ros::Time::now() + ros::Duration(seconds);
    while (ros::ok()) {
        //collect messages
        ros::spinOnce();

        //publish velocity message
        cartesian_velocity_pub.publish(velocities);
        r.sleep();
        if (ros::Time::now() > end) {
            break;
        }
    }
    kinova_msgs::PoseVelocity zeros;
    cartesian_velocity_pub.publish(zeros);

}

bool MicoManager::move_through_waypoints(const vector<geometry_msgs::Pose> &waypoints) {
    for (int i = 0; i < waypoints.size(); i++) {
        kinova_msgs::KinovaPose next = kinova::KinovaPose(waypoints.at(i)).constructKinovaPoseMsg();
        kinova_msgs::AddPoseToCartesianTrajectory::Request req;
        kinova_msgs::AddPoseToCartesianTrajectory::Response res;
        req.ThetaX = next.ThetaX;
        req.ThetaY = next.ThetaY;
        req.ThetaZ = next.ThetaZ;
        req.X = next.X;
        req.Y = next.Y;
        req.Z = next.Z;
        bool success = add_waypoint_client.call(req, res);
        if (!success) {
            return false;
        }
    }
    return true;
}

bool MicoManager::move_through_waypoints_moveit(const vector<geometry_msgs::Pose> &waypoints,
                                                const vector<sensor_msgs::PointCloud2> &obstacles,
                                                const moveit_msgs::Constraints &constraints) {

    for (int i = 0; i < waypoints.size(); ++i) {
        ROS_INFO_STREAM(waypoints.at(i));
    }

    vector<moveit_msgs::CollisionObject> moveit_obstacles = segbot_arm_manipulation::get_collision_boxes(obstacles);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //planning_scene_interface.addCollisionObjects(moveit_obstacles);
    group->setPlanningTime(5.0);
    group->setPoseTarget(waypoints.at(waypoints.size() - 1));
    group->setStartState(*group->getCurrentState());
    //group->setPathConstraints(constraints);

    moveit_msgs::RobotTrajectory result;
    double fraction = group->computeCartesianPath(waypoints, 0.01, 0,result);

    // We'll only execute if the full trajectory was generated
    if (fraction < 1) {
        return false;
    }

    robot_trajectory::RobotTrajectory rt(group->getCurrentState()->getRobotModel(), "arm");
    rt.setRobotTrajectoryMsg(*group->getCurrentState(), result);

    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    // Fourth compute computeTimeStamps
    bool success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(result);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    moveit_msgs::RobotState start_state;
    moveit::core::robotStateToRobotStateMsg(*group->getCurrentState(), start_state);
    my_plan.start_state_= start_state;
    my_plan.trajectory_= result;
    moveit::planning_interface::MoveItErrorCode error = group->execute(my_plan);
    return error == moveit::planning_interface::MoveItErrorCode::SUCCESS;
}

