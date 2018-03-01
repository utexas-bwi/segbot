#include "kinova_msgs/ArmJointAnglesAction.h"
#include <moveit_msgs/GetPositionIK.h>

#include "bwi_perception/SetObstacles.h"
#include <segbot_arm_manipulation/arm_utils.h>

#include <segbot_arm_manipulation/Mico.h>
#include <kinova_msgs/PoseVelocity.h>
#include <kinova_msgs/AddPoseToCartesianTrajectory.h>
#include <kinova_msgs/ClearTrajectories.h>
#include <kinova_driver/kinova_ros_types.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>
#include <geometry_msgs/WrenchStamped.h>

#include <segbot_arm_manipulation/NavSafety.h>
#include <bwi_moveit_utils/MoveitCartesianPose.h>
#include <bwi_moveit_utils/MoveitJointPose.h>
#include <bwi_moveit_utils/MoveitWaypoint.h>
#include <moveit_msgs/GetPositionFK.h>

using namespace std;


namespace segbot_arm_manipulation {

    const string Mico::jointNames[] = {"m1n6s200_joint_1", "m1n6s200_joint_2", "m1n6s200_joint_3", "m1n6s200_joint_4",
                                       "m1n6s200_joint_5", "m1n6s200_joint_6", "m1n6s200_joint_finger_1",
                                       "m1n6s200_joint_finger_2"};
    const string Mico::finger_action_topic = "/m1n6s200_driver/fingers_action/finger_positions";
    const string Mico::pose_action_topic = "/m1n6s200_driver/pose_action/tool_pose";
    const string Mico::joint_state_action_topic = "/m1n6s200_driver/joints_action/joint_angles";
    const string Mico::joint_state_topic = "/m1n6s200_driver/out/joint_state";
    const string Mico::tool_pose_topic = "/m1n6s200_driver/out/tool_pose";
    const string Mico::finger_position_topic = "/m1n6s200_driver/out/finger_position";
    const string Mico::home_arm_service = "/m1n6s200_driver/in/home_arm";


    const string Mico::j_pos_filename =
            ros::package::getPath("segbot_arm_manipulation") + "/data/jointspace_position_db.txt";
    const string Mico::c_pos_filename =
            ros::package::getPath("segbot_arm_manipulation") + "/data/toolspace_position_db.txt";
    const double Mico::arm_poll_rate = 100.0;

    Mico::Mico(ros::NodeHandle n) : pose_action(pose_action_topic, true),
                                    fingers_action(finger_action_topic, true),
                                    joint_state_action(joint_state_action_topic, true) {

        //joint positions
        joint_state_sub = n.subscribe(joint_state_topic, 1, &Mico::joint_state_cb, this);
        //cartesian tool position and orientation
        tool_sub = n.subscribe(tool_pose_topic, 1, &Mico::toolpose_cb, this);
        //finger positions
        finger_sub = n.subscribe(finger_position_topic, 1, &Mico::fingers_cb, this);
        home_client = n.serviceClient<kinova_msgs::HomeArm>(home_arm_service);
        safety_client = n.serviceClient<segbot_arm_manipulation::NavSafety>("/make_safe_for_travel");
        pose_moveit_client = n.serviceClient<bwi_moveit_utils::MoveitCartesianPose>("/cartesian_pose_service");
        joint_angles_moveit_client = n.serviceClient<bwi_moveit_utils::MoveitJointPose>("/joint_pose_service");
        waypoint_moveit_client = n.serviceClient<bwi_moveit_utils::MoveitWaypoint>("/waypoint_service");
        wrench_sub = n.subscribe("/m1n6s200_driver/out/tool_wrench", 1, &Mico::wrench_cb, this);
        ik_client = n.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
        fk_client = n.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
        add_waypoint_client = n.serviceClient<kinova_msgs::AddPoseToCartesianTrajectory>(
                "/m1n6s200_driver/in/add_pose_to_Cartesian_trajectory");
        clear_waypoints_client = n.serviceClient<kinova_msgs::ClearTrajectories>(
                "/m1n6s200_driver/in/clear_trajectories");

        angular_velocity_pub = n.advertise<kinova_msgs::JointAngles>("/m1n6s200_driver/in/joint_velocity", 10);
        cartesian_velocity_pub = n.advertise<kinova_msgs::PoseVelocity>("/m1n6s200_driver/in/cartesian_velocity", 10);

        position_db = new ArmPositionDB(j_pos_filename, c_pos_filename);
    }

//Joint positions cb
    void Mico::joint_state_cb(const sensor_msgs::JointStateConstPtr &msg) {
        current_state = *msg;
        heard_joint_state = true;
    }

//tool pose cb
    void Mico::toolpose_cb(const geometry_msgs::PoseStampedConstPtr &msg) {
        current_pose = *msg;
        heard_tool = true;
    }

//fingers state cb
    void Mico::fingers_cb(const kinova_msgs::FingerPositionConstPtr &msg) {
        current_finger = *msg;
        heard_fingers = true;
    }

    void Mico::wrench_cb(const geometry_msgs::WrenchStampedConstPtr &msg) {
        current_wrench = *msg;
        heard_wrench = true;
    }

//blocking call to listen for arm data (in this case, joint states)
    bool Mico::wait_for_data(double timeout) {
        heard_joint_state = false;
        heard_tool = false;
        heard_fingers = false;
        heard_wrench = false;

        ros::Rate r(arm_poll_rate);

        // Negative timeout is interpreted as no timeout
        bool use_timeout = timeout > 0.0;

        ros::Time end = ros::Time::now() + ros::Duration(timeout);
        while (ros::ok()) {
            ros::spinOnce();

            if (heard_joint_state && heard_tool && heard_fingers && heard_wrench)
                return true;

            r.sleep();

            if (use_timeout && ros::Time::now() > end) {
                return false;
            }
        }
    }

/*
   * blocks until force of sufficient amount is detected or timeout is exceeded
   * returns true of force degected, false if timeout
   */
    bool Mico::wait_for_force(const double force_threshold, const double timeout) {

        double total_delta;
        double delta_effort[6];

        // Set the stamp on entry. wait_for_force must take no longer than timeout seconds
        ros::Time end = ros::Time::now() + ros::Duration(timeout);
        bool got_data = wait_for_data(timeout);
        if (!got_data) {
            return got_data;
        }
        sensor_msgs::JointState prev_effort_state = current_state;


        ros::Rate r(arm_poll_rate);

        bool use_timeout = timeout > 0;

        while (ros::ok()) {
            //collect messages
            ros::spinOnce();

            total_delta = 0.0;

            for (int i = 0; i < 6; i++) {
                delta_effort[i] = fabs(current_state.effort[i] - prev_effort_state.effort[i]);
                total_delta += delta_effort[i];
                //ROS_INFO("Total delta=%f",total_delta);
            }

            if (total_delta > fabs(force_threshold)) {
                return true;
            }

            r.sleep();

            if (use_timeout && ros::Time::now() > end) {
                return false;
            }
        }
    }

    bool Mico::move_to_pose(const geometry_msgs::PoseStamped &pose) {
        kinova_msgs::ArmPoseGoal goalPose;
        goalPose.pose = pose;

        pose_action.waitForServer();

        //finally, send goal and wait
        pose_action.sendGoal(goalPose);
        return pose_action.waitForResult();

    }

    bool Mico::move_to_joint_state(const sensor_msgs::JointState &target) {
        return move_to_joint_state(segbot_arm_manipulation::state_to_angles(target));
    }


    bool Mico::move_to_joint_state(const kinova_msgs::JointAngles &target) {
        kinova_msgs::ArmJointAnglesGoal goal;
        goal.angles = target;

        joint_state_action.waitForServer();

        //finally, send goal and wait
        joint_state_action.sendGoal(goal);
        return joint_state_action.waitForResult();

    }

    bool Mico::move_fingers(const int finger1_value, const int finger2_value) {

        kinova_msgs::SetFingersPositionGoal goalFinger;
        goalFinger.fingers.finger1 = finger1_value;
        goalFinger.fingers.finger2 = finger2_value;
        // Not used for our arm
        goalFinger.fingers.finger3 = 0;

        fingers_action.waitForServer();
        fingers_action.sendGoal(goalFinger);
        return fingers_action.waitForResult();
    }

    bool Mico::move_fingers(int finger_value) {
        return move_fingers(finger_value, finger_value);
    }


    bool Mico::make_safe_for_travel() {
        safety_client.waitForExistence();
        segbot_arm_manipulation::NavSafety srv_safety;
        srv_safety.request.getSafe = true;

        if (safety_client.call(srv_safety)) {
            return srv_safety.response.safe;
        } else {
            return false;
        }
    }

    bool Mico::move_home() {
        kinova_msgs::HomeArm srv;
        return home_client.call(srv);
    }


    bool Mico::open_hand() {
        return move_fingers(OPEN_FINGER_VALUE);
    }

    bool Mico::close_hand() {
        return move_fingers(CLOSED_FINGER_VALUE);
    }

    moveit_msgs::GetPositionIK::Response Mico::compute_ik(const geometry_msgs::PoseStamped &p) {

        moveit_msgs::GetPositionIK::Request ik_request;
        moveit_msgs::GetPositionIK::Response ik_response;
        ik_request.ik_request.group_name = "arm";
        ik_request.ik_request.pose_stamped = p;

        /* Call the service */
        ik_client.call(ik_request, ik_response);
        return ik_response;

    }

    moveit_msgs::GetPositionFK::Response Mico::compute_fk() {

        moveit_msgs::GetPositionFK::Request req;
        moveit_msgs::GetPositionFK::Response res;

        //Load request with the desired link
        for (auto &joint_name: jointNames) {
            req.fk_link_names.push_back(joint_name);
        }

        if (fk_client.call(req, res)) {
            ros::spinOnce();
            ROS_DEBUG("FK call successful.");
        } else {
            ROS_ERROR("FK call failed. Moveit! probably can't be contacted. Preempting movement.");

        }
        return res;

    }

    bool Mico::move_to_pose_moveit(const geometry_msgs::PoseStamped &target,
                                   const vector<sensor_msgs::PointCloud2> &obstacles,
                                   const moveit_msgs::Constraints &constraints
    ) {
        bwi_moveit_utils::MoveitCartesianPose::Request req;
        bwi_moveit_utils::MoveitCartesianPose::Response res;

        vector<moveit_msgs::CollisionObject> moveit_obstacles = segbot_arm_manipulation::get_collision_boxes(obstacles);

        req.target = target;
        req.collision_objects = moveit_obstacles;
        req.constraints = constraints;
        return pose_moveit_client.call(req, res);

    }

    bool Mico::move_to_joint_state_moveit(const kinova_msgs::JointAngles &target,
                                          const vector<sensor_msgs::PointCloud2> &obstacles,
                                          const moveit_msgs::Constraints &constraints) {
        bwi_moveit_utils::MoveitJointPose::Request req;
        bwi_moveit_utils::MoveitJointPose::Response res;

        vector<moveit_msgs::CollisionObject> moveit_obstacles = segbot_arm_manipulation::get_collision_boxes(obstacles);

        vector<double> target_values;
        target_values.push_back(target.joint1);
        target_values.push_back(target.joint2);
        target_values.push_back(target.joint3);
        target_values.push_back(target.joint4);
        target_values.push_back(target.joint5);
        target_values.push_back(target.joint6);

        req.target = target_values;
        req.collision_objects = moveit_obstacles;
        req.constraints = constraints;
        return joint_angles_moveit_client.call(req, res);
    }

    bool Mico::move_to_joint_state_moveit(const sensor_msgs::JointState &target,
                                          const vector<sensor_msgs::PointCloud2> &obstacles,
                                          const moveit_msgs::Constraints &constraints) {

        return move_to_joint_state_moveit(segbot_arm_manipulation::state_to_angles(target), obstacles, constraints);
    }


    bool Mico::move_through_waypoints_moveit(const vector<geometry_msgs::Pose> &waypoints,
                                             const vector<sensor_msgs::PointCloud2> &obstacles,
                                             const moveit_msgs::Constraints &constraints) {
        bwi_moveit_utils::MoveitWaypoint::Request req;
        bwi_moveit_utils::MoveitWaypoint::Response res;

        vector<moveit_msgs::CollisionObject> moveit_obstacles = segbot_arm_manipulation::get_collision_boxes(obstacles);

        req.waypoints = waypoints;
        req.collision_objects = moveit_obstacles;
        req.constraints = constraints;
        return waypoint_moveit_client.call(req, res);
    }


    bool Mico::move_to_side_view() {
        if (!position_db->hasCarteseanPosition("side_view")) {
            return false;
        }
        geometry_msgs::PoseStamped out_of_view_pose = position_db->getToolPositionStamped("side_view",
                                                                                          "/m1n6s200_link_base");
        return move_to_pose_moveit(out_of_view_pose);

    }


    bool Mico::move_to_handover() {
        if (!position_db->hasCarteseanPosition("handover_front")) {
            return false;
        }
        geometry_msgs::PoseStamped handover_pose = position_db->getToolPositionStamped("handover_front",
                                                                                       "m1n6s200_link_base");

        return move_to_pose_moveit(handover_pose);


    }

    void Mico::move_with_angular_velocities(const kinova_msgs::JointAngles &velocities, const double seconds) {
        // Per the Kinova documentation, we have to publish at exactly 100Hz to get
        // predictable behavior
        ros::Rate r(arm_poll_rate);

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

    void Mico::move_with_cartesian_velocities(const kinova_msgs::PoseVelocity &velocities, const double seconds) {
        // Per the Kinova documentation, we have to publish at exactly 100Hz to get
        // predictable behavior
        ros::Rate r(arm_poll_rate);

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

    bool Mico::move_through_waypoints(const vector<geometry_msgs::Pose> &waypoints) {
        for (const auto &waypoint : waypoints) {
            kinova_msgs::KinovaPose next = kinova::KinovaPose(waypoint).constructKinovaPoseMsg();
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
}