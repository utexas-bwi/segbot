#include <ros/ros.h>
#include <bwi_manipulation/grasp_utils.h>
#include <bwi_manipulation/GraspCartesianCommand.h>
#include <segbot_arm_manipulation/Mico.h>
#include <segbot_arm_manipulation/arm_utils.h>
#include "segbot_arm_manipulation/TabletopGraspAction.h"

//actions
#include <actionlib/server/simple_action_server.h>

//srv for talking to table_object_detection_node.cpp
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/GetPositionFK.h>
#include <geometry_msgs/PoseArray.h>
#include <bwi_perception/BoundingBox.h>
#include <agile_grasp/Grasps.h>

//the action definition


//some defines related to filtering candidate grasps
#define MIN_DISTANCE_TO_PLANE 0.05

#define HAND_OFFSET_GRASP 0.03
#define HAND_OFFSET_APPROACH 0.1


//used when deciding whether a pair of an approach pose and a grasp pose are good;
//if the angular difference in joint space is too big, this means that the robot 
//cannot directly go from approach to grasp pose (so we filter those pairs out)
#define ANGULAR_DIFF_THRESHOLD 3.0


/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


using namespace bwi_manipulation;
using namespace std;

class TabletopGraspActionServer {
protected:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<segbot_arm_manipulation::TabletopGraspAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    segbot_arm_manipulation::TabletopGraspFeedback feedback_;
    segbot_arm_manipulation::TabletopGraspResult result_;

    agile_grasp::Grasps current_grasps;

    ros::Publisher agile_grasp_cloud_pub;
    ros::Publisher target_cloud_pub;
    ros::Publisher pose_array_pub;

    //used to compute transforms
    tf::TransformListener listener;

    bool heard_grasps;
    ros::Subscriber sub_grasps;

    //subscribers -- in an action server, these have to be class members
    segbot_arm_manipulation::Mico mico;

public:

    TabletopGraspActionServer(const std::string &name) :
            as_(nh_, name, boost::bind(&TabletopGraspActionServer::executeCB, this, _1), false),
            action_name_(name),
            mico(nh_), heard_grasps(false){
        pnh = ros::NodeHandle("~");
        //subscriber for grasps
        sub_grasps = nh_.subscribe("/find_grasps/grasps_handles", 1, &TabletopGraspActionServer::grasps_cb, this);

        //publish pose array
        pose_array_pub = pnh.advertise<geometry_msgs::PoseArray>("grasp_candidate", 10);

        //debugging publisher
        target_cloud_pub = pnh.advertise<sensor_msgs::PointCloud2>("target_cloud", 10);
        agile_grasp_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud", 10);

        ROS_INFO("Starting grasp action server...");

        as_.start();
    }

    ~TabletopGraspActionServer() = default;


    void grasps_cb(const agile_grasp::Grasps &msg) {
        ROS_INFO("Heard grasps!");
        current_grasps = msg;
        heard_grasps = true;
    }
    

    void listenForGrasps(float rate) {
        ros::Rate r(rate);
        heard_grasps = false;
        while (ros::ok()) {
            ros::spinOnce();
            if (heard_grasps)
                return;
            r.sleep();
        }
    }

    bool passesFilter(const std::string &filterName, const bwi_manipulation::GraspCartesianCommand &gc) {

        tf::Quaternion q(gc.approach_pose.pose.orientation.x,
                         gc.approach_pose.pose.orientation.y,
                         gc.approach_pose.pose.orientation.z,
                         gc.approach_pose.pose.orientation.w);
        tf::Matrix3x3 m(q);

        double r, p, y;
        m.getRPY(r, p, y);


        if (filterName == segbot_arm_manipulation::TabletopGraspGoal::SIDEWAY_GRASP_FILTER) {

            //ideally roll should be PI/2, while pitch should be 0

            return r > 1.1 && r < 1.9 && p > -0.25 && p < 0.25;

        } else if (filterName == segbot_arm_manipulation::TabletopGraspGoal::TOPDOWN_GRASP_FILTER) {
            double roll_abs = fabs(r);

            return roll_abs < 3.3 && roll_abs > 2.6 && p > -0.3 && p < 0.3;
        }

        return true;
    }


    std::vector<GraspCartesianCommand>
    generate_agile_grasps(const PointCloudT::Ptr &target_cloud, const std::string &frame_id) {
        ROS_INFO("[segbot_tabletop_grasp_as.cpp] Publishing point cloud...");
        // This extra conversion here is a bit wasteful, but it the penalty is dwarfed
        // by the publication, agile grasp overhead anyway
        string target_frame;
        nh_.getParam("find_grasps/cloud_frame",target_frame);

        PointCloudT::Ptr target_cloud_transformed(new PointCloudT);
        pcl_ros::transformPointCloud(target_frame, *target_cloud, *target_cloud_transformed, listener);

        sensor_msgs::PointCloud2 target_object_pc2;
        pcl::toROSMsg<PointT>(*target_cloud_transformed, target_object_pc2);


        agile_grasp_cloud_pub.publish(target_object_pc2);


        //wait for response at 5 Hz
        listenForGrasps(40.0);

        ROS_INFO("[segbot_tabletop_grasp_as.cpp] Heard %i grasps", (int) current_grasps.grasps.size());

        //next, compute approach and grasp poses for each detected grasp


        listener.waitForTransform(frame_id, "m1n6s200_link_base", ros::Time(0), ros::Duration(3.0));


        //here, we'll store all grasp options that pass the filters
        std::vector<GraspCartesianCommand> grasp_commands;

        for (const auto &grasp: current_grasps.grasps) {
            GraspCartesianCommand gc = bwi_manipulation::GraspCartesianCommand::from_agile_grasp(
                    grasp, HAND_OFFSET_APPROACH, HAND_OFFSET_GRASP, frame_id);
            grasp_commands.push_back(gc);
        }
        return grasp_commands;


    }


    void generate_grasps_along_bounding_box_side(const bwi_perception::BoundingBox &box, ulong varying_dimension,
                                                 const Eigen::Vector4f &varied_min, const Eigen::Vector4f &varied_max,
                                                 const Eigen::Vector4f &fixed, vector<GraspCartesianCommand> &grasps,
                                                 const geometry_msgs::Quaternion &orientation) {
        int steps = 10;
        double range = varied_max(varying_dimension) - varied_min(varying_dimension);
        double step_size = range / steps;

        geometry_msgs::PoseStamped grasp_pose;

        grasp_pose.header.frame_id = box.frame_id;
        grasp_pose.pose.orientation = orientation;

        for (int i = 0; i < steps; ++i) {
            Eigen::Vector4f position = fixed;
            position(varying_dimension) = varied_min(varying_dimension) + step_size * i;
            grasp_pose.pose.position.x = position.x();
            grasp_pose.pose.position.y = position.y();
            grasp_pose.pose.position.z = position.z();
            grasps.push_back(GraspCartesianCommand::from_grasp_pose(grasp_pose, HAND_OFFSET_APPROACH));
        }
    }


    void generate_grasps_varying_orientation(const bwi_perception::BoundingBox &box,
                                             const geometry_msgs::Quaternion &center_orientation,
                                             const double angle_radius, const Eigen::Vector4f &position,
                                             vector<GraspCartesianCommand> &grasps) {
        tf::Quaternion q;
        tf::quaternionMsgToTF(center_orientation, q);
        tf::Matrix3x3 m(q);

        // Get the min RPY values
        double b_r, b_p, b_y;
        m.getRPY(b_r, b_p, b_y);
        b_r -= angle_radius;
        b_p -= angle_radius;
        b_y -= angle_radius;

        int steps = 3;

        double step_size = angle_radius * 2.0 / steps;

        geometry_msgs::PoseStamped grasp_pose;

        grasp_pose.header.frame_id = box.frame_id;
        grasp_pose.pose.position.x = position.x();
        grasp_pose.pose.position.y = position.y();
        grasp_pose.pose.position.z = position.z();

        tf::Stamped<tf::Quaternion> quat;
        quat.frame_id_ = box.frame_id;

        geometry_msgs::QuaternionStamped quat_stamped;

        for (int i = 0; i < steps; ++i) {
            for (int j = 0; j < steps; ++j) {
                for (int k = 0; k < steps; ++k) {
                    quat.setRPY(b_r + i * step_size, b_p * j * step_size, b_y * step_size);
                    tf::quaternionStampedTFToMsg(quat, quat_stamped);
                    grasp_pose.pose.orientation = quat_stamped.quaternion;
                    grasps.push_back(GraspCartesianCommand::from_grasp_pose(grasp_pose, HAND_OFFSET_APPROACH));
                }
            }

        }
    }

    //TODO: Break this into a class and move it to bwi_manipulation
    std::vector<GraspCartesianCommand>
    generate_heuristic_grasps(const PointCloudT::Ptr &target_cloud, const std::string &frame_id) {


        vector<GraspCartesianCommand> grasp_commands;
        // Move from the sensor frame to the arm base frame
        const PointCloudT::Ptr &arm_frame(target_cloud);
        pcl_ros::transformPointCloud("m1n6s200_link_base", *arm_frame, *arm_frame, listener);
        auto boundingBox = bwi_perception::BoundingBox::from_cloud<PointT>(arm_frame);


        // The end effector frame has z extending along the finger tips. Here
        // we set roll pitch yaw with respect to the link base axes, which have the x axis extending
        // forward from the base of the robot. We pitch by 90 degrees to point the hands along the base's x axis

        // (point forward).
        tf::Stamped<tf::Quaternion> quat;
        quat.setRPY(0.0, M_PI / 2, 0);
        quat.frame_id_ = "m1n6s200_link_base";

        geometry_msgs::QuaternionStamped quat_stamped;
        tf::quaternionStampedTFToMsg(quat, quat_stamped);

        // Center of the object, but the minimum along the X axis
        Eigen::Vector4f fixed = boundingBox.position;
        fixed.x() = boundingBox.min.x()+HAND_OFFSET_GRASP;

        // Vary along Z axis between object min and max
        generate_grasps_along_bounding_box_side(boundingBox, 2, boundingBox.min, boundingBox.max, fixed, grasp_commands,
                                                quat_stamped.quaternion);

        fixed = boundingBox.position;
        fixed.z() = boundingBox.max.z()-HAND_OFFSET_GRASP;

        // Point down
        quat.setRPY(0.0, M_PI, 0);
        tf::quaternionStampedTFToMsg(quat, quat_stamped);

    //    generate_grasps_varying_orientation(boundingBox, quat_stamped.quaternion, 0.25, fixed, grasp_commands);

        // Right side grasp
        fixed = boundingBox.position;
        fixed.y() = boundingBox.min.y()+HAND_OFFSET_GRASP;

        // Point left
        quat.setRPY(0.0, M_PI / 2, M_PI / 2);
        tf::quaternionStampedTFToMsg(quat, quat_stamped);
        generate_grasps_along_bounding_box_side(boundingBox, 2, boundingBox.min, boundingBox.max, fixed, grasp_commands,
                                                quat_stamped.quaternion);

        // Left side grasp
        fixed = boundingBox.position;
        fixed.y() = boundingBox.max.y()-HAND_OFFSET_GRASP;

        // Point right
        quat.setRPY(0.0, M_PI / 2, -M_PI / 2);
        tf::quaternionStampedTFToMsg(quat, quat_stamped);
        generate_grasps_along_bounding_box_side(boundingBox, 2, boundingBox.min, boundingBox.max, fixed, grasp_commands,
                                                quat_stamped.quaternion);


        for (auto &grasp: grasp_commands) {
            listener.transformPose(frame_id, grasp.approach_pose, grasp.approach_pose);
            listener.transformPose(frame_id, grasp.grasp_pose, grasp.grasp_pose);
        }
        return grasp_commands;

    }


    ulong select_grasp(vector<GraspCartesianCommand> grasps, const std::string &selection_method, const bwi_perception::BoundingBox &boundingBox) {
        ulong selected_grasp_index;

        if (selection_method ==
            segbot_arm_manipulation::TabletopGraspGoal::CLOSEST_ORIENTATION_SELECTION) {
            //find the grasp with closest orientation to current pose
            double min_diff = numeric_limits<double>::max();
            for (unsigned int i = 0; i < grasps.size(); i++) {
                double d_i = segbot_arm_manipulation::quat_angular_difference(
                        grasps.at(i).approach_pose.pose.orientation, mico.current_pose.pose.orientation);

                ROS_INFO("Distance for pose %i:\t%f", (int) i, d_i);
                if (d_i < min_diff) {
                    selected_grasp_index = (int) i;
                    min_diff = d_i;
                }
            }
        } else if (selection_method == segbot_arm_manipulation::TabletopGraspGoal::RANDOM_SELECTION) {

            srand(time(NULL));
            selected_grasp_index = rand() % grasps.size();
            ROS_INFO("Randomly selected grasp = %i", (int) selected_grasp_index);
        } else if (selection_method == segbot_arm_manipulation::TabletopGraspGoal::CLOSEST_JOINTSPACE_SELECTION) {

            double min_diff = std::numeric_limits<double>::max();
            for (unsigned int i = 0; i < grasps.size(); i++) {
                std::vector<double> D_i = segbot_arm_manipulation::getJointAngleDifferences(
                        grasps.at(i).approach_joint_state, mico.current_state);

                double sum_d = 0;
                for (double p : D_i)
                    sum_d += p;

                if (sum_d < min_diff) {
                    selected_grasp_index = (int) i;
                    min_diff = sum_d;
                }
            }
        }

    else if (selection_method == segbot_arm_manipulation::TabletopGraspGoal::CLOSEST_TO_CENTROID_SELECTION) {
        double min_diff = std::numeric_limits<double>::max();
        for (unsigned int i = 0; i < grasps.size(); i++) {
            double dist = segbot_arm_manipulation::getDistanceDifferences(
                    grasps.at(i).grasp_pose.pose, boundingBox.centroid);

            if (dist < min_diff) {
                selected_grasp_index = (int) i;
                min_diff = dist;
            }
        }
    }
        return selected_grasp_index;
    }

    void executeCB(const segbot_arm_manipulation::TabletopGraspGoalConstPtr &goal) {

        GraspCartesianCommand final_pose;

        if (goal->cloud_clusters.empty()) {
            ROS_INFO("[segbot_tabletop_grap_as.cpp] No object point clouds received...aborting");
            as_.setAborted(result_);
            return;
        }


        ROS_INFO("[segbot_tabletop_grap_as.cpp] Received action request...proceeding.");
        mico.wait_for_data();

        //the result
        segbot_arm_manipulation::TabletopGraspResult result;

        //get the data out of the goal
        Eigen::Vector4f plane_coef_vector;
        for (int i = 0; i < 4; i++)
            plane_coef_vector(i) = goal->cloud_plane_coef[i];
        int selected_object = goal->target_object_cluster_index;

        PointCloudT::Ptr target_object(new PointCloudT);
        sensor_msgs::PointCloud2 target_object_pc2 = goal->cloud_clusters.at(goal->target_object_cluster_index);
        pcl::fromROSMsg(target_object_pc2, *target_object);

        std::string sensor_frame_id = goal->cloud_clusters.at(
                goal->target_object_cluster_index).header.frame_id;

        std::vector<GraspCartesianCommand> candidate_grasps;
        if (goal->grasp_generation_method == segbot_arm_manipulation::TabletopGraspGoal::HEURISTIC) {
            //tf::Stamped<tf::Quaternion> quat;
            //quat.setRPY(-M_PI/2, 0.0, -M_PI/2);
            //quat.frame_id_ = "m1n6s200_link_base";

            //geometry_msgs::QuaternionStamped quat_stamped;
            //tf::quaternionStampedTFToMsg(quat, quat_stamped);
            //candidate_grasps = generate_heuristic_grasps<PointT>(target_object,
            //                                                    quat_stamped,
            //                                                    HAND_OFFSET_GRASP,
            //                                                    HAND_OFFSET_APPROACH,
            //                                                    listener);
	    candidate_grasps = generate_heuristic_grasps(target_object, sensor_frame_id);
            target_cloud_pub.publish(target_object_pc2);
        } else {
            candidate_grasps = generate_agile_grasps(target_object, sensor_frame_id);
        }
        std::vector<GraspCartesianCommand> surviving_grasps;

        ROS_INFO("[segbot_tabletop_grap_as.cpp] found %lu possible grasps",candidate_grasps.size());
        for (const auto &grasp : candidate_grasps) {

            //geometry_msgs::PoseArray pa;
            //pa.header = grasp.approach_pose.header;
            //pa.poses.push_back(grasp.approach_pose.pose);
            //pa.poses.push_back(grasp.grasp_pose.pose);
            //pose_array_pub.publish(pa);
            ros::spinOnce();
            //filter 1: if the grasp is too close to plane, reject it
            bool ok_with_plane = bwi_manipulation::grasp_utils::checkPlaneConflict(grasp, plane_coef_vector,
                                                                                   MIN_DISTANCE_TO_PLANE);
            if (!ok_with_plane) {
                ROS_INFO("Grasp too close to plane");
                continue;
            }
            auto complete_grasp = grasp;

            //for filter 2, the grasps need to be in the arm's frame of reference
            listener.transformPose("m1n6s200_link_base", complete_grasp.approach_pose, complete_grasp.approach_pose);
            listener.transformPose("m1n6s200_link_base", complete_grasp.grasp_pose, complete_grasp.grasp_pose);


            //filter 2: apply grasp filter method in request
            bool passed_filter = passesFilter(goal->grasp_filter_method, complete_grasp);

            if (!passed_filter) {
                ROS_INFO("Filter failed");
                continue;
            }


            //filter two -- if IK fails
            moveit_msgs::GetPositionIK::Response ik_response_approach = mico.compute_ik(complete_grasp.approach_pose);

            if (ik_response_approach.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
                ROS_INFO("No IK solution for approach pose");
                continue;
            }
            moveit_msgs::GetPositionIK::Response ik_response_grasp = mico.compute_ik(complete_grasp.grasp_pose);

            if (ik_response_grasp.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
                ROS_INFO("No IK solution for grasp pose");
                continue;
            }


            //now check to see how close the two sets of joint angles are -- if the joint configurations for the approach and grasp poses differ by too much, the grasp will not be accepted
            std::vector<double> D = segbot_arm_manipulation::getJointAngleDifferences(
                    ik_response_approach.solution.joint_state, ik_response_grasp.solution.joint_state);

            double sum_d = 0;
            for (double p : D) {
                sum_d += p;
            }


            if (sum_d >= ANGULAR_DIFF_THRESHOLD) {
                ROS_INFO("Approach and grasp configurations too different") ; continue;
            }

            //store the IK results

            complete_grasp.approach_joint_state = ik_response_approach.solution.joint_state;
            complete_grasp.grasp_joint_state = ik_response_grasp.solution.joint_state;

            surviving_grasps.push_back(complete_grasp);
        }

        //check to see if all potential grasps have been filtered out
        if (surviving_grasps.empty()) {
            ROS_WARN("[segbot_tabletop_grasp_as.cpp] No feasible grasps found. Aborting.");
            as_.setAborted(result_);
            return;
        }


        //make sure we're working with the correct tool pose
        mico.wait_for_data();

        // Move from the sensor frame to the arm base frame
        const PointCloudT::Ptr &arm_frame(target_object);
        pcl_ros::transformPointCloud("m1n6s200_link_base", *arm_frame, *arm_frame, listener);
        bwi_perception::BoundingBox boundingBox = bwi_perception::BoundingBox::from_cloud<PointT>(arm_frame);

        ulong selected_grasp_index = select_grasp(surviving_grasps, goal->grasp_selection_method,boundingBox);
        final_pose=surviving_grasps.at(selected_grasp_index);
        //compute RPY for target pose
        ROS_INFO("Selected approach pose:");
        ROS_INFO_STREAM(final_pose.approach_pose);

        geometry_msgs::PoseArray pa;
        pa.header = final_pose.approach_pose.header;
        pa.poses.push_back(final_pose.approach_pose.pose);
        pa.poses.push_back(final_pose.grasp_pose.pose);
        pose_array_pub.publish(pa);
        //close fingers while moving
        mico.close_hand();

        //move to approach pose -- do it twice to correct
        mico.move_to_pose_moveit(final_pose.approach_pose, goal->cloud_clusters );
        mico.move_to_pose_moveit(final_pose.approach_pose, goal->cloud_clusters );

        //open fingers
        mico.open_hand();

        //move to grasp pose
        mico.move_to_pose_moveit(final_pose.grasp_pose);
        mico.move_to_pose_moveit(final_pose.grasp_pose);

        //close hand
        mico.close_hand();

        result_.approach_joint_state=final_pose.approach_joint_state;
        result_.approach_pose=final_pose.approach_pose;
        result_.grasp_joint_state=final_pose.grasp_joint_state;
        result_.grasp_pose=final_pose.grasp_pose;


        result_.success = true;
        as_.setSucceeded(result_);
        return;
    }

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "segbot_tabletop_grasp_as");

    TabletopGraspActionServer as(ros::this_node::getName());
    ros::spin();

    return 0;
}
