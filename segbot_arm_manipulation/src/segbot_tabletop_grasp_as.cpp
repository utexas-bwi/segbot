#include <ros/ros.h>

#include <segbot_arm_manipulation/MicoManager.h>
#include <segbot_arm_manipulation/grasp_utils.h>
#include <segbot_arm_manipulation/arm_utils.h>


//actions
#include <actionlib/server/simple_action_server.h>


//srv for talking to table_object_detection_node.cpp
#include "bwi_perception/TabletopPerception.h"

#include <moveit_msgs/DisplayRobotState.h>
// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>


#include <kinova_msgs/PoseVelocity.h>
#include <geometry_msgs/PoseArray.h>

//the action definition
#include "segbot_arm_manipulation/TabletopGraspAction.h"

//tf stuff
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

//some defines related to filtering candidate grasps
#define MIN_DISTANCE_TO_PLANE 0.05

#define HAND_OFFSET_GRASP 0.02
#define HAND_OFFSET_APPROACH 0.10


//used when deciding whether a pair of an approach pose and a grasp pose are good;
//if the angular difference in joint space is too big, this means that the robot 
//cannot directly go from approach to grasp pose (so we filter those pairs out)
#define ANGULAR_DIFF_THRESHOLD 3.0


/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


class TabletopGraspActionServer {
protected:

    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<segbot_arm_manipulation::TabletopGraspAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    segbot_arm_manipulation::TabletopGraspFeedback feedback_;
    segbot_arm_manipulation::TabletopGraspResult result_;

    agile_grasp::Grasps current_grasps;
    
    ros::Publisher cloud_pub;
    ros::Publisher cloud_grasp_pub;
    ros::Publisher pose_array_pub;
    ros::Publisher pose_pub;

    std::vector<PointCloudT::Ptr> detected_objects;

    //used to compute transforms
    tf::TransformListener listener;

    bool heard_grasps;
    ros::Subscriber sub_grasps;

    //subscribers -- in an action server, these have to be class members
    MicoManager mico;

public:

    TabletopGraspActionServer(const std::string &name) :
            as_(nh_, name, boost::bind(&TabletopGraspActionServer::executeCB, this, _1), false),
            action_name_(name),
            mico(nh_), heard_grasps(false){
        
        //subscriber for grasps
        sub_grasps = nh_.subscribe("/find_grasps/grasps_handles", 1, &TabletopGraspActionServer::grasps_cb, this);

        //publish pose array
        pose_array_pub = nh_.advertise<geometry_msgs::PoseArray>("pose_array", 10);

        //publish pose 
        pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/agile_grasp_demo/pose_out", 10);

        //debugging publisher
        cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud_debug", 10);
        cloud_grasp_pub = nh_.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud", 10);

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

    bool passesFilter(const std::string &filterName, const GraspCartesianCommand &gc) {

        tf::Quaternion q(gc.approach_pose.pose.orientation.x,
                         gc.approach_pose.pose.orientation.y,
                         gc.approach_pose.pose.orientation.z,
                         gc.approach_pose.pose.orientation.w);
        tf::Matrix3x3 m(q);

        double r, p, y;
        m.getRPY(r, p, y);


        if (filterName == segbot_arm_manipulation::TabletopGraspGoal::SIDEWAY_GRASP_FILTER) {

            //ROS_INFO("%f, %f",fabs(p),fabs(3.14/2.0 - r) ); 

            //ideally roll should be PI/2, while pitch should be 0

            if (r > 1.1 && r < 1.9 && p > -0.25 && p < 0.25)
                return true;
            else return false;

        } else if (filterName == segbot_arm_manipulation::TabletopGraspGoal::TOPDOWN_GRASP_FILTER) {
            double roll_abs = fabs(r);

            if (roll_abs < 3.3 && roll_abs > 2.6 && p > -0.3 && p < 0.3) {
                return true;
            } else {
                return false;
            }
        }

        return true;
    }


    void executeCB(const segbot_arm_manipulation::TabletopGraspGoalConstPtr &goal) {

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
        pcl::PCLPointCloud2 target_object_pc2;
        pcl_conversions::toPCL(goal->cloud_clusters.at(goal->target_object_cluster_index), target_object_pc2);
        pcl::fromPCLPointCloud2(target_object_pc2, *target_object);

        ROS_INFO("[segbot_tabletop_grasp_as.cpp] Publishing point cloud...");
        cloud_grasp_pub.publish(goal->cloud_clusters.at(goal->target_object_cluster_index));

        //wait for response at 5 Hz
        listenForGrasps(40.0);

        ROS_INFO("[segbot_tabletop_grasp_as.cpp] Heard %i grasps", (int) current_grasps.grasps.size());

        //next, compute approach and grasp poses for each detected grasp

        //wait for transform from visual space to arm space

        std::string sensor_frame_id = goal->cloud_clusters.at(goal->target_object_cluster_index).header.frame_id;

        listener.waitForTransform(sensor_frame_id, "m1n6s200_link_base", ros::Time(0), ros::Duration(3.0));


        //here, we'll store all grasp options that pass the filters
        std::vector<GraspCartesianCommand> grasp_commands;


        for (const auto &grasp : current_grasps.grasps) {

            GraspCartesianCommand gc_i = segbot_arm_manipulation::grasp_utils::constructGraspCommand(
                    grasp, HAND_OFFSET_APPROACH, HAND_OFFSET_GRASP, sensor_frame_id);

            geometry_msgs::PoseArray pa;
            pa.header = gc_i.approach_pose.header;
            pa.poses.push_back(gc_i.approach_pose.pose);
            pa.poses.push_back(gc_i.grasp_pose.pose);
            pose_array_pub.publish(pa);
            ros::spinOnce();
            //filter 1: if the grasp is too close to plane, reject it
            bool ok_with_plane = segbot_arm_manipulation::grasp_utils::checkPlaneConflict(gc_i, plane_coef_vector,
                                                                                          MIN_DISTANCE_TO_PLANE);
            if (!ok_with_plane) {
                ROS_INFO("Grasp too close to plane");
                continue;
            }

            //for filter 2, the grasps need to be in the arm's frame of reference
            listener.transformPose("m1n6s200_link_base", gc_i.approach_pose, gc_i.approach_pose);
            listener.transformPose("m1n6s200_link_base", gc_i.grasp_pose, gc_i.grasp_pose);


            //filter 2: apply grasp filter method in request
            bool passed_filter = passesFilter(goal->grasp_filter_method, gc_i);

            if (!passed_filter) {
                ROS_INFO("Filter failed");
                continue;
            }


            //filter two -- if IK fails
            moveit_msgs::GetPositionIK::Response ik_response_approach = mico.compute_ik(gc_i.approach_pose);

            if (ik_response_approach.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
                ROS_INFO("No IK solution for approach pose");
                continue;
            }
            moveit_msgs::GetPositionIK::Response ik_response_grasp = mico.compute_ik(gc_i.grasp_pose);

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
                ROS_INFO("Approach and grasp configurations too different");
                continue;
            }

            //store the IK results
            gc_i.approach_q = ik_response_approach.solution.joint_state;
            gc_i.grasp_q = ik_response_grasp.solution.joint_state;

            grasp_commands.push_back(gc_i);
        }


        //check to see if all potential grasps have been filtered out
        if (grasp_commands.empty()) {
            ROS_WARN("[segbot_tabletop_grasp_as.cpp] No feasible grasps found. Aborting.");
            as_.setAborted(result_);
            return;
        }

        //make sure we're working with the correct tool pose
        mico.wait_for_data();

        int selected_grasp_index = -1;


        if (goal->grasp_selection_method ==
            segbot_arm_manipulation::TabletopGraspGoal::CLOSEST_ORIENTATION_SELECTION) {
            //find the grasp with closest orientatino to current pose
            double min_diff = 1000000.0;
            for (unsigned int i = 0; i < grasp_commands.size(); i++) {
                double d_i = segbot_arm_manipulation::quat_angular_difference(
                        grasp_commands.at(i).approach_pose.pose.orientation, mico.current_pose.pose.orientation);

                ROS_INFO("Distance for pose %i:\t%f", (int) i, d_i);
                if (d_i < min_diff) {
                    selected_grasp_index = (int) i;
                    min_diff = d_i;
                }
            }
        } else if (goal->grasp_selection_method == segbot_arm_manipulation::TabletopGraspGoal::RANDOM_SELECTION) {

            srand(time(NULL));
            selected_grasp_index = rand() % grasp_commands.size();
            ROS_INFO("Randomly selected grasp = %i", selected_grasp_index);
        } else if (goal->grasp_selection_method ==
                   segbot_arm_manipulation::TabletopGraspGoal::CLOSEST_JOINTSPACE_SELECTION) {

            double min_diff = 1000000.0;
            for (unsigned int i = 0; i < grasp_commands.size(); i++) {
                std::vector<double> D_i = segbot_arm_manipulation::getJointAngleDifferences(
                        grasp_commands.at(i).approach_q, mico.current_state);

                double sum_d = 0;
                for (double p : D_i)
                    sum_d += p;

                if (sum_d < min_diff) {
                    selected_grasp_index = (int) i;
                    min_diff = sum_d;
                }
            }

        }

        if (selected_grasp_index == -1) {
            ROS_WARN("[segbot_tabletop_grasp_as.cpp] Grasp selection failed. Aborting.");
            as_.setAborted(result_);
            return;
        }

        //compute RPY for target pose
        ROS_INFO("Selected approach pose:");
        ROS_INFO_STREAM(grasp_commands.at(selected_grasp_index).approach_pose);

        //publish individual pose for visualization purposes
        pose_pub.publish(grasp_commands.at(selected_grasp_index).approach_pose);

        //close fingers while moving
        mico.close_hand();

        //move to approach pose -- do it twice to correct
        mico.move_to_pose_moveit(grasp_commands.at(selected_grasp_index).approach_pose);
        mico.move_to_pose_moveit(grasp_commands.at(selected_grasp_index).approach_pose);

        //open fingers
        mico.open_hand();

        //move to grasp pose
        mico.move_to_pose_moveit(grasp_commands.at(selected_grasp_index).grasp_pose);

        //close hand
        mico.close_hand();

        result_.success = true;
        as_.setSucceeded(result_);
    }

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "segbot_tabletop_grasp_as");

    TabletopGraspActionServer as(ros::this_node::getName());
    ros::spin();

    return 0;
}