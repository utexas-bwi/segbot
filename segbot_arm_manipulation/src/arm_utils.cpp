#include <vector>
#include <sensor_msgs/JointState.h>
#include <segbot_arm_manipulation/arm_utils.h>
#include <kinova_msgs/JointAngles.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>
#include <Eigen/src/Core/Matrix.h>
#include <sensor_msgs/PointCloud2.h>
#include <moveit_msgs/CollisionObject.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>

using namespace std;

namespace segbot_arm_manipulation {
    std::vector<double> getJointAngleDifferences(sensor_msgs::JointState A, sensor_msgs::JointState B) {
        return segbot_arm_manipulation::getJointAngleDifferences(segbot_arm_manipulation::state_to_angles(A), segbot_arm_manipulation::state_to_angles(B));
    }

    std::vector<double> getJointAngleDifferences(kinova_msgs::JointAngles A, kinova_msgs::JointAngles B) {
        std::vector<double> result;
        std::set<string> arm_joints;
        arm_joints.insert(std::begin(jointNames), std::end(jointNames));
        std::map<string, double> a_pos;
        std::map<string, double> b_pos;
        a_pos.insert(std::pair<string, double>(jointNames[0], A.joint1));
        a_pos.insert(std::pair<string, double>(jointNames[1], A.joint2));
        a_pos.insert(std::pair<string, double>(jointNames[2], A.joint3));
        a_pos.insert(std::pair<string, double>(jointNames[3], A.joint4));
        a_pos.insert(std::pair<string, double>(jointNames[4], A.joint5));
        a_pos.insert(std::pair<string, double>(jointNames[5], A.joint6));

        b_pos.insert(std::pair<string, double>(jointNames[0], B.joint1));
        b_pos.insert(std::pair<string, double>(jointNames[1], B.joint2));
        b_pos.insert(std::pair<string, double>(jointNames[2], B.joint3));
        b_pos.insert(std::pair<string, double>(jointNames[3], B.joint4));
        b_pos.insert(std::pair<string, double>(jointNames[4], B.joint5));
        b_pos.insert(std::pair<string, double>(jointNames[5], B.joint6));

        for (unsigned int i = 0; i < arm_joints.size(); i++) {
            std::string name = jointNames[i];
            double a_p = a_pos[name];
            double b_p = b_pos[name];

            double d;
            if (name == "m1n6s200_joint_2" || name == "m1n6s200_joint_3")
                d = fabs(a_p - b_p);
            else {
                if (b_p > a_p) {
                    if (b_p - a_p < a_p + 2 * PI - b_p)
                        d = fabs(b_p - a_p);
                    else
                        d = fabs(2 * PI - b_p + a_p);
                } else {
                    if (a_p - b_p > b_p + 2 * PI - a_p)
                        d = fabs(b_p + 2 * PI - a_p);
                    else
                        d = fabs(a_p - b_p);
                }

            }
            result.push_back(d);
        }
        return result;
    }

    std::vector<moveit_msgs::CollisionObject>
    get_collision_boxes(std::vector<sensor_msgs::PointCloud2> obstacles) {
        //wait for transform and perform it

        std::vector<moveit_msgs::CollisionObject> collision_objects;
        for (int i = 0; i < obstacles.size(); i++) {

            // transform
            tf::TransformListener tf_listener;
            tf_listener.waitForTransform(obstacles[i].header.frame_id, "base_link",
                                         ros::Time(0), ros::Duration(3.0));

            sensor_msgs::PointCloud2 object_cloud;
            pcl_ros::transformPointCloud("base_link", obstacles[i], object_cloud, tf_listener);

            // convert to PCL
            PointCloudT::Ptr object_i(new PointCloudT);
            pcl::PCLPointCloud2 pc_i;
            pcl_conversions::toPCL(object_cloud, pc_i);
            pcl::fromPCLPointCloud2(pc_i, *object_i);

            // get the min and max
            PointT min_pt;
            PointT max_pt;

            //3D Min/Max
            pcl::getMinMax3D(*object_i, min_pt, max_pt);
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*object_i, centroid);

            // create a bounding box
            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = "base_link";

            //Id of object used to identify it
            collision_object.id = "box";

            //Define a box to add to the world
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);

            primitive.dimensions[0] = max_pt.x - min_pt.x;
            primitive.dimensions[1] = max_pt.y - min_pt.y;
            primitive.dimensions[2] = max_pt.z - min_pt.z;

            geometry_msgs::Pose box_pose;
            box_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
            box_pose.position.x = centroid[0];
            box_pose.position.y = centroid[1];
            box_pose.position.z = centroid[2];

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;
            collision_objects.push_back(collision_object);

        }

        return collision_objects;
    }


    sensor_msgs::JointState values_to_joint_state(std::vector<float> joint_values) {
        sensor_msgs::JointState js;

        for (unsigned int i = 0; i < 8; i++) {
            js.name.push_back(jointNames[i]);

            if (i < 6) {
                js.position.push_back(joint_values.at(i));
            } else
                js.position.push_back(0.0);

            js.velocity.push_back(0.0);
            js.effort.push_back(0.0);
        }

        return js;
    }



    kinova_msgs::JointAngles state_to_angles(const sensor_msgs::JointState &state) {
        //check if this is specified just for the arm

        std::map<string, double> joint_pos;
        for(int i = 0; i < state.name.size(); i++) {
            joint_pos.insert(std::pair<string, double>(state.name.at(i), state.position.at(i)));
        }

        kinova_msgs::JointAngles as_angles;
        as_angles.joint1 = joint_pos[jointNames[0]];
        as_angles.joint2 = joint_pos[jointNames[1]];
        as_angles.joint3 = joint_pos[jointNames[2]];
        as_angles.joint4 = joint_pos[jointNames[3]];
        as_angles.joint5 = joint_pos[jointNames[4]];
        as_angles.joint6 = joint_pos[jointNames[5]];

        return as_angles;
    }


    double quat_angular_difference(const geometry_msgs::Quaternion &a, const geometry_msgs::Quaternion &b){
        Eigen::Vector4f dv;
        dv[0] = b.w; dv[1] = b.x; dv[2] = b.y; dv[3] = b.z;
        Eigen::Matrix<float, 3,4> inv;
        inv(0,0) = -a.x; inv(0,1) = a.w; inv(0,2) = -a.z; inv(0,3) = a.y;
        inv(1,0) = -a.y; inv(1,1) = a.z; inv(1,2) = a.w;	inv(1,3) = -a.x;
        inv(2,0) = -a.z; inv(2,1) = -a.y;inv(2,2) = a.x;  inv(2,3) = a.w;

        Eigen::Vector3f m = inv * dv * -2.0;
        return m.norm();
    }


}