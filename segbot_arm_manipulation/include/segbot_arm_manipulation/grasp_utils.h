#ifndef GRASP_UTILS_H
#define GRASP_UTILS_H
#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>


#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

//tf stuff
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>



#include "agile_grasp/Grasps.h"
#include "agile_grasp/Grasp.h"

struct GraspCartesianCommand {
	sensor_msgs::JointState approach_q;
	geometry_msgs::PoseStamped approach_pose;
		
	sensor_msgs::JointState grasp_q;
	geometry_msgs::PoseStamped grasp_pose;
		
};

namespace segbot_arm_manipulation {
	namespace grasp_utils {
		
		double quat_angular_difference(geometry_msgs::Quaternion c,geometry_msgs::Quaternion d){
			Eigen::Vector4f dv;
			dv[0] = d.w; dv[1] = d.x; dv[2] = d.y; dv[3] = d.z;
			Eigen::Matrix<float, 3,4> inv;
			inv(0,0) = -c.x; inv(0,1) = c.w; inv(0,2) = -c.z; inv(0,3) = c.y;
			inv(1,0) = -c.y; inv(1,1) = c.z; inv(1,2) = c.w;	inv(1,3) = -c.x;
			inv(2,0) = -c.z; inv(2,1) = -c.y;inv(2,2) = c.x;  inv(2,3) = c.w;
			
			Eigen::Vector3f m = inv * dv * -2.0;
			return m.norm();
		}
	
		
		Eigen::Matrix3d reorderHandAxes(const Eigen::Matrix3d& Q)
		{
			std::vector<int> axis_order_;
			axis_order_.push_back(2);
			axis_order_.push_back(0);
			axis_order_.push_back(1);
			
			Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3, 3);
			R.col(axis_order_[0]) = Q.col(0); // grasp approach vector
			R.col(axis_order_[1]) = Q.col(1); // hand axis
			R.col(axis_order_[2]) = Q.col(2); // hand binormal
			return R;
		}
		
		geometry_msgs::PoseStamped graspToPose(agile_grasp::Grasp grasp, double hand_offset, std::string frame_id){
			
			Eigen::Vector3d center_; // grasp position
			Eigen::Vector3d surface_center_; //  grasp position projected back onto the surface of the object
			Eigen::Vector3d axis_; //  hand axis
			Eigen::Vector3d approach_; //  grasp approach vector
			Eigen::Vector3d binormal_; //  vector orthogonal to the hand axis and the grasp approach direction
			
			tf::vectorMsgToEigen(grasp.axis, axis_);
			tf::vectorMsgToEigen(grasp.approach, approach_);
			tf::vectorMsgToEigen(grasp.center, center_);
			tf::vectorMsgToEigen(grasp.surface_center, surface_center_);
			
			approach_ = -1.0 * approach_; // make approach vector point away from handle centroid
			binormal_ = axis_.cross(approach_); // binormal (used as rotation axis to generate additional approach vectors)
			
			//step 1: calculate hand orientation
			
			// rotate by 180deg around the grasp approach vector to get the "opposite" hand orientation
			Eigen::Transform<double, 3, Eigen::Affine> T_R(Eigen::AngleAxis<double>(M_PI/2, approach_));
			
			//to do: compute the second possible grasp by rotating -M_PI/2 instead
			
			// calculate first hand orientation
			Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3, 3);
			R.col(0) = -1.0 * approach_;
			R.col(1) = T_R * axis_;
			R.col(2) << R.col(0).cross(R.col(1));
					
			Eigen::Matrix3d R1 = reorderHandAxes(R);
			tf::Matrix3x3 TF1;		
			tf::matrixEigenToTF(R1, TF1);
			tf::Quaternion quat1;
			TF1.getRotation(quat1);		
			quat1.normalize();
			
			
			//use the first quaterneon for now
			tf::Quaternion quat = quat1;
			
			//angles to try
			double theta = 0.0;
			
			// calculate grasp position
			Eigen::Vector3d position;
			Eigen::Vector3d approach = -1.0 * approach_;
			if (theta != 0)
			{
				// project grasp bottom position onto the line defined by grasp surface position and approach vector
				Eigen::Vector3d s, b, a;
				position = (center_ - surface_center_).dot(approach) * approach;
				position += surface_center_;
			}
			else
				position = center_;
				
			// translate grasp position by <hand_offset_> along the grasp approach vector
			position = position + hand_offset * approach;
					
			geometry_msgs::PoseStamped pose_st;
			pose_st.header.stamp = ros::Time(0);
			pose_st.header.frame_id = frame_id;
			tf::pointEigenToMsg(position, pose_st.pose.position);
			tf::quaternionTFToMsg(quat, pose_st.pose.orientation);
			
			return pose_st;
		};	
		
		
		bool checkPlaneConflict(GraspCartesianCommand gcc, Eigen::Vector4f plane_c, float min_distance_to_plane){
			//filter 1: if too close to the plane
			pcl::PointXYZ p_a;
			p_a.x=gcc.approach_pose.pose.position.x;
			p_a.y=gcc.approach_pose.pose.position.y;
			p_a.z=gcc.approach_pose.pose.position.z;
			
			pcl::PointXYZ p_g;
			p_g.x=gcc.grasp_pose.pose.position.x;
			p_g.y=gcc.grasp_pose.pose.position.y;
			p_g.z=gcc.grasp_pose.pose.position.z;
			
			if (pcl::pointToPlaneDistance(p_a, plane_c) < min_distance_to_plane 
				|| pcl::pointToPlaneDistance(p_g, plane_c) < min_distance_to_plane){
				
				return false;
			}
			
			return true;
		};
		
	
		GraspCartesianCommand constructGraspCommand(agile_grasp::Grasp ag, float offset_approach, float offset_grasp, std::string object_cloud_frameid){
			//pose for approach
			geometry_msgs::PoseStamped p_grasp_i = segbot_arm_manipulation::grasp_utils::graspToPose(ag,offset_grasp,object_cloud_frameid);
			
			//pose for grasp
			geometry_msgs::PoseStamped p_approach_i = segbot_arm_manipulation::grasp_utils::graspToPose(ag,offset_approach,object_cloud_frameid);

			
			GraspCartesianCommand gc_i;
			gc_i.approach_pose = p_approach_i;
			gc_i.grasp_pose = p_grasp_i;
			
			return gc_i;
		}
	}
}
#endif
