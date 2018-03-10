#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/String.h>

#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>


//tf stuff
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

// PCL specific includes
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

#include <tf/transform_listener.h>
#include <tf/tf.h>


//the action definition
#include "segbot_arm_manipulation/TabletopApproachAction.h"

#include <actionlib/server/simple_action_server.h>

#include <nav_msgs/Odometry.h>

#include <segbot_arm_manipulation/Mico.h>
#include <bwi_manipulation/ArmPositionDB.h>

//for playing sounds when backing up
#include <sound_play/sound_play.h>
#include <sound_play/SoundRequest.h>
#include <bwi_perception/BoundingBox.h>
#include <bwi_perception/PerceiveTabletopScene.h>

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//any table further than this away from the sensor will not be seen (in m)
#define FILTER_Z_VALUE 1.5


class TableApproachActionServer
{
protected:
	
	ros::NodeHandle nh_;
	// NodeHandle instance must be created before this line. Otherwise strange error may occur.
	actionlib::SimpleActionServer<segbot_arm_manipulation::TabletopApproachAction> as_; 
	std::string action_name_;
	
	
	
	nav_msgs::Odometry current_odom;
	bool heard_odom;
	
	ros::Publisher pub_base_velocity;
	ros::Publisher pose_pub;
	ros::Publisher sound_pub;

	//used to compute transforms
	tf::TransformListener tf_listener;
	
	segbot_arm_manipulation::TabletopApproachResult result_;
	
	ros::Subscriber sub_odom_;
	
	//holds set of predefined positions
    segbot_arm_manipulation::Mico mico;
	
public:

  TableApproachActionServer(std::string name) :
    as_(nh_, name, boost::bind(&TableApproachActionServer::executeCB, this, _1), false),
    action_name_(name), mico(nh_)
  {

	
	//subscribe to odometry 
	sub_odom_= nh_.subscribe("/odom", 1,&TableApproachActionServer::odom_cb,this);

	//publisher for debugging purposes
	pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/segbot_table_approach_as/approach_table_target_pose", 1);
	
	//used to publish sound requests
	sound_pub = nh_.advertise<sound_play::SoundRequest>("/robotsound", 1);
	
	//velocity publisher
	pub_base_velocity = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    as_.start();
  }

  ~TableApproachActionServer(void)
  = default;

    //odom state cb
	void odom_cb(const nav_msgs::OdometryConstPtr& input){
		current_odom = *input;
		heard_odom = true;
	}
		
	double getYaw(geometry_msgs::Pose pose){
		tf::Quaternion q(pose.orientation.x, 
								pose.orientation.y, 
								pose.orientation.z, 
								pose.orientation.w);
		tf::Matrix3x3 m(q);
		
		double r, p, y;
		m.getRPY(r, p, y);
		return y;
	}

    void servo_yaw(double amount_to_turn) {
        ros::Rate r(30);

        //first, wait for odometry
        heard_odom = false;
        while (!heard_odom){
            r.sleep();
            ros::spinOnce();
        }

        double initial_yaw = getYaw(current_odom.pose.pose);
        double target_yaw = initial_yaw + amount_to_turn;
        double turn_velocity = 0.5 * amount_to_turn / 2.0;

        ROS_INFO("Turn angle = %f", amount_to_turn);


        geometry_msgs::Twist v_i;
        v_i.linear.x = 0; v_i.linear.y = 0; v_i.linear.z = 0;
        v_i.angular.x = 0; v_i.angular.y = 0;

        while (ros::ok()){
            v_i.angular.z = turn_velocity;

            pub_base_velocity.publish(v_i);

            ros::spinOnce();

            r.sleep();

            double current_yaw = getYaw(current_odom.pose.pose);

            if (fabs(current_yaw - target_yaw) < 0.05)
                break;
        }
        v_i.angular.z = 0;
        pub_base_velocity.publish(v_i);

    }

    void servo_linear(double distance_to_travel) {
        ros::Rate r(30);
        double start_odom_x = current_odom.pose.pose.position.x;
        double start_odom_y = current_odom.pose.pose.position.y;

        double x_vel = 0.15;

        geometry_msgs::Twist v_i;
        v_i.linear.x = 0; v_i.linear.y = 0; v_i.linear.z = 0;
        v_i.angular.x = 0; v_i.angular.y = 0;
        while (ros::ok()){
            double distance_traveled = sqrt(  pow(current_odom.pose.pose.position.x - start_odom_x,2) +
                                              pow(current_odom.pose.pose.position.y - start_odom_y,2));

            ROS_INFO("Distance traveled = %f",distance_traveled);

            if (distance_traveled > distance_to_travel){
                break;
            }


            v_i.linear.x = x_vel;
            pub_base_velocity.publish(v_i);

            r.sleep();
            ros::spinOnce();



        }
        v_i.linear.x = 0;
        pub_base_velocity.publish(v_i);
    }

    void approach_circular_table(const PointCloudT::Ptr &cloud_plane) {
        //find the point on the table closest to the robot's 0,0
        int closest_point_index = -1;
        double closest_point_distance = 0.0;
        for (int i = 0; i < (int)cloud_plane->points.size(); i++){
            double d_i = sqrt(   pow(cloud_plane->points.at(i).x,2) +   pow(cloud_plane->points.at(i).y,2));
            if (closest_point_index == -1 || d_i < closest_point_distance){
                closest_point_index = i;
                closest_point_distance = d_i;
            }
        }

        geometry_msgs::PoseStamped pose_debug;
        pose_debug.header.frame_id = "/base_footprint";
        pose_debug.header.seq = 1;
        pose_debug.pose.position.x = cloud_plane->points.at(closest_point_index).x;
        pose_debug.pose.position.y = cloud_plane->points.at(closest_point_index).y;
        pose_debug.pose.position.z = cloud_plane->points.at(closest_point_index).z;
        pose_debug.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
        pose_pub.publish(pose_debug);


        //next, make arm safe to move again
        mico.move_home();
        bool safe = mico.make_safe_for_travel();
        if (!safe) {
            ROS_ERROR("[segbot_table_approach_as.cpp] Cannot make arm safe for travel! Aborting!");
            result_.success = false;
            result_.error_msg = "cannot_make_arm_safe";
            as_.setAborted(result_);
            return;
        }


        //calculate turn angle
        double target_turn_angle = atan2( cloud_plane->points.at(closest_point_index).y, cloud_plane->points.at(closest_point_index).x);
        servo_yaw(target_turn_angle);

        heard_odom = false;
        while (!heard_odom){
            ros::spinOnce();
        }
        double final_yaw = getYaw(current_odom.pose.pose);


        //now, approach table
        double distance_to_travel = closest_point_distance - 0.25;
        servo_linear(distance_to_travel);
    }

	void approach_rectangular_table(const PointCloudT::Ptr &cloud_plane) {
		auto bounding_box = bwi_perception::BoundingBox::oriented_from_cloud<PointT>(cloud_plane);

		geometry_msgs::PoseStamped pose_debug;
		pose_debug.header.frame_id = "/base_footprint";
		pose_debug.header.seq = 1;
		pose_debug.pose.position.x = bounding_box.position.x();
		pose_debug.pose.position.y = bounding_box.min.y();
		pose_debug.pose.position.z = bounding_box.position.z();
		pose_debug.pose.orientation.x = bounding_box.orientation.x();
		pose_debug.pose.orientation.y = bounding_box.orientation.y();
		pose_debug.pose.orientation.z = bounding_box.orientation.z();
		pose_debug.pose.orientation.w = bounding_box.orientation.w();
		pose_pub.publish(pose_debug);


	}

	void executeCB(const segbot_arm_manipulation::TabletopApproachGoalConstPtr &goal)
	{
		if (goal->command == "approach"){
		
			//step 1: query table_object_detection_node to segment the blobs on the table

			//home the arm
			mico.move_home();
			mico.close_hand();
	

            mico.move_to_named_joint_position(segbot_arm_manipulation::Mico::side_view_position_name);
			
			
			ros::ServiceClient client_tabletop_perception = nh_.serviceClient<bwi_perception::PerceiveTabletopScene>("perceive_tabletop_scene");
			bwi_perception::PerceiveTabletopScene srv; //the srv request is just empty
			
			srv.request.override_filter_z = true;
			srv.request.max_z_value = FILTER_Z_VALUE;
			
			if (client_tabletop_perception.call(srv))
			{
				ROS_INFO("Received Response from tabletop_object_detection_service");
			}
			else
			{
				ROS_ERROR("Failed to call perception service");
				result_.success = false;
				result_.error_msg = "cannot_call_tabletop_perception";
				as_.setSucceeded(result_);
				return;
			}
			
			Eigen::Vector4f plane_coef_vector;
			for (int i = 0; i < 4; i ++)
				plane_coef_vector(i)=srv.response.cloud_plane_coef[i];

			if (srv.response.is_plane_found == false){
				ROS_ERROR("[segbot_table_approach_as.cpp] Table not found. The end.");
				result_.success = false;
				result_.error_msg = "table_not_found";
				as_.setAborted(result_);
				return;
			}
			
			ROS_INFO("Wait for transform...");
			tf_listener.waitForTransform(srv.response.cloud_plane.header.frame_id, "/base_footprint", ros::Time(0.0), ros::Duration(30.0)); 
			ROS_INFO("..done!");
			
			//transform clound into base_link frame of reference
			sensor_msgs::PointCloud cloud_pc1;
			sensor_msgs::convertPointCloud2ToPointCloud(srv.response.cloud_plane,cloud_pc1);
			
			sensor_msgs::PointCloud transformed_cloud;
			tf_listener.transformPointCloud("/base_footprint",cloud_pc1,transformed_cloud);	
					
			//convert back to sensor_msgs::PointCloud2 and then to pcl format
			sensor_msgs::PointCloud2 plane_cloud_pc2;
			sensor_msgs::convertPointCloudToPointCloud2(transformed_cloud,plane_cloud_pc2);
			
			PointCloudT::Ptr cloud_plane (new PointCloudT);
			pcl::fromROSMsg(plane_cloud_pc2, *cloud_plane);

            if (goal->table_type == "circular") {
                approach_circular_table(cloud_plane);
            } else if (goal->table_type == "rectangular") {
                approach_rectangular_table(cloud_plane);
            } else {
				ROS_ERROR("Specify a table type");
				result_.success = false;
				as_.setAborted(result_);
			}


			
			result_.success = true;
			as_.setSucceeded(result_);
		}
		else if (goal->command == "back_out"){
			//store current odom
			heard_odom = false;
			
			float vel_pub_rate = 30.0;
			ros::Rate r(vel_pub_rate);
			while (!heard_odom){
				r.sleep();
				ros::spinOnce();
			}
			nav_msgs::Odometry start_odom = current_odom;
			
			float distance_to_travel = 0.25;
			
			double start_odom_x = current_odom.pose.pose.position.x;
			double start_odom_y = current_odom.pose.pose.position.y;
			
			double x_vel = -0.15;
			
			
			
			geometry_msgs::Twist v_i;
			v_i.linear.x = 0; v_i.linear.y = 0; v_i.linear.z = 0;
			v_i.angular.x = 0; v_i.angular.y = 0;
			
			//create sound client and play sound
			
			sound_play::SoundRequest sound_msg;
			sound_msg.sound = 1; //back out sound
			sound_msg.command = 1; //play the sound
			
			//start playing sound to warn people of robot going backwards
			ros::Rate r_sound(1);
			for (int i = 0; i < 4; i ++){
				sound_pub.publish(sound_msg);
				r_sound.sleep();
			}
			
			int c = 0;
			while (ros::ok()){
				double distance_traveled = sqrt(  pow(current_odom.pose.pose.position.x - start_odom_x,2) +
												pow(current_odom.pose.pose.position.y - start_odom_y,2));
												
				
				ROS_INFO("Distance traveled = %f",distance_traveled);				
												
				if (distance_traveled > distance_to_travel){
					break;
				}
				
				
				v_i.linear.x = x_vel;
				pub_base_velocity.publish(v_i);
				
				c++;
				if (c > vel_pub_rate){ //i.e., 1 second has passed
					sound_pub.publish(sound_msg);
					c = 0;
				}
				
				r.sleep();
				ros::spinOnce();
				
				
			}
			v_i.linear.x = 0;
			pub_base_velocity.publish(v_i);			
			
			result_.success = true;
			as_.setSucceeded(result_);
		}
			
				
	}


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "segbot_table_approach_as");

  TableApproachActionServer as(ros::this_node::getName());
  ros::spin();

  return 0;
}


