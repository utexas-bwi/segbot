/*
 *
 * 
 * 
 * 
 * Implements the people detection algorithm described here:
 * M. Munaro, F. Basso and E. Menegatti,
 * Tracking people within groups with RGB-D data,
 * In Proceedings of the International Conference on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.
 */
 
#include <signal.h> 
#include <vector>
#include <string>
#include <ros/ros.h>

#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <pcl_ros/impl/transforms.hpp>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>
#include <pcl/filters/crop_box.h>

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//some custom functions
//#include "utils/file_io.h"
//#include "utils/viz_utils.h" 
//#include "utils/pcl_utils.h"
 

 
//some constants
bool visualize = false;
bool calibrate_plane = false;

const std::string data_topic = "nav_kinect/depth_registered/points"; 
const std::string classifier_location = ros::package::getPath("pcl_perception") + "/classifier.yaml";
const std::string node_name = "segbot_people_detector";

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

//refresh rate
double ros_rate = 10.0;


Eigen::VectorXf ground_coeffs;


// Mutex: //
boost::mutex cloud_mutex;


bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud (new PointCloudT);
PointCloudT::Ptr person_cloud (new PointCloudT);
sensor_msgs::PointCloud2 person_cloud_ros;

void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	cloud_mutex.lock (); 
	
	//convert to PCL format
	pcl::fromROSMsg (*input, *cloud);

	//state that a new cloud is available
	new_cloud_available_flag = true;
	
	cloud_mutex.unlock ();
}

struct callback_args{
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};


void write_vector_to_file(const char *filename, Eigen::VectorXf V, int n){
	FILE *fp = fopen(filename,"w");	
	for (int j = 0; j < n; j++)
		fprintf(fp,"%f\t",V(j));
	fprintf(fp,"\n");
	fclose(fp);
}

Eigen::VectorXf load_vector_from_file(const char *filename, int n){
	Eigen::VectorXf c;
	c.resize(n);
		
	FILE *fp = fopen(filename,"r");
		
	if (fp == NULL){
		ROS_WARN("File %s does not exist!",filename);
		return c;
	}
	
	for (int j = 0; j < n; j++){
		float f;
		int r = fscanf(fp,"%f\t",&f);
		if (r > 0)
			c(j)=f;
	}
		
	return c;
}

visualization_msgs::Marker create_next_person_marker(std::vector<pcl::people::PersonCluster<PointT> >::iterator it, 
					std::string frame_id,
					std::string ns,
					int k){
	
	Eigen::Vector3f centroid_k = it->getCenter();
	Eigen::Vector3f bottom_k = it->getBottom();	
	
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "people";
	marker.id = k;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::CUBE;

	// Set the marker action.  Options are ADD and DELETE
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = bottom_k(0);
	marker.pose.position.y = 0.0;
	marker.pose.position.z = bottom_k(2);
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 1.0;
	marker.pose.orientation.w = 0.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.25;
	marker.scale.y = 1.55;
	marker.scale.z = 0.25;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 0.8;

	marker.lifetime = ros::Duration(0.0);

	return marker;
}

void applyBoxFilter(Eigen::Vector3f box_min, Eigen::Vector3f box_max, pcl::PointCloud<PointT>::Ptr cloudId, pcl::PointCloud<PointT>::Ptr cloudOut){
		Eigen::Vector4f minPoint;
		minPoint[0]=box_min[0];  // define minimum point x
		minPoint[1]=box_min[1];  // define minimum point y
		minPoint[2]=box_min[2];  // define minimum point z
		Eigen::Vector4f maxPoint;
		maxPoint[0]=box_max[0];  // define max point x
		maxPoint[1]=box_max[1];  // define max point y
		maxPoint[2]=box_max[2];  // define max point z 
						
		pcl::CropBox<PointT> cropFilter;
		cropFilter.setInputCloud (cloudId);
		cropFilter.setMin(minPoint);
		cropFilter.setMax(maxPoint);
		cropFilter.filter (*cloudOut); 
}


void
pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
  struct callback_args* data = (struct callback_args *)args;
  if (event.getPointIndex () == -1)
    return;
  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->clicked_points_3d->points.push_back(current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}


void calibrate_floor(){
	ROS_INFO("[main_ground_based_people_detection.cpp] Waiting for next cloud...");
	while(!new_cloud_available_flag) {
		//collect messages
		ros::spinOnce();
	}

	ROS_INFO("[main_ground_based_people_detection.cpp] Heard first cloud, calibrating ground plane...");
	  
	new_cloud_available_flag = false;

	cloud_mutex.lock ();    // for not overwriting the point cloud

	// Display pointcloud:
	pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
		
	// PCL viewer //
	pcl::visualization::PCLVisualizer viewer_calibrate("PCL Viewer");
	viewer_calibrate.addPointCloud<PointT> (cloud, rgb, "input_cloud");
	viewer_calibrate.setCameraPosition(0,0,-2,0,-1,0,0);

	// Add point picking callback to viewer:
	struct callback_args cb_args;
	PointCloudT::Ptr clicked_points_3d (new PointCloudT);
	cb_args.clicked_points_3d = clicked_points_3d;
	cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer_calibrate);
	viewer_calibrate.registerPointPickingCallback (pp_callback, (void*)&cb_args);
	std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

	// Spin until 'Q' is pressed:
	viewer_calibrate.spin();
	std::cout << "done." << std::endl;
	  
	cloud_mutex.unlock ();    

	// Ground plane estimation:
		
	std::vector<int> clicked_points_indices;
	for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
		clicked_points_indices.push_back(i);
	pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
	model_plane.computeModelCoefficients(clicked_points_indices,ground_coeffs);
	//save to file
	write_vector_to_file("ground_plane.txt",ground_coeffs,4);
	
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "segbot_background_person_detector");
	ros::NodeHandle nh;
	
	
	nh.param<bool>("background_person_detector/visualize", visualize, false);
	nh.param<double>("background_person_detector/rate", ros_rate, 10.0);
	
	string param_out_frame_id;
	nh.param<std::string>(std::string("background_person_detector/out_frame_id"), param_out_frame_id, "/map");

	string param_topic;
	nh.param<std::string>(std::string("background_person_detector/rgbd_topic"), param_topic, data_topic);
	
	string param_classifier;
	nh.param<std::string>(std::string("background_person_detector/classifier_location"), 
							param_classifier, 
							ros::package::getPath("pcl_perception")+"/data/classifier.yaml");
	
	
	string param_sensor_frame_id;
	nh.param<std::string>(std::string("background_person_detector/sensor_frame_id"), 
							param_sensor_frame_id, 
							"/nav_kinect_rgb_optical_frame");
	
	
	//nh.getParam("background_person_detector/rgbd_topic", data_topic);
	
	
	//initialize marker publisher
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("segbot_person_detector/marker", 10);
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("segbot_person_detector/human_poses", 10);
	ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("segbot_person_detector/human_clouds", 10);
	  
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe (param_topic, 1, cloud_cb);

	// Algorithm parameters:
	std::string svm_filename = param_classifier;
	float min_confidence = -1.5;//-1.9
	float min_height = 1.3;
	float max_height = 2.3;
	float min_width = 0.2;
	float max_width = 1.0;
	float voxel_size = 0.06;
	Eigen::Matrix3f rgb_intrinsics_matrix;
	rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics

	//register ctrl-c
	signal(SIGINT, sig_handler);

	//load ground plane coeffs
	ground_coeffs.resize(4);
	std::string ground_plane_file, path_to_package, path; 

	if (false == ros::param::has("~ground_plane_file")) {
		ROS_ERROR("ground_plane_file parameter needs to be set");
		ros::shutdown(); 
	} 
	else {
		ros::param::get("~ground_plane_file", ground_plane_file); 
	}

	string plane_coefs_location = ros::package::getPath("segbot_person_detector")+"/data/"+ground_plane_file;
	ground_coeffs = load_vector_from_file(plane_coefs_location.c_str(),4);

	
	// Initialize new viewer:
	pcl::visualization::PCLVisualizer *viewer_display;          // viewer initialization
	if (visualize){
		viewer_display = new pcl::visualization::PCLVisualizer("People Viewer"); 
		viewer_display->setCameraPosition(0,0,-2,0,-1,0,0);
	}

	// Create classifier for people detection: 
	pcl::people::PersonClassifier<pcl::RGB> person_classifier;
	person_classifier.loadSVMFromFile(param_classifier);   // load trained SVM

	// People detection app initialization:
	pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
	people_detector.setVoxelSize(voxel_size);                        // set the voxel size
	people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
	people_detector.setClassifier(person_classifier);                // set person classifier
	people_detector.setPersonClusterLimits(min_height, max_height, min_width, max_width);         // set person classifier
//  people_detector.setSensorPortraitOrientation(true);             // set sensor orientation to vertical

	// For timing:
	static unsigned count = 0;
	static double last = pcl::getTime ();

	//
	int detection_count=0;
	bool set_ground = false;
	
	ros::Rate r(ros_rate);

	tf::TransformListener listener;
	tf::StampedTransform transform;

	// Main loop:
	while (!g_caught_sigint && ros::ok())
	{
		//collect messages
		ros::spinOnce();
		
		r.sleep();
		
		if (new_cloud_available_flag && cloud_mutex.try_lock ())    // if a new cloud is available
		{
			new_cloud_available_flag = false;

			// Perform people detection on the new cloud:
			std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
			std::vector<pcl::people::PersonCluster<PointT> > clusters_filtered;
			people_detector.setInputCloud(cloud);
			people_detector.setGround(ground_coeffs);    
			
			people_detector.compute(clusters);                           // perform people detection

			ground_coeffs = people_detector.getGround();                 // get updated floor coefficients

			// Draw cloud and people bounding boxes in the viewer:
			if (visualize){
				viewer_display->removeAllPointClouds();
				viewer_display->removeAllShapes();
				pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
				viewer_display->addPointCloud<PointT> (cloud, rgb, "input_cloud");
			}
			
			unsigned int k = 0;
			for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
			{
				if(it->getPersonConfidence() > min_confidence)             // draw only people with confidence above a threshold
				{
					
					Eigen::Vector3f centroid_k = it->getCenter();
					Eigen::Vector3f top_k = it->getTop();
					Eigen::Vector3f bottom_k = it->getBottom();	
						
					//calculate the distance from the centroid of the cloud to the plane
					pcl::PointXYZ p_k;
						
					p_k.x=bottom_k(0);p_k.y=bottom_k(1);p_k.z=bottom_k(2);
					double dist_to_ground_bottom = pcl::pointToPlaneDistance(p_k,ground_coeffs);
							
					p_k.x=top_k(0);p_k.y=top_k(1);p_k.z=top_k(2);
					double dist_to_ground_top = pcl::pointToPlaneDistance(p_k,ground_coeffs);
							
					p_k.x=centroid_k(0);p_k.y=centroid_k(1);p_k.z=centroid_k(2);
					double dist_to_ground = pcl::pointToPlaneDistance(p_k,ground_coeffs);
							
					/*ROS_INFO("Cluter centroid: %f, %f, %f",centroid_k(0),centroid_k(1),centroid_k(2));
					ROS_INFO("\tDistance to ground (top): %f",dist_to_ground_top);
					ROS_INFO("\tDistance to ground (centroid): %f",dist_to_ground);
					ROS_INFO("\tDistance to ground (bottom): %f",dist_to_ground_bottom);
					ROS_INFO("\tCluster height: %f",it->getHeight());
					ROS_INFO("\tCluster points: %i",it->getNumberPoints());
					ROS_INFO("\tDistance from sensor: %f",it->getDistance());	
					ROS_INFO("\tconfidence: %f",it->getPersonConfidence());	*/

					bool accept = true;
						
					if (it->getNumberPoints() < 250) //a person should have about 350 points +- 50 depending on distance from kinect
						accept = false;
					else if (it->getNumberPoints() > 600) //a person should have about 450 points +- 50 depending on distance from kinect
						accept = false;
					else if (it->getHeight() < 1.1) //nobody should be shorter than a meter and 10 cm
						accept = false;
					else if (it->getHeight() > 2.2) //or taller than 2.2 meters
						accept = false;
					if (dist_to_ground_bottom > 0.3) //or hovering more than 30 cm over the floor
						accept = false;
						
							
					if (accept){
					
					
						// draw theoretical person bounding box in the PCL viewer:
						if (visualize)
							it->drawTBoundingBox(*viewer_display, k);
						
						//get just the person out of the whole cloud
						applyBoxFilter(it->getMin(), it->getMax(),cloud,person_cloud);
						
						//publish person cloud
						pcl::toROSMsg(*person_cloud,person_cloud_ros);
						person_cloud_ros.header.frame_id = param_sensor_frame_id;
						cloud_pub.publish(person_cloud_ros);	
				
						//transforms the pose into /map frame
						geometry_msgs::Pose pose_i;
						pose_i.position.x=centroid_k(0);
						pose_i.position.y=0.5;
						pose_i.position.z=centroid_k(2);
						pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-3.14/2);
						
						
						geometry_msgs::PoseStamped stampedPose;

						stampedPose.header.frame_id = param_sensor_frame_id;
						stampedPose.header.stamp = ros::Time(0);
						stampedPose.pose = pose_i;
						
						geometry_msgs::PoseStamped stampOut;
						listener.waitForTransform(param_sensor_frame_id, param_out_frame_id, ros::Time(0), ros::Duration(3.0)); 
						listener.transformPose(param_out_frame_id, stampedPose, stampOut);

						
						//transform the human point cloud into presumably the /map frame of reference
						pcl_ros::transformPointCloud (param_out_frame_id, person_cloud_ros, person_cloud_ros, listener);
						
						//save to file for analysis
						ros::Time nowTime = ros::Time::now();
						

						stringstream ss;
						ss << ros::package::getPath("pcl_perception") << "/data/human_kinect_" << nowTime.toNSec() << ".pcd";
						pcl::io::savePCDFileASCII (ss.str(), *person_cloud);
					
						//save cloud in map frame of reference
						pcl::fromROSMsg(person_cloud_ros,*person_cloud);
						ss.str(std::string());
						ss << ros::package::getPath("pcl_perception") << "/data/human_map_" << nowTime.toNSec() << ".pcd";
						pcl::io::savePCDFileASCII (ss.str(), *person_cloud);
						
						stampOut.pose.position.z = 0.7;
						stampOut.header.stamp = nowTime;
						
						//publish the marker
						visualization_msgs::Marker marker_k = create_next_person_marker(it,param_out_frame_id,"segbot_pcl_person_detector",detection_count);	
						marker_k.pose = stampOut.pose;
						marker_pub.publish(marker_k);
						
						//publish the pose
						stampOut.pose.position.z = 0.0;
						pose_pub.publish(stampOut);
		
						k++;
						
						detection_count++;
					}	
				}	
			}
						
			if (visualize){
				viewer_display->spinOnce();
			}

			cloud_mutex.unlock ();
		}
	}

	return 0;
}

