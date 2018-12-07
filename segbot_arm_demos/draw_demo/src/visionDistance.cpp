/*
Subscribes to kinect pc data, downsamples, and publishes. 
*/


#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <opencv2/highgui/highgui.hpp> //not supported in opcv3
//#include "opencv2/imgproc/imgproc.hpp"


ros::Publisher pub;
bool capture;
bool seg = true;
void 
cloud_cb (const sensor_msgs::ImageConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

 /*cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }*/
 
//SEGMENTATION BELOW
  if(seg){
 
 // Do data processing here...
  //output = *input;


  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ> ());
  // Read in the cloud data
//  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
//  reader.read ("table4.pcd", *cloud);
    pcl::fromROSMsg(output, *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //* 

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }
  cloud_final = cloud_plane; //this captures the table. 2nd largest contiguous segment
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.0105);
  ec.setMinClusterSize (1000);
  ec.setMaxClusterSize (1000000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    *cloud_final += *cloud_cluster;
    j++;
  }

  pcl::toROSMsg(*cloud_final, output);
  }
  /*else{


  cv::Mat src_gray;
  /// Convert it to gray
  cvtColor( cv_ptr->image, src_gray, cv::COLOR_RGB2GRAY );

  /// Reduce the noise so we avoid false circle detection
  GaussianBlur( src_gray, src_gray, cv::Size(9, 9), 2, 2 );

  std::vector<cv::Vec3f> circles;

  /// Apply the Hough Transform to find the circles
  HoughCircles( src_gray, circles, cv::HOUGH_GRADIENT, 2, 20, 85, 100, 0, 30 );


   cv::Point ur, ul, lr, ll;
   ul.x = 10000;
   ul.y = 10000;
   ur.x = 10000;
   ur.y = 10000;
   lr.x = 0;
   lr.y = 0;
   ll.x = 10000;
   ll.y = 10000;



  /// Draw the circles detected
   int radius;
  for( size_t i = 0; i < circles.size(); i++ )
  {
      cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      std::cout << "X: " << circles[i][0] << " Y: " << circles[i][1] << std::endl;
      radius = cvRound(circles[i][2]);
      // circle center
      circle( cv_ptr->image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
      // circle outline
      circle( cv_ptr->image, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );

      if(circles.size() > 1)
      for(int g = 0; g < circles.size(); g++){
        if(circles[i][0] < ul.x && circles[i][1] < ul.y)
          ul = cv::Point(circles[i][0], circles[i][1]);
        
        else if(circles[i][0] > lr.x && circles[i][0] > ul.x && circles[i][1] > lr.y && circles[i][1] > ul.y)
          lr = cv::Point(circles[i][0], circles[i][1]);
        }


   }
   std::cout << "Number of circles detected: " << circles.size() << std::endl;
  std::cout << "ul " << ul.x << " " << ul.y;
    std::cout << "lr " <<  lr.x << " " << lr.y;

  //Mat cropedImage = src(Rect(ul.x + radius,ul.y + radius,lr.x - ul.x - (2*radius), lr.y - ul.y - (2*radius)));

  /// Show your results
   if(capture && circles.size() >= 2){
   	cv::Mat croppedImage = src_gray(cv::Rect(ul.x + radius,ul.y + radius,lr.x - ul.x - (2*radius), lr.y - ul.y - (2*radius)));

   	//cv::imshow("OPENCV_WINDOW", cv_ptr->image);
   	cv::imshow("Cropped image window", croppedImage);
   	capture = false;
   }
	cv::imshow("Circle Detection Window", cv_ptr->image);


   cv::waitKey(30);
*/

  // Publish the data.
  pub.publish (output);
  
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  image_transport::ImageTransport it_(nh);
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  image_sub_ = it_.subscribe("camera/rgb/image_color", 1, cloud_cb);

  //allows user to trigger image capture
  /*std::cout << "Please enter '1' to capture the next image that contains two circles, one in diagonal corners." << std::endl;
  char in = '0';
  while(ros::ok() && in != '1'){
  	std::cin >> in;
  	if(in == '1')
  		capture = true;
  	ros::spinOnce();
  }
  std::cout << "Capturing the next available image." << std::endl;
*/
  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe ("camera/rgb/image_color", 1, cloud_cb);
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("segmented", 1);

  // Spin
  ros::spin ();
}
