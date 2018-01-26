#include <iostream>
#include <vector>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

#include <ros/ros.h>
#include <ros/package.h>

int
pressEntertoContinue (void)
{
    std::cout << "Press ENTER to continue..." << std::endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    return 0;
}


std::vector<float>
findZRange (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
    // TODO need to check for empty pointcloud case
    std::vector<float> z_range(2); // min = [0], max = [1]
    int i;

    for (i = 0; i < input_cloud->points.size(); i++)
    {
        if (i == 0)
        {
            z_range[0] = input_cloud->points[i].z;
            z_range[1] = input_cloud->points[i].z;
        }
        if (input_cloud->points[i].z < z_range[0]) // Min
        {
            z_range[0] = input_cloud->points[i].z;
        }
        if (input_cloud->points[i].z > z_range[1]) // Max
        {
            z_range[1] = input_cloud->points[i].z;
        }
    }
    std::cout << "range = [" << z_range[0] << ", " << z_range[1] << "]" << endl;
    return z_range;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
zFilter (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, float min_ratio, float max_ratio)
{
    std::vector<float> z_range = findZRange(input_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;

    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_ratio * z_range[0], max_ratio * z_range[1]);
    pass.filter(*filtered_cloud);
    return filtered_cloud;
}


int
main (int argc, char** argv)
{
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    pcl::PCDReader reader;
//    std::string pathName = ros::package::getPath("push_button");
    std::string pathName = "/nishome/bwi/catkin_ws/src/segbot_arm/demos/push_button/src/table_scene_lms400.pcd";
    reader.read (pathName, *cloud_blob);

    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud_blob);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered_blob);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    // Write the downsampled version to disk
    pcl::PCDWriter writer;
    std::string pathNameWrite = "/nishome/bwi/catkin_ws/src/segbot_arm/demos/push_button/src/table_scene_lms400_downsampled.pcd";
    writer.write<pcl::PointXYZ> (pathNameWrite, *cloud_filtered, false);




    // std::cerr << "Start Cloud Viewer..." << std::endl;
    // std::cerr << "Original point cloud" << std::endl;
    // pcl::visualization::CloudViewer viewer ("plane segmentation");
    // viewer.showCloud (cloud_filtered);
    // while (!viewer.wasStopped ());
    // pressEntertoContinue();
    // Apply z-filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_filtered2 = zFilter(cloud_filtered, 0.70, 1);
    std::cerr << "Start Cloud Viewer..." << std::endl;
    std::cerr << "z-filtered point cloud" << std::endl;
    pcl::visualization::CloudViewer viewer2 ("plane segmentation");
    viewer2.showCloud (cloud_filtered2);
    while (!viewer2.wasStopped ());
    return 0;



    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int) cloud_filtered->points.size ();
    // While 30% of the original cloud is still there
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        // Start visualizing the current plane
        std::cerr << "Start Cloud Viewer..." << std::endl;
        pcl::visualization::CloudViewer viewer ("plane segmentation");
        viewer.showCloud (cloud_p);
        while (!viewer.wasStopped ());

        std::stringstream ss;
        ss << "table_scene_lms400_plane_" << i << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_filtered.swap (cloud_f);
        i++;
    }

    return (0);
}
