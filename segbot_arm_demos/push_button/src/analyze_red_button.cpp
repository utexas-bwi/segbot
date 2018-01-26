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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob2(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    std::string pathName = "/nishome/bwi/catkin_ws/src/segbot_arm/segbot_arm_perception/pcd/red_button_0.pcd";
    reader.read (pathName, *cloud_blob2);

    std::cerr << "Start printing points..." << std::endl;
    std::cerr << "num points = " << cloud_blob2->points.size() << std::endl;
    for (int i = 0; i < cloud_blob2->points.size(); i++) {
        std::cerr << (unsigned int)(cloud_blob2->points[i].r) << "\t"
                  << (unsigned int)(cloud_blob2->points[i].g) << "\t"
                  << (unsigned int)(cloud_blob2->points[i].b) << "\t"
                  << std::endl;
    }

    return 0;

    // Visualize
    pcl::visualization::CloudViewer viewer2 ("plane segmentation");
    viewer2.showCloud (cloud_blob2);
    while (!viewer2.wasStopped ());

    return 0;
}
