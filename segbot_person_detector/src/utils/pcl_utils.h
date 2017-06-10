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

#include <signal.h> 
#include <vector>
#include <string.h>


namespace pcl_utils {

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

}
