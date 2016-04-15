#ifndef SEGBOT_SENSORS_VELODYNE_FILTER_H_
#define SEGBOT_SENSORS_VELODYNE_FILTER_H_

#include <geometry_msgs/Point.h>

// PCL includes
#include <pcl_ros/filters/filter.h>

// Dynamic reconfigure
#include <segbot_sensors/SegbotVelodyneOutlierRemovalConfig.h>

namespace segbot_sensors
{
  class SegbotVelodyneOutlierRemoval : public pcl_ros::Filter
  {
    protected:
      /** \brief Pointer to a dynamic reconfigure service. */
      boost::shared_ptr <dynamic_reconfigure::Server<SegbotVelodyneOutlierRemovalConfig> > srv_;

      /** \brief Call the actual filter. 
        * \param input the input point cloud dataset
        * \param indices the input set of indices to use from \a input
        * \param output the resultant filtered dataset
        */
      inline void
      filter (const PointCloud2::ConstPtr &input, const IndicesPtr &indices, 
              PointCloud2 &output)
      {
        frame_ = input->header.frame_id;
        boost::mutex::scoped_lock lock (mutex_);
        pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(*input, *pcl_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*pcl_cloud, *cloud);

        for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end();) {
          // http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
          float &x = it->x;
          float &y = it->y;
          bool point_not_in_polygon = false;
          for (int i = 0, j = footprint_.size() - 1; i < footprint_.size(); j = i++) {
            if (((footprint_[i].y > y) != (footprint_[j].y > y)) &&
                (x < (footprint_[j].x - footprint_[i].x) * (y - footprint_[i].y) / (footprint_[j].y - footprint_[i].y) + footprint_[i].x)) {
              point_not_in_polygon = !point_not_in_polygon;
            }
          }
          if (point_not_in_polygon) {
            it = cloud->erase(it);
          } else {
            // Keep the point.
            ++it;
          }
        }

        pcl::toPCLPointCloud2(*cloud, *pcl_cloud);
        pcl_conversions::moveFromPCL(*pcl_cloud, output);
      }

      /** \brief Child initialization routine.
        * \param nh ROS node handle
        * \param has_service set to true if the child has a Dynamic Reconfigure service
        */
      bool child_init (ros::NodeHandle &nh, bool &has_service);

      /** \brief Dynamic reconfigure callback
        * \param config the config object
        * \param level the dynamic reconfigure level
        */
      void config_callback (segbot_sensors::SegbotVelodyneOutlierRemovalConfig &config, uint32_t level);

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
      std::vector<geometry_msgs::Point> footprint_;
      ros::Publisher footprint_publisher_;
      std::string frame_;
  };
}

#endif
