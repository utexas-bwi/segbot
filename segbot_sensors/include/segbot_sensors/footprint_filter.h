#ifndef LASER_SCAN_FOOTPRINT_FILTER_H
#define LASER_SCAN_FOOTPRINT_FILTER_H

#include <tf/transform_listener.h>
#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>
#include <XmlRpcException.h>

namespace segbot_sensors
{
  class SegbotFootprintFilter : public filters::FilterBase<sensor_msgs::LaserScan>
  {
    public:
      std::vector<tf::Point> footprint_;
      std::string footprint_frame_;
      bool get_transformation_;
      boost::shared_ptr<tf::TransformListener> listener_;

      bool configure() {

        listener_.reset(new tf::TransformListener());

        // Get the frame of reference here. The sensor should not move w.r.t to 
        // this frame of reference
        get_transformation_ = true;
        getParam("footprint_frame", footprint_frame_);
        if (footprint_frame_.empty()) {
          ROS_WARN("No footprint frame provided, assuming footprint is in the frame of reference of the sensor.");
          get_transformation_ = false;
          return false;
        }

        XmlRpc::XmlRpcValue footprint_list;
        getParam("footprint", footprint_list);
        ROS_ASSERT(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

        try {
          for (int32_t i = 0; i < footprint_list.size() - 1; i+=2) {
            float valx, valy;
            if (footprint_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) 
              valx = static_cast<double>(footprint_list[i]);
            else if (footprint_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
              valx = static_cast<int>(footprint_list[i]);
            else {
              ROS_FATAL("Footprint should be a list of numeric values");
              return false;
            }
            if (footprint_list[i+1].getType() == XmlRpc::XmlRpcValue::TypeDouble) 
              valy = static_cast<double>(footprint_list[i+1]);
            else if (footprint_list[i+1].getType() == XmlRpc::XmlRpcValue::TypeInt)
              valy = static_cast<int>(footprint_list[i+1]);
            else {
              ROS_FATAL("Footprint should be a list of numeric values");
              return false;
            }
            footprint_.push_back(tf::Point(valx, valy, 0));
          }
        } catch (XmlRpc::XmlRpcException ex) {
          ROS_FATAL_STREAM(ex.getMessage());
          return false;
        }
        
        if(footprint_.size() < 3) {
          ROS_FATAL("Footprint needs to contain atleast 3 vertices.");
          return false;
        }

        return true;
      }

      virtual ~SegbotFootprintFilter(){}

      bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan){

        // Get the transformation the first time only
        if (get_transformation_) {
          try {
            tf::StampedTransform transform;
            listener_->waitForTransform(footprint_frame_, input_scan.header.frame_id, ros::Time(0), ros::Duration(10.0) );
            listener_->lookupTransform(footprint_frame_, input_scan.header.frame_id, ros::Time(0), transform);
            for (uint32_t i = 0; i < footprint_.size(); ++i) {
              footprint_[i] = transform * footprint_[i]; 
            }
            
          } catch (tf::TransformException ex) {
            ROS_ERROR("Error getting transformation. The filtered scan is probably incorrect: %s",ex.what());
          }
          get_transformation_ = false;
        }

        filtered_scan.ranges.resize(input_scan.ranges.size());
        double current_angle = input_scan.angle_min;

        for(unsigned int i = 0; i < input_scan.ranges.size(); ++i){
          current_angle += input_scan.angle_increment;
          if (input_scan.range_min <= input_scan.ranges[i] && 
              input_scan.ranges[i] <= input_scan.range_max) {
            // Valid measurement, check if it lies in the footprint
            // http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
            float testx = cosf(current_angle) * input_scan.ranges[i];
            float testy = sinf(current_angle) * input_scan.ranges[i];
            int i, j, c = 0;
            int nvert = footprint_.size(); 
            for (i = 0, j = nvert-1; i < nvert; j = i++) {
              if ( ((footprint_[i].y()>testy) != (footprint_[j].y()>testy)) &&
               (testx < (footprint_[j].x()-footprint_[i].x()) * (testy-footprint_[i].y()) / (footprint_[j].y()-footprint_[i].y()) + footprint_[i].x()) )
                 c = !c;
            }
            // in accordance with REP 117
            filtered_scan.ranges[i] = (c) ? input_scan.ranges[i] : std::numeric_limits<float>::quiet_NaN();
          }
        }

        //make sure to set all the needed fields on the filtered scan
        filtered_scan.header.frame_id = input_scan.header.frame_id;
        filtered_scan.header.stamp = input_scan.header.stamp;
        filtered_scan.angle_min = input_scan.angle_min;
        filtered_scan.angle_max = input_scan.angle_max;
        filtered_scan.angle_increment = input_scan.angle_increment;
        filtered_scan.time_increment = input_scan.time_increment;
        filtered_scan.scan_time = input_scan.scan_time;
        filtered_scan.range_min = input_scan.range_min;
        filtered_scan.range_max = input_scan.range_max;
        filtered_scan.intensities = input_scan.intensities;

        return true;

      }
  };
}
#endif

