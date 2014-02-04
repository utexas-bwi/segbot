/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2014, Jack O'Quin
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/


/** @file

@brief ROS node for converting RangeArray messages to PointCloud2.

@par Subscribes

 - @b sensor_ranges topic (segbot_sensors/RangeArray) a collection of
   sensor_msgs/Range information for each ultrasound and IR sensor.

@par Advertises

 - @b range_points topic (sensor_msgs/PointCloud2) converted points.

*/

#include <string>
#include <limits>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <pcl_ros/point_cloud.h>
#include "ranges_to_cloud.h"

namespace segbot_sensors
{
  /** @brief Constructor. */
  RangesToCloud::RangesToCloud(ros::NodeHandle node,
                               ros::NodeHandle private_nh)
  {
    // advertise output point cloud (before subscribing to input data)
    points_ =
      node.advertise<sensor_msgs::PointCloud2>("range_points", 1);

    // subscribe to input ranges
    ranges_ =
      node.subscribe("sensor_ranges", 10,
                     &RangesToCloud::processRanges, (RangesToCloud *) this,
                     ros::TransportHints().tcpNoDelay(true));
  }

  void RangesToCloud::processRanges(const segbot_sensors::RangeArray::ConstPtr
                                    &ranges_msg)
  {
    unsigned int nsensors = ranges_msg->ranges.size(); // number of sensors
    ROS_DEBUG_STREAM("Received " << nsensors << " ranges");
    if (nsensors == 0)                  // empty message?
      return;

    // allocate a point cloud message
    pcl::PointCloud<pcl::PointXYZ>::Ptr
        cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // cloud's header is a pcl::PCLHeader, convert it before stamp assignment
    cloud->header.stamp =
      pcl_conversions::toPCL(ranges_msg->ranges[0].header).stamp;
    cloud->header.frame_id = "sensor_base"; // TODO: make this a parameter
    cloud->height = 1;

    // Provide points for sensor ranges provided.
    for (int sensor = 0; sensor < nsensors; ++sensor)
      {
        // Skip readings with no object within range (see: REP-0117).
        const sensor_msgs::Range *r = &ranges_msg->ranges[sensor];
        if (r->range < std::numeric_limits<float>::infinity())
          {
            // Transform this range point into the "sensor_base" frame.
            tf::StampedTransform transform;
            try
              {
                listener_.lookupTransform(cloud->header.frame_id,
                                          r->header.frame_id,
                                          ros::Time(0), transform);
              }
            catch (tf::TransformException ex)
              {
                ROS_WARN_STREAM(ex.what());
                continue;               // skip this sensor
              }

            tf::Point pt(r->range, 0.0, 0.0);
            pt = transform * pt;

            pcl::PointXYZ pcl_point;
            pcl_point.x = pt.m_floats[0];
            pcl_point.y = pt.m_floats[1];
            pcl_point.z = pt.m_floats[2];
            cloud->points.push_back(pcl_point);
            ++cloud->width;
          }
      }

    // publish the accumulated cloud message
    points_.publish(cloud);
  }


}; // end namespace segbot_sensors
