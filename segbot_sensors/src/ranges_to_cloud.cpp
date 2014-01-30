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

#include <sensor_msgs/PointCloud2.h>
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
                                    &rangesMsg)
  {
    ROS_INFO_STREAM("Received " << rangesMsg->ranges.size() << " ranges");
  }


}; // end namespace segbot_sensors
