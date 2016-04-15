/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 *
 * $Id: statistical_outlier_removal.cpp 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#include <geometry_msgs/PolygonStamped.h>
#include <pluginlib/class_list_macros.h>
#include <segbot_sensors/segbot_velodyne_outlier_removal.h>

//////////////////////////////////////////////////////////////////////////////////////////////
bool
segbot_sensors::SegbotVelodyneOutlierRemoval::child_init (ros::NodeHandle &nh, bool &has_service)
{
  // Enable the dynamic reconfigure service
  has_service = true;
  srv_ = boost::make_shared <dynamic_reconfigure::Server<segbot_sensors::SegbotVelodyneOutlierRemovalConfig> > (nh);
  dynamic_reconfigure::Server<segbot_sensors::SegbotVelodyneOutlierRemovalConfig>::CallbackType f = 
    boost::bind (&SegbotVelodyneOutlierRemoval::config_callback, this, _1, _2);
  srv_->setCallback (f);

  // Latched publication.
  footprint_publisher_ = nh.advertise<geometry_msgs::PolygonStamped>("~footprint", 1, true);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
segbot_sensors::SegbotVelodyneOutlierRemoval::config_callback (segbot_sensors::SegbotVelodyneOutlierRemovalConfig &config, uint32_t level)
{
  boost::mutex::scoped_lock lock (mutex_);

  footprint_.clear();

  bool at_near = true;
  float near_range = 0.3f;
  for (int angle_idx = 0; angle_idx < 8; ++angle_idx) {
    // Assuming the wire conduit increases the 6th and 7th angles. 
    float angle = (angle_idx == 5 || angle_idx == 6) ? config.wire_conduit_angle : config.support_struct_angle;
    std::vector<float> ranges;
    if (at_near) {
      angle += (angle_idx / 2) * M_PI/2;
      ranges.push_back(near_range); 
      ranges.push_back(config.range);
    } else {
      angle = ((angle_idx + 1) / 2) * M_PI/2 - angle;
      ranges.push_back(config.range);
      ranges.push_back(near_range); 
    }
    for (int range_idx = 0; range_idx < 2; ++range_idx) {
      geometry_msgs::Point p;
      p.x = cosf(angle) * ranges[range_idx]; 
      p.y = sinf(angle) * ranges[range_idx]; 
      footprint_.push_back(p);
    }
    at_near != at_near;
  }

  // Publish polygon corresponding to footprint - visualization
  geometry_msgs::PolygonStamped polygon_msg;
  polygon_msg.header.frame_id = frame_;
  polygon_msg.header.stamp = ros::Time::now();
  polygon_msg.polygon.points.resize(footprint_.size());
  for (uint32_t i = 0; i < footprint_.size(); ++i) {
    polygon_msg.polygon.points[i].x = footprint_[i].x;
    polygon_msg.polygon.points[i].y = footprint_[i].y;
    polygon_msg.polygon.points[i].z = 0;
  }
  footprint_publisher_.publish(polygon_msg);
}

typedef segbot_sensors::SegbotVelodyneOutlierRemoval SegbotVelodyneOutlierRemoval;
PLUGINLIB_EXPORT_CLASS(SegbotVelodyneOutlierRemoval,nodelet::Nodelet);
