/*
 * LICENSE: https://github.com/utexas-bwi/segbot/blob/devel/LICENSE
 */

#include "segbot_sensors/footprint_filter.h"
#include "segbot_sensors/nan_to_inf_filter.h"
#include "sensor_msgs/LaserScan.h"
#include "filters/filter_base.h"

#include "pluginlib/class_list_macros.h"

PLUGINLIB_REGISTER_CLASS(segbot_sensors/SegbotFootprintFilter, segbot_sensors::SegbotFootprintFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_REGISTER_CLASS(segbot_sensors/NanToInfFilter, segbot_sensors::NanToInfFilter, filters::FilterBase<sensor_msgs::LaserScan>)
