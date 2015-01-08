/*
 * LICENSE: https://github.com/utexas-bwi/segbot/blob/devel/LICENSE
 */

#ifndef LASER_SCAN_ANGLE_FILTER_H
#define LASER_SCAN_ANGLE_FILTER_H

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>

namespace segbot_sensors
{
 
  /*
   *  A class that filters LaserScan messages using a minimum and maximum angles (radians)
   *  For 
   * 
   * 
   */
 
  class AngleRangeFilter : public filters::FilterBase<sensor_msgs::LaserScan>
  {
    public:

      bool configure() {
		
		getParam("angle_min", angle_param_min);  
		getParam("angle_max", angle_param_max);  
		
		start_index=-1;
		end_index=-1;
		computed_indeces = false;
		    
		  
        return true;
      }

      virtual ~AngleRangeFilter(){}

      bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan){

		int num_original_measurements = input_scan.ranges.size();
		
		//compute the start and end index in the original vector of values
		if (!computed_indeces){
			
			if (angle_param_max > input_scan.angle_max){ //check if requested range is outside the actual range
				angle_param_max = input_scan.angle_max;
				end_index = num_original_measurements - 1;
			}
			else 
				end_index = (int)floor((double)num_original_measurements*(angle_param_max-input_scan.angle_min)/(input_scan.angle_max-input_scan.angle_min));
			
				
			if (angle_param_min < input_scan.angle_min){ //check if requested range is outside the actual range
				angle_param_min = input_scan.angle_min;
				start_index = 0;
			}
			else 
				start_index = (int)floor((double)num_original_measurements*(angle_param_min - input_scan.angle_min)/(input_scan.angle_max-input_scan.angle_min));
			
			//get the new values for the minimum and maximum angle -- these are going to be approximation to the input arguments
			angle_out_min = input_scan.angle_min + (start_index*input_scan.angle_increment);
			angle_out_max = input_scan.angle_min + (end_index*input_scan.angle_increment);
			
			computed_indeces = true;
			
			/*ROS_INFO("[angle range filter]: total num measurements: %i",num_original_measurements);
			ROS_INFO("[angle range filter]: original angle range: %f to %f", input_scan.angle_min,  input_scan.angle_max);
			ROS_INFO("[angle range filter]: start and end indeces %i, %i",start_index,end_index);*/
		}
        
		filtered_scan.ranges.resize(end_index-start_index+1);
		filtered_scan.intensities.resize(end_index-start_index+1);
		
		for(unsigned int count = 0; count < filtered_scan.ranges.size(); ++count){
          filtered_scan.ranges[count] = input_scan.ranges[count+start_index];
          filtered_scan.intensities[count] = input_scan.intensities[count+start_index];
        }
    

        //make sure to set all the needed fields on the filtered scan
        filtered_scan.header.frame_id = input_scan.header.frame_id;
        filtered_scan.header.stamp = input_scan.header.stamp;
        filtered_scan.angle_min = angle_out_min; //override
        filtered_scan.angle_max = angle_out_max; //override
        filtered_scan.angle_increment = input_scan.angle_increment;
        filtered_scan.time_increment = input_scan.time_increment;
        filtered_scan.scan_time = input_scan.scan_time;
        filtered_scan.range_min = input_scan.range_min;
        filtered_scan.range_max = input_scan.range_max;


        return true;

      }
      
    private:
		double angle_param_min, angle_param_max;
		double angle_out_min, angle_out_max;
		int start_index,end_index;
		bool computed_indeces;
		
  };
}
#endif

