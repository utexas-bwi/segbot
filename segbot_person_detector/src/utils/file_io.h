#include <signal.h> 
#include <vector>
#include <string.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

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

Eigen::VectorXf load_vector_from_file(const char *filename, int n){
	Eigen::VectorXf c;
	c.resize(n);
		
	FILE *fp = fopen(filename,"r");
		
	if (fp == NULL){
		ROS_WARN("File %s does not exist!",filename);
		return c;
	}
	
	for (int j = 0; j < n; j++){
		float f;
		int r = fscanf(fp,"%f\t",&f);
		if (r > 0)
			c(j)=f;
	}
		
	return c;
}

void write_vector_to_file(const char *filename, Eigen::VectorXf V, int n){
	FILE *fp = fopen(filename,"w");	
	for (int j = 0; j < n; j++)
		fprintf(fp,"%f\t",V(j));
	fprintf(fp,"\n");
	fclose(fp);
}
	
	

