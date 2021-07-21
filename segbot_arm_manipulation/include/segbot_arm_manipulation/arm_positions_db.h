#ifndef ARM_POSITION_DB_H
#define ARM_POSITION_DB_H
#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <stdio.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#define NUM_JOINTS_ARMONLY 6

#define PI 3.14159265


class ArmPositionDB {
	
	private:
		std::vector<std::string> j_position_names;
		std::vector< std::vector<float> > j_positions;
	
		std::vector<std::string> tool_position_names;
		std::vector<geometry_msgs::Pose> t_positions;
	
	public:
	
		bool hasJointPosition(std::string name){
			for (unsigned int i = 0; i < j_position_names.size(); i++){
				if (j_position_names[i] == name){
					return true;
				}
			}
			return false;
		}
		
		bool hasCarteseanPosition(std::string name){
			for (unsigned int i = 0; i < tool_position_names.size(); i++){
				if (tool_position_names[i] == name){
					return true;
				}
			}
			return false;
		}
		
		geometry_msgs::PoseStamped getToolPositionStamped(
						std::string name,
						std::string frame_id){
			for (unsigned int i = 0; i < tool_position_names.size(); i++){
				if (tool_position_names[i] == name){
					geometry_msgs::PoseStamped target_pose;
					target_pose.header.stamp = ros::Time::now();
					target_pose.header.frame_id = frame_id;
					target_pose.pose=t_positions[i];
					return target_pose;
				}
			}
			
			ROS_ERROR("[arm_positions_db.h] Requesting pose that is not in DB!");
			geometry_msgs::PoseStamped p_empty;
			return p_empty;
		}
	
		geometry_msgs::Pose getToolPosition(std::string name){
			for (unsigned int i = 0; i < tool_position_names.size(); i++){
				if (tool_position_names[i] == name){
					return t_positions[i];
				}
			}
			
			geometry_msgs::Pose p_empty;
			return p_empty;
		}
		
		std::vector<float> getJointPosition(std::string name){
			for (unsigned int i = 0; i < j_position_names.size(); i++){
				if (j_position_names[i] == name){
					return j_positions[i];
				}
			}
		}
		
		void print(){
			std::cout << "# of joint-space positions: " << j_position_names.size() << "\n";
			
			for (unsigned int i = 0; i < j_position_names.size(); i++){
				std::cout << "\tname:" << j_position_names.at(i) << "\t";
				
				for (unsigned int j = 0; j < NUM_JOINTS_ARMONLY; j++){
					std::cout << j_positions[i][j];
					if (j < NUM_JOINTS_ARMONLY - 1)
						std::cout << ",";
					else std::cout << "\n";
					
				}
			}
			
			std::cout << "\n# of tool-space positions: " << tool_position_names.size() << "\n";
			
			for (unsigned int i = 0; i < tool_position_names.size(); i++){
				std::cout << "\tname:" << tool_position_names.at(i) << "\t";
				ROS_INFO_STREAM(t_positions[i]);
			}
		}
		
		ArmPositionDB(std::string joint_positions_filename, 
						std::string tool_positions_filename){
							
			
			
			
			FILE *fp_j=fopen(joint_positions_filename.c_str(), "r");
			
			//the first integer is the # of entries in the file
			int num_entries_j = 0;
			fscanf(fp_j,"%i\n",&num_entries_j);
			
			for (int i = 0; i < num_entries_j; i++){
				char name_i[80];
				std::vector<float> p_i;
				p_i.resize(NUM_JOINTS_ARMONLY);
				
				fscanf(fp_j,"%s\t%f,%f,%f,%f,%f,%f\n",name_i,&p_i[0],&p_i[1],&p_i[2],&p_i[3],&p_i[4],&p_i[5]);
			
				j_positions.push_back(p_i);
				
				std::string name_i_s(name_i);
				j_position_names.push_back(name_i_s);
			}
			
			//load cartesean positions
			FILE *fp_c=fopen(tool_positions_filename.c_str(), "r");
			
			//the first integer is the # of entries in the file
			int num_entries_c = 0;
			fscanf(fp_c,"%i\n",&num_entries_c);
					
			for (int i = 0; i < num_entries_c; i++){
				char name_i[80];
				std::vector<float> p_i;
				p_i.resize(7);
				
				fscanf(fp_c,"%s\t%f,%f,%f,%f,%f,%f,%f\n",name_i,&p_i[0],&p_i[1],&p_i[2],&p_i[3],&p_i[4],&p_i[5],&p_i[6]);
				
				geometry_msgs::Pose pose_i;
				pose_i.position.x = p_i[0];
				pose_i.position.y = p_i[1];
				pose_i.position.z = p_i[2];
				pose_i.orientation.x = p_i[3];
				pose_i.orientation.y = p_i[4];
				pose_i.orientation.z = p_i[5];
				pose_i.orientation.w = p_i[6];
			
				
				t_positions.push_back(pose_i);
				
				std::string name_i_s(name_i);
				tool_position_names.push_back(name_i_s);
			}
					
		}
		
		
	
		
};

#endif
