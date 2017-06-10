
#include <cstdio>
#include <vector>
#include <string.h>
#include <iostream>
#include <signal.h>
#include <float.h>
#include <deque>

//for ROS_INFO
#include <ros/ros.h>

//for transformations
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace pcl_perception {

	class Group3F {
		public:
			std::vector<int> indeces;
			std::vector<Eigen::Vector3f> data;
			Eigen::Vector3f mean;
			void add(int index, Eigen::Vector3f x ){
				indeces.push_back(index);
				data.push_back(x);
				
				for (int i = 0; i < 3; i ++)
					mean(i)=0.0;
					
				for (unsigned int i = 0; i < data.size(); i++)
					mean = mean + data.at(i);
				mean = mean / (float)(data.size());
			}
	};

	class Grouper3F {
		
		public:
			std::vector<Group3F> groups;
			
			void print(){
				ROS_INFO("Num gorups: %i",(int)groups.size());
				for (unsigned int i = 0; i < groups.size(); i ++){
					ROS_INFO("\tGroup %i with %i points and mean (%f, %f, %f)",
						i,(int)groups.at(i).indeces.size(), groups.at(i).mean(0),groups.at(i).mean(1),groups.at(i).mean(2));
				
				}
			}
			
			int cluster_item(Eigen::Vector3f x){
				int best_group = -1;
				float best_score = FLT_MAX ;
				
				for (unsigned int i = 0; i < groups.size(); i ++){
					for (unsigned int j = 0; j < groups.at(i).data.size(); j++){
						float score_ij = (x - groups.at(i).data.at(j)).norm();
						if (score_ij < best_score){
							best_score = score_ij;
							best_group = i;
						}
					}
				}
				
				return best_group;
			}
			
			std::vector<int> get_group_indeces(int group_index){
				return groups.at(group_index).indeces;
			}
			
			int get_largest_group_index(){
				int index = -1;
				unsigned int max = 0;
				
				for (unsigned int i = 0; i < groups.size(); i++){
					if (groups.at(i).data.size() > max){
						max = groups.at(i).data.size();
						index = i;
					}
				}
				
				return index;
			}
			
			void start_new_group(int index, Eigen::Vector3f x){
				Group3F G;
				G.add(index,x);
				groups.push_back(G);
			}
		
			void add_next(int index, Eigen::Vector3f x, float theta){
				if (groups.size() == 0){
					start_new_group(index,x);
					return;
				}
				
				std::vector<float> scores;
				float min_score = 10000;
				int min_score_index = -1;
				
				for (unsigned int i = 0; i < groups.size(); i ++){
					Eigen::Vector3f diff_i = x - groups.at(i).mean;	
					float score = diff_i.norm();
					
					//ROS_INFO("L2 betweeen %f, %f, %f, and %f, %f, %f is %f",
					//	x(0),x(1),x(2),groups.at(i).mean(0),
					//	groups.at(i).mean(1),groups.at(i).mean(2),score);
					
					scores.push_back(score);
					
					if (score < min_score){
						min_score = score;
						min_score_index = i;
					}
				}
				
				if (min_score < theta){
					groups.at(min_score_index).add(index,x);
				}
				else {
					start_new_group(index,x);
				}
			}
	};

}
