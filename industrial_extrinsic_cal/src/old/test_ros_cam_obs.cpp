/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <ros/ros.h>
#include <industrial_extrinsic_cal/ros_camera_observer.h>
#include <yaml-cpp/yaml.h>
#include<fstream>

using industrial_extrinsic_cal::ROSCameraObserver;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "my_node_name");
	ros::NodeHandle nh;
	std::string topic="/camera/rgb/image_rect_color";

	 ROSCameraObserver cam_observer(topic);

	 boost::shared_ptr<industrial_extrinsic_cal::Target> target (new industrial_extrinsic_cal::Target);
	 industrial_extrinsic_cal::Roi roi;
	 target->target_type_ =  pattern_options::CircleGrid;
	 target->is_moving_=false;
	 target->target_name_="checkerboard";
	 target->checker_board_parameters_.pattern_rows=11;
	 target->checker_board_parameters_.pattern_cols=11;
	 //target->target_name="circlegrid";
	 //target->circle_grid_parameters.pattern_rows=31;
	 //target->circle_grid_parameters.pattern_cols=30;
	 //target->circle_grid_parameters.is_symmetric=true;
	 //target->pts[0]=0;
	 //target->pose=NULL;
	 roi.x_min=100;
	 roi.y_min=70;
	 roi.x_max=460;
	 roi.y_max=430;

	 std::string cost_type_str("dummy_cost_function");
	 if (cam_observer.addTarget(target , roi, cost_type_str))
	 {
		 ROS_INFO_STREAM("Added target successfully");
	 }
	 cam_observer.triggerCamera();
	 industrial_extrinsic_cal::CameraObservations camera_obs;
	 //cam_observer.getObservations(camera_obs);
	 if (cam_observer.getObservations(camera_obs))
	 {
		 ROS_INFO_STREAM("Success!");
	 }
/*
	 YAML::Emitter out;
	 out << YAML::BeginMap;
         //out << YAML::Key << "Points";
         //out << YAML::Value << "f";
         //out ;
         out << YAML::Key << "Points";
         out << YAML::Value;
         out << YAML::BeginMap;
	 for (int i=0; i<camera_obs.observations.size(); i++)
	 {
         out << YAML::Key << "Point_id" << YAML::Value << i;
	 out << YAML::Key << "Pnts";
	 out << YAML::Value << YAML::BeginSeq;
	 out << camera_obs.observations.at(i).image_loc_x << camera_obs.observations.at(i).image_loc_y;
         out << YAML::EndSeq;
	 }
	 out << YAML::EndMap;
	 out << YAML::EndMap;

	 std::ofstream fout("observations.txt");
	 fout << out.c_str();*/


	 return 0;
}
