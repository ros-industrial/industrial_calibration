/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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

#include <ostream>
#include <stdio.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <boost/foreach.hpp>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/calibration_job.hpp>
#include <industrial_extrinsic_cal/camera_observer.hpp>
#include <industrial_extrinsic_cal/ceres_costs_utils.hpp>

// ROS includes
#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>



namespace industrial_extrinsic_cal {

  using std::string;

  bool CalibrationJob::run()
  {
    runObservations();
    runOptimization();
  }

  bool CalibrationJob::runObservations()
  {
    //  BOOST_FOREACH(ObservationCommand);
  }

  bool CalibrationJob::load()
  {
    std::ifstream camera_input_file(camera_def_file_name_.c_str());
    std::ifstream target_input_file(target_def_file_name_.c_str());
    std::ifstream caljob_input_file(caljob_def_file_name_.c_str());
    if(camera_input_file.fail()){
      printf("ERROR CalibrationJob::load(), couldn't open camera_input_file: %s\n",
	     camera_def_file_name_.c_str());
      return(false);
    }
    if( target_input_file.fail() ){
      printf("ERROR CalibrationJob::load(), couldn't open target_input_file: %s\n",
	     target_def_file_name_.c_str());
      return(false);
    }
    if(caljob_input_file.fail() ){
      printf("ERROR CalibrationJob::load(), couldn't open caljob_input_file: %s\n",
	     caljob_def_file_name_.c_str());
      return(false);
    }

    string temp_name;
    CameraParameters temp_parameters;
    unsigned int scene_id;
    try
      {
	YAML::Parser camera_parser(camera_input_file);
	YAML::Node camera_doc;
	camera_parser.GetNextDocument(camera_doc);

	// read in all static cameras
	if(const YAML::Node *camera_parameters = camera_doc.FindValue("static_cameras")){

	  printf("Found %d static cameras\n",camera_parameters->size());
	  for(unsigned int i=0; i<camera_parameters->size();i++){
	    (*camera_parameters)[i]["camera_name"]    >> temp_name;
	    (*camera_parameters)[i]["angle_axis_ax"]  >> temp_parameters.angle_axis[0];
	    (*camera_parameters)[i]["angle_axis_ay"]  >> temp_parameters.angle_axis[1];
	    (*camera_parameters)[i]["angle_axis_az"]  >> temp_parameters.angle_axis[2];
	    (*camera_parameters)[i]["position_x"]     >> temp_parameters.position[0];
	    (*camera_parameters)[i]["position_y"]     >> temp_parameters.position[1];
	    (*camera_parameters)[i]["position_z"]     >> temp_parameters.position[2];
	    (*camera_parameters)[i]["focal_length_x"] >> temp_parameters.focal_length_x;
	    (*camera_parameters)[i]["focal_length_y"] >> temp_parameters.focal_length_y;
	    (*camera_parameters)[i]["center_x"]       >> temp_parameters.center_x;
	    (*camera_parameters)[i]["center_y"]       >> temp_parameters.center_y;
	    (*camera_parameters)[i]["distortion_k1"]  >> temp_parameters.distortion_k1;
	    (*camera_parameters)[i]["distortion_k2"]  >> temp_parameters.distortion_k2;
	    (*camera_parameters)[i]["distortion_k3"]  >> temp_parameters.distortion_k3;
	    (*camera_parameters)[i]["distortion_p1"]  >> temp_parameters.distortion_p1;
	    (*camera_parameters)[i]["distortion_p2"]  >> temp_parameters.distortion_p2;
	    Camera temp_camera(temp_name,temp_parameters,false); // create a static camera
	    blocks_for_ceres_.addStaticCamera(temp_camera);
	  }
	}

	// read in all moving cameras
	if(const YAML::Node *camera_parameters = camera_doc.FindValue("moving_cameras")){
	  printf("Found %d moving cameras\n",camera_parameters->size());
	  for(unsigned int i=0; i<camera_parameters->size();i++){
	    (*camera_parameters)[i]["camera_name"]    >> temp_name;
	    (*camera_parameters)[i]["angle_axis_ax"]  >> temp_parameters.angle_axis[0];
	    (*camera_parameters)[i]["angle_axis_ay"]  >> temp_parameters.angle_axis[1];
	    (*camera_parameters)[i]["angle_axis_az"]  >> temp_parameters.angle_axis[2];
	    (*camera_parameters)[i]["position_x"]     >> temp_parameters.position[0];
	    (*camera_parameters)[i]["position_y"]     >> temp_parameters.position[1];
	    (*camera_parameters)[i]["position_z"]     >> temp_parameters.position[2];
	    (*camera_parameters)[i]["focal_length_x"] >> temp_parameters.focal_length_x;
	    (*camera_parameters)[i]["focal_length_y"] >> temp_parameters.focal_length_y;
	    (*camera_parameters)[i]["center_x"]       >> temp_parameters.center_x;
	    (*camera_parameters)[i]["center_y"]       >> temp_parameters.center_y;
	    (*camera_parameters)[i]["distortion_k1"]  >> temp_parameters.distortion_k1;
	    (*camera_parameters)[i]["distortion_k2"]  >> temp_parameters.distortion_k2;
	    (*camera_parameters)[i]["distortion_k3"]  >> temp_parameters.distortion_k3;
	    (*camera_parameters)[i]["distortion_p1"]  >> temp_parameters.distortion_p1;
	    (*camera_parameters)[i]["distortion_p2"]  >> temp_parameters.distortion_p2;
	    (*camera_parameters)[i]["scene_id"]       >> scene_id;
	    Camera temp_camera(temp_name,temp_parameters,true);
	    blocks_for_ceres_.addMovingCamera(temp_camera,scene_id);
	  }
	}   
      } // end try
    catch(...)
      {
	printf("load() Failed to read in moving cameras from  yaml file\n");
	printf("camera name = %s \n",temp_name.c_str());
	printf("angle_axis_ax = %lf \n",temp_parameters.angle_axis[0]);
	printf("angle_axis_ay = %lf \n",temp_parameters.angle_axis[1]);
	printf("angle_axis_az = %lf \n",temp_parameters.angle_axis[2]);
	printf("position_x = %lf \n",temp_parameters.position[0]);
	printf("position_y = %lf \n",temp_parameters.position[1]);
	printf("position_z = %lf \n",temp_parameters.position[2]);
	printf("focal_length_x = %lf \n",temp_parameters.focal_length_x);
	printf("focal_length_y = %lf \n",temp_parameters.focal_length_y);
	printf("center_x = %lf \n",temp_parameters.center_x);
	printf("center_y = %lf \n",temp_parameters.center_y);
	printf("distortion_k1 = %lf \n",temp_parameters.distortion_k1);
	printf("distortion_k2 = %lf \n",temp_parameters.distortion_k2);
	printf("distortion_k3 = %lf \n",temp_parameters.distortion_k3);
	printf("distortion_p1 = %lf \n",temp_parameters.distortion_p1);
	printf("distortion_p2 = %lf \n",temp_parameters.distortion_p2);
	printf("scene_id = %u \n",scene_id);
	//	ROS_ERROR("load() Failed to read in cameras yaml file");
	return(false);
      }
    printf("Successfully read in cameras\n");

    Target temp_target;
    try
      {
	YAML::Parser target_parser(target_input_file);
	YAML::Node target_doc;
	target_parser.GetNextDocument(target_doc);
	
	// read in all static targets
	if(const YAML::Node *target_parameters = target_doc.FindValue("static_targets")){
	  printf("Found %d static targets\n",target_parameters->size());
	  temp_target.is_moving = false;
	  for(unsigned int i=0; i<target_parameters->size();i++){
	    (*target_parameters)[i]["target_name"]    >> temp_target.target_name;
	    (*target_parameters)[i]["angle_axis_ax"]  >> temp_target.pose.ax;
	    (*target_parameters)[i]["angle_axis_ax"]  >> temp_target.pose.ay;
	    (*target_parameters)[i]["angle_axis_ay"]  >> temp_target.pose.az;
	    (*target_parameters)[i]["position_x"]     >> temp_target.pose.x;
	    (*target_parameters)[i]["position_y"]     >> temp_target.pose.y;
	    (*target_parameters)[i]["position_z"]     >> temp_target.pose.z;
	    (*target_parameters)[i]["num_points"]     >> temp_target.num_points;
	    const YAML::Node *points_node = (*target_parameters)[i].FindValue("points");
	    for(int j=0;j<points_node->size();j++){
	      const YAML::Node *pnt_node = (*points_node)[j].FindValue("pnt");
	      std::vector<float> temp_pnt;
	      (*pnt_node) >> temp_pnt;
	      Point3d temp_pnt3d;
	      temp_pnt3d.x = temp_pnt[0];
	      temp_pnt3d.y = temp_pnt[1];
	      temp_pnt3d.z = temp_pnt[2];
	      temp_target.pts.push_back(temp_pnt3d);
	    }	    
	    blocks_for_ceres_.addStaticTarget(temp_target);
	  }
	}

	// read in all moving targets
	if(const YAML::Node *target_parameters = target_doc.FindValue("moving_targets")){
	  printf("Found %d moving targets\n",target_parameters->size());
	  Target temp_target;
	  unsigned int scene_id;
	  temp_target.is_moving = true;
	  for(unsigned int i=0; i<target_parameters->size();i++){
	    (*target_parameters)[i]["target_name"]    >> temp_target.target_name;
	    (*target_parameters)[i]["angle_axis_ax"]  >> temp_target.pose.ax;
	    (*target_parameters)[i]["angle_axis_ax"]  >> temp_target.pose.ay;
	    (*target_parameters)[i]["angle_axis_ay"]  >> temp_target.pose.az;
	    (*target_parameters)[i]["position_x"]     >> temp_target.pose.x;
	    (*target_parameters)[i]["position_y"]     >> temp_target.pose.y;
	    (*target_parameters)[i]["position_z"]     >> temp_target.pose.z;
	    (*target_parameters)[i]["scene_id"]       >> scene_id;
	    (*target_parameters)[i]["num_points"]     >> temp_target.num_points;
	    const YAML::Node *points_node = (*target_parameters)[i].FindValue("points");
	    for(int j=0;j<points_node->size();j++){
	      const YAML::Node *pnt_node = (*points_node)[j].FindValue("pnt");
	      std::vector<float> temp_pnt;
	      (*pnt_node) >> temp_pnt;
	      Point3d temp_pnt3d;
	      temp_pnt3d.x = temp_pnt[0];
	      temp_pnt3d.y = temp_pnt[1];
	      temp_pnt3d.z = temp_pnt[2];
	      temp_target.pts.push_back(temp_pnt3d);
	    }	    
	    blocks_for_ceres_.addMovingTarget(temp_target,scene_id);
	  }
	}
	printf("Successfully read targets\n");
      }// end try
    catch(...)
      {
	printf("load() Failed to read in targets yaml file\n");
	//	ROS_ERROR("load() Failed to read in cameras yaml file");
	return(false);
      }
    //    ROS_INFO("successfuly read in cameras");
    return(true);
  }// end load()

  bool CeresBlocks::addStaticCamera(Camera camera_to_add)
  {
    BOOST_FOREACH(Camera cam, static_cameras_){
      if(cam.camera_name_ == camera_to_add.camera_name_) return(false); // camera already exists
    }
    static_cameras_.push_back(camera_to_add);
    return(true);
  }
  bool CeresBlocks::addStaticTarget(Target target_to_add)
  {
    BOOST_FOREACH(Target targ, static_targets_){
      if(targ.target_name == target_to_add.target_name) return(false); // target already exists
    }
    static_targets_.push_back(target_to_add);
    return(true);
  }
  bool CeresBlocks::addMovingCamera(Camera camera_to_add, int scene_id)
  {
    BOOST_FOREACH(MovingCamera cam, moving_cameras_){
      if(cam.cam.camera_name_ == camera_to_add.camera_name_ &&
	 cam.scene_id == scene_id) return(false); // camera already exists
    }
    MovingCamera Temp;
    Temp.cam = camera_to_add;
    Temp.scene_id = scene_id;
    moving_cameras_.push_back(Temp);
    return(true);
  }
  bool CeresBlocks::addMovingTarget(Target target_to_add, int scene_id)
  {
    BOOST_FOREACH(MovingTarget targ, moving_targets_){
      if(targ.targ.target_name == target_to_add.target_name &&
	 targ.scene_id == scene_id) return(false); // target already exists
    }
    MovingTarget Temp;
    Temp.targ = target_to_add;
    Temp.scene_id = scene_id;
    moving_targets_.push_back(Temp);
    return(true);
  }

  bool CalibrationJob::runOptimization()
  {
    // take all the data collected and create a Ceres optimization problem and run it

    // the following is an example taken from some working code. Clearly it won't work with
    // our data structures, but I included it here as an example

    // Create residuals for each observation in the bundle adjustment problem. The
    // parameters for cameras and points are added automatically.
    ceres::Problem problem;

    // need a block of cameras 
    // need a block of targets
    // these two lists need to be able to search for an existing item by name
    // and also an existing item by both name and scene id
  
    //  BOOST_FOREACH(ObservationScene Scene, os_){
    //    BOOST_FOREACH(CameraObservation CO, Scene.co){
    // determine if this is a new camera
    // Important Cases where we need to add a new camera: 
    // 1. first time a fixed camera is observed 
    // 2. First time in the scene a moving camera is observed 
    // ADD camera if necessary
    //      BOOST_FOREACH(Observation O, CO.o)
    // determine if this is a new target
    // Important Cases where we need to add a new target: 
    // 1. first time a fixed target is observed 
    // 2. First time in the scene a moving target is observed 
 
    // ADD all target points if necessary
   
	

 
       
       
    /*
    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.
    ceres::CostFunction* cost_function =  Camera_reprj_error::Create(Ob[i].x,Ob[i].y);

    problem.AddResidualBlock(cost_function, NULL , 
    C.PB_extrinsics, 
    C.PB_intrinsics, 
    Pts[i].PB);
    problem.SetParameterBlockConstant(C.PB_intrinsics);
    problem.SetParameterBlockConstant(Pts[i].PB);
    }

    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems.
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 1000;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    */
  }
} // end of namespace

using industrial_extrinsic_cal::CalibrationJob;
using std::string;
int main()
{
  string camera_file_name("camera_def.yaml");
  string target_file_name("target_def.yaml");
  string caljob_file_name("caljob_def.yaml");
  CalibrationJob Cal_job(camera_file_name,target_file_name,caljob_file_name);
  printf("hello world\n");
  Cal_job.load();
}
