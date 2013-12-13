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

#include <stdio.h>
#include <boost/foreach.hpp>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/calibration_job.hpp>
#include <industrial_extrinsic_cal/camera_observer.hpp>
#include <industrial_extrinsic_cal/ceres_costs_utils.hpp>

namespace industrial_extrinsic_cal {

  bool CalibrationJob::run()
  {
    runObservations();
    runOptimization();
  }

  bool CalibrationJob::runObservations()
  {
    //  BOOST_FOREACH(ObservationCommand);
  }

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
  string camera_file_name("junk1");
  string target_file_name("junk2");
  string caljob_file_name("junk3");
  CalibrationJob Calib_job(camera_file_name,target_file_name,caljob_file_name);
  printf("hello world\n");
}
