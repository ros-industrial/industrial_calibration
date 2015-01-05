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
#include <ros/package.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <industrial_extrinsic_cal/camera_observer_trigger.h>
#include <industrial_extrinsic_cal/user_accept.h>
#include <industrial_extrinsic_cal/ros_camera_observer.h>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/ceres_costs_utils.h> 
#include <industrial_extrinsic_cal/ceres_costs_utils.hpp> 
#include <industrial_extrinsic_cal/target_locator.h> 
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/types.h"

using std::string;
using boost::shared_ptr;
using boost::make_shared;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using industrial_extrinsic_cal::target_locator;
using industrial_extrinsic_cal::Target;
using industrial_extrinsic_cal::CameraObservations;
using industrial_extrinsic_cal::ROSCameraObserver;
using industrial_extrinsic_cal::Roi;
using industrial_extrinsic_cal::Pose6d;
using industrial_extrinsic_cal::Point3d;

class TargetLocatorService 
{
public:
  TargetLocatorService(ros::NodeHandle nh);
  ~TargetLocatorService()  {  } ;
  bool executeCallBack( target_locator::Request &req,
			target_locator::Response &res);
  void  init_target(int rows, int cols, double circle_dia, double spacing);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer target_locate_server_;
  shared_ptr<Target> target_;
  double focal_length_x_;
  double focal_length_y_;
  double center_x_;
  double center_y_;
  string image_topic_;
  int target_rows_;
  int target_cols_;
};

TargetLocatorService::TargetLocatorService(ros::NodeHandle nh)
{
  
  nh_ = nh;
  ros::NodeHandle pnh("~");

  int rows, cols;
  double diameter, spacing;
  if(!pnh.getParam( "image_topic", image_topic_)){
    ROS_ERROR("Must set param:  image_topic");
  }
  if(!pnh.getParam( "target_rows", target_rows_)){
    ROS_ERROR("Must set param:  target_rows");
  }
  if(!pnh.getParam( "target_cols", target_cols_)){
    ROS_ERROR("Must set param:  target_cols");
  }
  if(!pnh.getParam( "target_circle_dia", diameter)){
    ROS_ERROR("Must set param:  target_circle_dia");
  }
  if(!pnh.getParam( "target_spacing", spacing)){
    ROS_ERROR("Must set param:  target_spacing");
  }
  if(!pnh.getParam( "focal_length_x", focal_length_x_)){
    ROS_ERROR("Must set param:  focal_length_x");
  }
  if(!pnh.getParam( "focal_length_y", focal_length_y_)){
    ROS_ERROR("Must set param:  focal_length_y");
  }
  if(!pnh.getParam( "center_x", center_x_)){
    ROS_ERROR("Must set param:  center_x");
  }
  if(!pnh.getParam( "center_y", center_y_)){
    ROS_ERROR("Must set param:  center_y");
  }
  init_target(target_rows_, target_cols_, diameter, spacing);
  
  target_locate_server_ = nh_.advertiseService( "TargetLocateService", &TargetLocatorService::executeCallBack, this);
}

bool TargetLocatorService::executeCallBack( target_locator::Request &req, target_locator::Response &res)
{
  ros::NodeHandle nh;
  CameraObservations camera_observations;
  
  ROSCameraObserver camera_observer(image_topic_);
  camera_observer.clearObservations();
  camera_observer.clearTargets();

  Roi roi;
  roi.x_min = req.roi.x_offset;
  roi.y_min = req.roi.y_offset;
  roi.x_max = req.roi.x_offset + req.roi.width;
  roi.y_max = req.roi.y_offset + req.roi.height;

  industrial_extrinsic_cal::Cost_function cost_type;
  
  camera_observer.clearTargets();
  camera_observer.clearObservations();

  camera_observer.addTarget(target_, roi, cost_type);
  camera_observer.triggerCamera();
  while (!camera_observer.observationsDone()) ;

  camera_observer.getObservations(camera_observations);
  
  int num_observations = (int) camera_observations.size();
  if(num_observations != target_rows_ * target_cols_){
    ROS_ERROR("Target Locator could not find target");
    return(false);
  }

  // set initial conditions
  Pose6d target_pose;
  target_pose.setQuaternion(req.initial_pose.orientation.x, req.initial_pose.orientation.y, req.initial_pose.orientation.z, req.initial_pose.orientation.w );
  target_pose.setOrigin(req.initial_pose.position.x, req.initial_pose.position.y, req.initial_pose.position.z );

  Problem problem;
  for(int i=0; i<num_observations; i++){
    double image_x = camera_observations[i].image_loc_x;
    double image_y = camera_observations[i].image_loc_y;
    Point3d point = target_->pts_[i]; // assume the correct ordering
    CostFunction* cost_function =
      industrial_extrinsic_cal::CameraReprjErrorPK::Create(image_x, image_y, focal_length_x_, focal_length_y_,center_x_, center_y_, point);
    problem.AddResidualBlock(cost_function, NULL , target_pose.pb_pose);
  }
  Solver::Options options;
  Solver::Summary summary;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 1000;
  ceres::Solve(options, &problem, &summary);

  if(summary.termination_type == ceres::USER_SUCCESS
     || summary.termination_type == ceres::FUNCTION_TOLERANCE
     || summary.termination_type == ceres::GRADIENT_TOLERANCE
     || summary.termination_type == ceres::PARAMETER_TOLERANCE
     ){

    double error_per_observation = summary.final_cost/num_observations;
    if(error_per_observation <= req.allowable_cost_per_observation){
      res.final_pose.position.x = target_pose.x;
      res.final_pose.position.y = target_pose.y;
      res.final_pose.position.z = target_pose.z;
      res.final_cost_per_observation  = error_per_observation;
      target_pose.getQuaternion(res.final_pose.orientation.x, res.final_pose.orientation.y, res.final_pose.orientation.z, res.final_pose.orientation.w);

      return true;
    }
    else{
      res.final_cost_per_observation  = error_per_observation;
      ROS_ERROR("allowable cost exceeded %f > %f", error_per_observation, req.allowable_cost_per_observation);
      return(false);
    }
  }
}

void TargetLocatorService::init_target(int rows, int cols, double circle_dia, double spacing)
{
  target_ =  make_shared<industrial_extrinsic_cal::Target>();
  target_->target_name_ = "modified_circle_target";
  target_->target_frame_ = "target_frame";
  target_->target_type_ =  2;
  target_->circle_grid_parameters_.pattern_rows =rows;
  target_->circle_grid_parameters_.pattern_cols = cols;
  target_->circle_grid_parameters_.circle_diameter = circle_dia;
  target_->circle_grid_parameters_.is_symmetric = true; 
  // create a grid of points
  target_->pts_.clear();
  for(int i=0; i<rows; i++){
    for(int j=0; j<cols; j++){
      Point3d point;
      point.x = (rows -1 -i)*spacing;
      point.y = (cols -1 -j)*spacing;
      point.z = 0.0;
      target_->pts_.push_back(point);
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_locator_service");
  ros::NodeHandle node_handle;
  TargetLocatorService target_locator(node_handle);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
