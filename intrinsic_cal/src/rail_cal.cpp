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
#include <industrial_extrinsic_cal/camera_observer_trigger.h>
#include <industrial_extrinsic_cal/user_accept.h>
#include <industrial_extrinsic_cal/ros_camera_observer.h>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/camera_definition.h>
#include <industrial_extrinsic_cal/ceres_costs_utils.h> 
#include <industrial_extrinsic_cal/ceres_costs_utils.hpp> 
#include <intrinsic_cal/rail_ical_run.h>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/types.h"

using std::string;
using boost::shared_ptr;
using boost::make_shared;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using industrial_extrinsic_cal::Target;
using industrial_extrinsic_cal::CameraObservations;
using industrial_extrinsic_cal::ROSCameraObserver;
using industrial_extrinsic_cal::Roi;
using industrial_extrinsic_cal::Pose6d;
using industrial_extrinsic_cal::Point3d;
using industrial_extrinsic_cal::Camera;
using industrial_extrinsic_cal::CameraParameters;
using industrial_extrinsic_cal::NoWaitTrigger;

class RailCalService 
{
public:
  RailCalService(ros::NodeHandle nh);
  ~RailCalService()  {  } ;
  bool executeCallBack( intrinsic_cal::rail_ical_run::Request &req, intrinsic_cal::rail_ical_run::Response &res);
  void  initMCircleTarget(int rows, int cols, double circle_dia, double spacing);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer rail_cal_server_;
  shared_ptr<Target> target_;
  shared_ptr<Camera> camera_;
  double focal_length_x_;
  double focal_length_y_;
  double center_x_;
  double center_y_;
  string image_topic_;
  string camera_info_topic_;
  int target_type_;
  int target_rows_;
  int target_cols_;
  double circle_spacing_;
  double circle_diameter_;
  int num_camera_locations_;
  double camera_spacing_;
  int image_height_;
  int image_width_;
  double D0_;
  CameraParameters camera_parameters_;
};

RailCalService::RailCalService(ros::NodeHandle nh)
{
  
  nh_ = nh;
  ros::NodeHandle pnh("~");

  if(!pnh.getParam( "image_topic", image_topic_)){
    ROS_ERROR("Must set param:  image_topic");
  }

  if(!pnh.getParam( "camera_info_topic", camera_info_topic_)){
    ROS_ERROR("Must set param:  camera_info_topic");
  }

  target_type_ == pattern_options::ModifiedCircleGrid;

  if(!pnh.getParam( "target_rows", target_rows_)){
    ROS_ERROR("Must set param:  target_rows");
  }
  if(!pnh.getParam( "target_cols", target_cols_)){
    ROS_ERROR("Must set param:  target_cols");
  }
  if(!pnh.getParam( "target_circle_dia", circle_diameter_)){
    ROS_ERROR("Must set param:  target_circle_dia");
  }
  if(!pnh.getParam( "target_spacing", circle_spacing_)){
    ROS_ERROR("Must set param:  target_spacing");
  }
  if(!pnh.getParam( "num_camera_locations", num_camera_locations_)){
    ROS_ERROR("Must set param:  num_camera_locations");
  }
  if(!pnh.getParam( "camera_spacing", camera_spacing_)){
    ROS_ERROR("Must set param:  camera_spacing");
  }
  if(!pnh.getParam( "image_height", image_height_)){
    ROS_ERROR("Must set param:  image_height_");
  }
  if(!pnh.getParam( "image_width", image_width_)){
    ROS_ERROR("Must set param:  image_width_");
  }
  if(!pnh.getParam( "target_to_rail_distance", D0_)){
    ROS_ERROR("Must set param:  target_to_rail_distance");
  }

  // setting camera extrinsic parameters in no longer necessary they are not used
  tf::Matrix3x3 R; 
  Pose6d pose;
  R[0][0] = 0;   R[0][1] = 0;   R[0][2] = -1;
  R[1][0] = 1;   R[1][1] = 0;   R[1][2] = 0;
  R[2][0] = 0;   R[2][1] = -1;   R[2][2] = 0;

  pose.setBasis(R);
  pose.setOrigin(0.0, 0.0, 0.0);
  camera_parameters_.angle_axis[0] = pose.ax;
  camera_parameters_.angle_axis[1] = pose.ay;
  camera_parameters_.angle_axis[1] = pose.az;
  camera_parameters_.position[0] = pose.x;
  camera_parameters_.position[1] = pose.y;
  camera_parameters_.position[2] = pose.z;

  camera_parameters_.focal_length_x = 2785.0;
  camera_parameters_.focal_length_y = 2781.0;
  camera_parameters_.center_x = 963.505;
  camera_parameters_.center_y = 575.686;
  
  bool is_moving = true;
  camera_ =  make_shared<industrial_extrinsic_cal::Camera>("my_camera", camera_parameters_, is_moving);
  camera_->trigger_ = make_shared<NoWaitTrigger>();
  camera_->camera_observer_ = make_shared<ROSCameraObserver>(image_topic_, camera_info_topic_);
  camera_->camera_observer_->pullCameraInfo(camera_->camera_parameters_.focal_length_x,
                                           camera_->camera_parameters_.focal_length_y,
                                           camera_->camera_parameters_.center_x,
                                           camera_->camera_parameters_.center_y,
                                           camera_->camera_parameters_.distortion_k1,
                                           camera_->camera_parameters_.distortion_k2,
                                           camera_->camera_parameters_.distortion_k3,
                                           camera_->camera_parameters_.distortion_p1,
                                           camera_->camera_parameters_.distortion_p2);
  initMCircleTarget(target_rows_, target_cols_, circle_diameter_, circle_spacing_);
  rail_cal_server_ = nh_.advertiseService( "RailCalService", &RailCalService::executeCallBack, this);
}

bool RailCalService::executeCallBack( intrinsic_cal::rail_ical_run::Request &req, intrinsic_cal::rail_ical_run::Response &res)
{
  ros::NodeHandle nh;
  CameraObservations camera_observations;
  int num_observations ;

  camera_->camera_observer_->clearObservations();
  camera_->camera_observer_->clearTargets();

  // set the roi to the whole image
  Roi roi;
  roi.x_min = 0;
  roi.y_min = 0;
  roi.x_max = image_width_;
  roi.y_max = image_height_;

  industrial_extrinsic_cal::Cost_function cost_type = industrial_extrinsic_cal::cost_functions::CameraReprjErrorWithDistortion;
  Problem problem; // note, a new problem gets created each time execute is called, so old observation data is not re-used.

  // set initial conditions, 
  // x of target aligned with y of rail    [ 0  0  1 ]
  // y of target aligned with z of rail    [ 1  0  0 ]
  // z of target aligned with x of rail,   [ 0  1  0 ]
  // target about D0 meters behind rail
  tf::Matrix3x3 R; 
  R[0][0] = 1;   R[0][1] = 0;   R[0][2] = 0;
  R[1][0] = 0;   R[1][1] = -1;   R[1][2] = 0;
  R[2][0] = 0;   R[2][1] = 0;   R[2][2] = -1;
  
  target_->pose_.setBasis(R);
  target_->pose_.setOrigin(-0.011, 0.05, D0_);
  target_->pose_.show("initial target pose");  
  ros::NodeHandle pnh("~");
  bool camera_ready = false;
  pnh.setParam("camera_ready", camera_ready);
  for(int i=0; i<num_camera_locations_; i++){
    double rail_position = i*camera_spacing_;
    ROS_ERROR("Move Camera to location %d which should be %lf meters from start. Then set camera_ready", i, i*camera_spacing_);
    
    // wait for camera to be moved
    camera_ready = false;
    pnh.setParam("camera_ready", camera_ready);
    while(camera_ready == false){
      if(!pnh.getParam("camera_ready", camera_ready)){
	ROS_ERROR("parameter camera_ready does not exists");
      }
    }
    
    // gather next image
    camera_->camera_observer_->clearTargets();
    camera_->camera_observer_->clearObservations();
    camera_->camera_observer_->addTarget(target_, roi, cost_type);
    camera_->camera_observer_->triggerCamera();
    while (!camera_->camera_observer_->observationsDone()) ;
    camera_->camera_observer_->getObservations(camera_observations);
    num_observations = (int) camera_observations.size();
    if(num_observations != target_rows_* target_cols_){
      ROS_ERROR("Target Locator could not find target %d", num_observations);
    }
    
    // add a new cost to the problem for each observation
    CostFunction* cost_function[num_observations]; // not sure I need a new cost function each time, but this just uses memory
    for(int i=0; i<num_observations; i++){
      double image_x = camera_observations[i].image_loc_x;
      double image_y = camera_observations[i].image_loc_y;
      Point3d point = target_->pts_[i]; // assume correct ordering from camera observer
      cost_function[i] = industrial_extrinsic_cal::RailICal::Create(image_x, image_y, rail_position, point);
      problem.AddResidualBlock(cost_function[i], NULL , 
			       camera_->camera_parameters_.pb_intrinsics,
			       target_->pose_.pb_pose);
      double residual[2];
      industrial_extrinsic_cal::RailICal RC(image_x, image_y, rail_position, point);
    } // for each observation at this camera_location
  }// for each camera_location

  // set up and solve the problem
  Solver::Options options;
  Solver::Summary summary;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 2000;
  ceres::Solve(options, &problem, &summary);
  if(summary.termination_type != ceres::NO_CONVERGENCE){
    double initial_cost = summary.initial_cost/num_observations;
    double final_cost = summary.final_cost/num_observations;
    ROS_INFO("Problem solved, initial cost = %lf, final cost = %lf", initial_cost, final_cost);
    target_->pose_.show("target_pose");
    ROS_ERROR("camera_matrix data: [ %lf, 0.0, %lf, 0.0, %lf, %lf, 0.0, 0.0, 1.0]", 
	      camera_->camera_parameters_.focal_length_x,
	      camera_->camera_parameters_.center_x,
	      camera_->camera_parameters_.focal_length_y,
	      camera_->camera_parameters_.center_y);
    ROS_ERROR("distortion data: [ %lf,  %lf,  %lf,  %lf,  %lf]",
   	      camera_->camera_parameters_.distortion_k1,
   	      camera_->camera_parameters_.distortion_k2,
   	      camera_->camera_parameters_.distortion_p1,
   	      camera_->camera_parameters_.distortion_p2,
   	      camera_->camera_parameters_.distortion_k3);
    ROS_ERROR("projection_matrix data: [ %lf, 0.0, %lf, 0.0, 0.0, %lf, %lf, 0.0, 0.0, 0.0, 1.0, 0.0]", 
	      camera_->camera_parameters_.focal_length_x,
	      camera_->camera_parameters_.center_x,
	      camera_->camera_parameters_.focal_length_y,
	      camera_->camera_parameters_.center_y);
    if(final_cost <= req.allowable_cost_per_observation){
      res.final_pose.position.x = target_->pose_.x;
      res.final_pose.position.y = target_->pose_.y;
      res.final_pose.position.z = target_->pose_.z;
      res.final_cost_per_observation  = final_cost;
      target_->pose_.getQuaternion(res.final_pose.orientation.x,
				   res.final_pose.orientation.y, 
				   res.final_pose.orientation.z,
				   res.final_pose.orientation.w);
  camera_->camera_observer_->pushCameraInfo(camera_->camera_parameters_.focal_length_x,
                                           camera_->camera_parameters_.focal_length_y,
                                           camera_->camera_parameters_.center_x,
                                           camera_->camera_parameters_.center_y,
                                           camera_->camera_parameters_.distortion_k1,
                                           camera_->camera_parameters_.distortion_k2,
                                           camera_->camera_parameters_.distortion_k3,
                                           camera_->camera_parameters_.distortion_p1,
                                           camera_->camera_parameters_.distortion_p2);

      return true;
    }
    else{
      res.final_cost_per_observation  = final_cost;
      ROS_ERROR("allowable cost exceeded %f > %f", final_cost, req.allowable_cost_per_observation);
      return(false);
    }
  }
}

void RailCalService::initMCircleTarget(int rows, int cols, double circle_dia, double spacing)
{
  target_ =  make_shared<industrial_extrinsic_cal::Target>();
  target_->is_moving_ = true;
  target_->target_name_ = "modified_circle_target";
  target_->target_frame_ = "target_frame";
  target_->target_type_ =  2;
  target_->circle_grid_parameters_.pattern_rows =rows;
  target_->circle_grid_parameters_.pattern_cols = cols;
  target_->circle_grid_parameters_.circle_diameter = circle_dia;
  target_->circle_grid_parameters_.is_symmetric = true; 
  // create a grid of points
  target_->pts_.clear();
  target_->num_points_ = rows*cols;
  for(int i=0; i<rows; i++){
    for(int j=0; j<cols; j++){
      Point3d point;
      point.x = j*spacing;
      point.y = (rows -1 -i)*spacing;
      point.z = 0.0;
      target_->pts_.push_back(point);
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rail_cal_service");
  ros::NodeHandle node_handle;
  RailCalService rail_cal(node_handle);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
