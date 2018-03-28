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
#include <industrial_extrinsic_cal/ceres_costs_utils.h>
#include <industrial_extrinsic_cal/ceres_costs_utils.hpp>
#include <target_finder/target_locator.h>
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
using target_finder::target_locator;
class TargetLocatorService
{
public:
  TargetLocatorService(ros::NodeHandle nh);
  ~TargetLocatorService(){};
  bool executeCallBack(target_locator::Request& req, target_locator::Response& res);
  void initMCircleTarget(int rows, int cols, double circle_dia, double spacing);
  void initBallsTarget();

private:
  ros::NodeHandle nh_;
  ros::ServiceServer target_locate_server_;
  shared_ptr<Target> target_;
  string image_topic_;
  string camera_name_;
  int target_type_;
  int target_rows_;
  int target_cols_;
};

TargetLocatorService::TargetLocatorService(ros::NodeHandle nh)
{
  nh_ = nh;
  ros::NodeHandle pnh("~");

  int rows, cols;
  double diameter, spacing;
  if (!pnh.getParam("image_topic", image_topic_))
  {
    ROS_ERROR("Must set param:  image_topic");
  }

  if (!pnh.getParam("target_type", target_type_))
  {
    ROS_ERROR("Must set param: target_type");
  }

  if (!pnh.getParam("camera_name", camera_name_))
  {
    ROS_ERROR("Must set param: camera_name");
  }

  if (target_type_ == pattern_options::Balls)
  {
    ROS_ERROR("initializing balls target");
    initBallsTarget();
  }
  else if (target_type_ == pattern_options::ModifiedCircleGrid)
  {
    if (!pnh.getParam("target_rows", target_rows_))
    {
      ROS_ERROR("Must set param:  target_rows");
    }
    if (!pnh.getParam("target_cols", target_cols_))
    {
      ROS_ERROR("Must set param:  target_cols");
    }
    if (!pnh.getParam("target_circle_dia", diameter))
    {
      ROS_ERROR("Must set param:  target_circle_dia");
    }
    if (!pnh.getParam("target_spacing", spacing))
    {
      ROS_ERROR("Must set param:  target_spacing");
    }
    initMCircleTarget(target_rows_, target_cols_, diameter, spacing);
  }
  else
  {
    ROS_ERROR("Target type not supported, check your launch file, only 2, and 4 for MCircle, and Balls type targets");
  }

  std::string service_name;
  if (!pnh.getParam("service_name", service_name))
  {
    service_name = "TargetLocateService";
  }
  target_locate_server_ = nh_.advertiseService(service_name.c_str(), &TargetLocatorService::executeCallBack, this);
}

bool TargetLocatorService::executeCallBack(target_locator::Request& req, target_locator::Response& res)
{
  ros::NodeHandle nh;
  CameraObservations camera_observations;

  ROSCameraObserver camera_observer(image_topic_, camera_name_);

  // get the focal length and optical center
  double fx, fy, cx, cy;
  double k1, k2, k3, p1, p2;  // unused
  int height, width;          // unused
  if (!camera_observer.pullCameraInfo(fx, fy, cx, cy, k1, k2, k3, p1, p2, width, height))
  {
    ROS_ERROR("could not access camera info");
  }
  camera_observer.clearObservations();
  camera_observer.clearTargets();

  // set the roi to the requested
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
  while (!camera_observer.observationsDone())
    ;
  camera_observer.getObservations(camera_observations);
  int num_observations = (int)camera_observations.size();
  if (num_observations != target_rows_ * target_cols_)
  {
    ROS_ERROR("Target Locator could not find target %d", num_observations);
    return (false);
  }

  // set initial conditions
  target_->pose_.setQuaternion(req.initial_pose.orientation.x, req.initial_pose.orientation.y,
                               req.initial_pose.orientation.z, req.initial_pose.orientation.w);
  target_->pose_.setOrigin(req.initial_pose.position.x, req.initial_pose.position.y, req.initial_pose.position.z);

  Problem problem;
  for (int i = 0; i < num_observations; i++)
  {
    double image_x = camera_observations[i].image_loc_x;
    double image_y = camera_observations[i].image_loc_y;
    Point3d point = target_->pts_[i];  // assume the correct ordering
    CostFunction* cost_function =
        industrial_extrinsic_cal::CameraReprjErrorPK::Create(image_x, image_y, fx, fy, cx, cy, point);
    problem.AddResidualBlock(cost_function, NULL, target_->pose_.pb_pose);
  }
  Solver::Options options;
  Solver::Summary summary;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 1000;
  ceres::Solve(options, &problem, &summary);

  if (summary.termination_type != ceres::NO_CONVERGENCE)
  {
    double error_per_observation = summary.final_cost / num_observations;
    if (error_per_observation <= req.allowable_cost_per_observation)
    {
      res.final_pose.position.x = target_->pose_.x;
      res.final_pose.position.y = target_->pose_.y;
      res.final_pose.position.z = target_->pose_.z;
      res.final_cost_per_observation = error_per_observation;
      target_->pose_.getQuaternion(res.final_pose.orientation.x, res.final_pose.orientation.y,
                                   res.final_pose.orientation.z, res.final_pose.orientation.w);

      return true;
    }
    else
    {
      res.final_cost_per_observation = error_per_observation;
      ROS_ERROR("allowable cost exceeded %f > %f", error_per_observation, req.allowable_cost_per_observation);
      return (false);
    }
  }
}

void TargetLocatorService::initMCircleTarget(int rows, int cols, double circle_dia, double spacing)
{
  target_ = make_shared<industrial_extrinsic_cal::Target>();
  target_->is_moving_ = true;
  target_->target_name_ = "modified_circle_target";
  target_->target_frame_ = "target_frame";
  target_->target_type_ = 2;
  target_->circle_grid_parameters_.pattern_rows = rows;
  target_->circle_grid_parameters_.pattern_cols = cols;
  target_->circle_grid_parameters_.circle_diameter = circle_dia;
  target_->circle_grid_parameters_.is_symmetric = true;
  // create a grid of points
  target_->pts_.clear();
  target_->num_points_ = rows * cols;
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      Point3d point;
      point.x = j * spacing;
      point.y = (rows - 1 - i) * spacing;
      point.z = 0.0;
      target_->pts_.push_back(point);
    }
  }
}

void TargetLocatorService::initBallsTarget()
{
  target_ = make_shared<industrial_extrinsic_cal::Target>();
  target_->target_name_ = "four_ball_target";
  target_->target_frame_ = "target_frame";
  target_->target_type_ = 4;
  // create 4 points
  target_->pts_.clear();
  Point3d point_1, point_2, point_3, point_4, point_5;
  point_1.x = 0.09;
  point_2.x = 0.0;
  point_3.x = -0.057;
  point_4.x = 0.0;
  point_5.x = 0.0;
  point_1.y = 0.0;
  point_2.y = -0.133;
  point_3.y = 0.0;
  point_4.y = 0.0;
  point_4.y = 0.043;
  point_1.z = 0.0;
  point_2.z = 0.0;
  point_3.z = 0.0;
  point_4.z = 0.058;
  point_5.z = 0.0;
  target_->pts_.push_back(point_1);
  target_->pts_.push_back(point_2);
  target_->pts_.push_back(point_3);
  target_->pts_.push_back(point_4);
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
