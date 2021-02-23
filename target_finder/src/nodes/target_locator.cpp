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
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <target_finder/target_finderConfig.h>
#include <industrial_extrinsic_cal/camera_observer_trigger.h>
#include <industrial_extrinsic_cal/user_accept.h>
#include <industrial_extrinsic_cal/ros_camera_observer.h>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/ros_transform_interface.h>
#include <industrial_extrinsic_cal/pose_yaml_parser.h>
#include <industrial_extrinsic_cal/ceres_costs_utils.h>
#include <industrial_extrinsic_cal/ceres_costs_utils.hpp>
#include <industrial_extrinsic_cal/ros_target_display.hpp>
#include <target_finder/target_locator.h>
#include <target_finder/target_verify.h>
#include <target_finder/target_save_location.h>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/types.h"

using boost::make_shared;
using boost::shared_ptr;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using industrial_extrinsic_cal::CameraObservations;
using industrial_extrinsic_cal::Point3d;
using industrial_extrinsic_cal::Pose6d;
using industrial_extrinsic_cal::Roi;
using industrial_extrinsic_cal::ROSCameraObserver;
using industrial_extrinsic_cal::ROSListenerTransInterface;
using industrial_extrinsic_cal::Target;
using std::string;
using target_finder::target_locator;
using target_finder::target_save_location;
using target_finder::target_verify;

class TargetLocatorService
{
public:
  TargetLocatorService(ros::NodeHandle nh);
  ~TargetLocatorService()
  {
    delete camera_observer_;
    delete target_to_camera_TI_;
  };
  bool executeCallBack(target_locator::Request& req, target_locator::Response& res);
  bool verifyCallBack(target_verify::Request& req, target_verify::Response& res);
  bool saveLocCallBack(target_save_location::Request& req, target_save_location::Response& res);
  void initMCircleTarget(int rows, int cols, double circle_dia, double spacing);
  void dynReConfCallBack(target_finder::target_finderConfig& config, uint32_t level);
  Pose6d loadPose(std::string filepath);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer target_locate_server_;  // provides the location of the target as a service
  ros::ServiceServer target_verify_server_;  // verifies the location matches stored location
  ros::ServiceServer target_savelo_server_;  // saves the location of the target to a file
  shared_ptr<Target> target_;
  string image_topic_;
  string camera_name_;
  string camera_frame_;
  string target_frame_;
  string data_directory_;
  int target_type_;
  int target_rows_;
  int target_cols_;
  boost::shared_ptr<dynamic_reconfigure::Server<target_finder::target_finderConfig> > reconf_srv_;
  dynamic_reconfigure::Server<target_finder::target_finderConfig>::CallbackType reconf_CB_;
  ROSCameraObserver* camera_observer_;
  ROSListenerTransInterface* target_to_camera_TI_;
  // listen:pullTransform(), read:loadPose(scene_, file) and write:saveCurrentPose(scene, file)
  // scene is optional if file is not ""
  // otherwise scene is used to create the name of the file
  // full file path takes the form data_directory_/filexxx.yaml
};

TargetLocatorService::TargetLocatorService(ros::NodeHandle nh)
{
  nh_ = nh;
  ros::NodeHandle pnh("~");

  // In launch you may also set the dynamic reconfigure variables of:
  // target_locator/target_rows
  // target_locator/target_cols
  // target_locator/target_circle_dia
  // target_locator/target_spacing

  if (!pnh.getParam("image_topic", image_topic_))
  {
    ROS_ERROR("Must set param:  image_topic");
  }

  if (!pnh.getParam("camera_name", camera_name_))  // need this to get the intrinsics
  {
    ROS_ERROR("Must set param: camera_name");
  }

  camera_observer_ = new ROSCameraObserver(image_topic_, camera_name_);

  camera_observer_->use_circle_detector_ = true;
  pnh.getParam("use_circle_detector", camera_observer_->use_circle_detector_);

  if (!pnh.getParam("target_frame", target_frame_))  // need this to get the intrinsics
  {
    ROS_ERROR("Must set param: target_frame");
  }

  if (!pnh.getParam("camera_frame", camera_frame_))  // need this to get the intrinsics
  {
    ROS_ERROR("Must set param: camera_frame");
  }

  if (!pnh.getParam("data_directory", data_directory_))  // need this to get the intrinsics
  {
    ROS_ERROR("Must set param: data_directory");
  }

  // initialize the transform interface it listens from the frame in the constructor to the reference frame
  target_to_camera_TI_ = new industrial_extrinsic_cal::ROSListenerTransInterface(target_frame_);
  target_to_camera_TI_->setReferenceFrame(camera_frame_);
  target_to_camera_TI_->setDataDirectory(data_directory_);
  target_type_ = pattern_options::ModifiedCircleGrid;
  target_locate_server_ = nh_.advertiseService("target_locate_srv", &TargetLocatorService::executeCallBack, this);
  target_verify_server_ = nh_.advertiseService("target_verify_srv", &TargetLocatorService::verifyCallBack, this);
  target_savelo_server_ =
      nh_.advertiseService("target_save_location_srv", &TargetLocatorService::saveLocCallBack, this);

  reconf_srv_.reset(new dynamic_reconfigure::Server<target_finder::target_finderConfig>(nh_));
  dynamic_reconfigure::Server<target_finder::target_finderConfig>::CallbackType f;
  f = boost::bind(&TargetLocatorService::dynReConfCallBack, this, _1, _2);
  reconf_srv_->setCallback(f);
}

void TargetLocatorService::dynReConfCallBack(target_finder::target_finderConfig& config, uint32_t level)
{
  // resize target
  target_rows_ = config.target_rows;
  target_cols_ = config.target_cols;
  initMCircleTarget(config.target_rows, config.target_cols, config.target_circle_dia, config.target_spacing);
}

bool TargetLocatorService::executeCallBack(target_locator::Request& req, target_locator::Response& res)
{
  ros::NodeHandle nh;
  CameraObservations camera_observations;
  // get the focal length and optical center
  double fx, fy, cx, cy;
  double k1, k2, k3, p1, p2;  // unused
  int height, width;          // unused
  if (!camera_observer_->pullCameraInfo(fx, fy, cx, cy, k1, k2, k3, p1, p2, width, height))
  {
    ROS_ERROR("could not access camera info");
  }
  camera_observer_->clearObservations();
  camera_observer_->clearTargets();

  // set the roi to whole image, lets keep this simple
  Roi roi;
  if (req.roi.height == 0 || req.roi.width == 0)  // both must be non-zero otherwise use whole image
  {
    roi.x_min = 0;
    roi.y_min = 0;
    roi.x_max = width;
    roi.y_max = height;
  }
  else
  {
    roi.x_min = req.roi.x_offset;
    roi.y_min = req.roi.y_offset;
    roi.x_max = req.roi.x_offset + req.roi.width;
    roi.y_max = req.roi.y_offset + req.roi.height;
  }

  industrial_extrinsic_cal::Cost_function cost_type;

  camera_observer_->clearTargets();
  camera_observer_->clearObservations();
  camera_observer_->addTarget(target_, roi, cost_type);
  camera_observer_->triggerCamera();

  while (!camera_observer_->observationsDone());

  camera_observer_->getObservations(camera_observations);
  int num_observations = (int)camera_observations.size();
  if (num_observations != target_rows_ * target_cols_)
  {
    ROS_ERROR("Target Locator could not find target %d", num_observations);
    res.success = false;
    return (true);
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

  double error_per_observation = summary.final_cost / num_observations;
  res.cost_per_observation = error_per_observation;
  if (summary.termination_type != ceres::NO_CONVERGENCE)
  {
    if (error_per_observation <= req.allowable_cost_per_observation)
    {
      res.final_pose.position.x = target_->pose_.x;
      res.final_pose.position.y = target_->pose_.y;
      res.final_pose.position.z = target_->pose_.z;
      target_->pose_.getQuaternion(res.final_pose.orientation.x, res.final_pose.orientation.y,
                                   res.final_pose.orientation.z, res.final_pose.orientation.w);
      res.success = true;
      return (true);
    }
    else
    {
      ROS_ERROR("allowable cost exceeded %f > %f", error_per_observation, req.allowable_cost_per_observation);
      res.success = false;
      return (true);
    }
  }
  else
  {
    ROS_ERROR("allowable cost exceeded %f > %f", error_per_observation, req.allowable_cost_per_observation);
  }
  ROS_ERROR("NO CONVERGENCE");
  res.success = false;
  return (true);
}

bool TargetLocatorService::verifyCallBack(target_verify::Request& req, target_verify::Response& res)
{
  // call executeCallback and compare results to that found in the saved pose in the file
  target_locator::Request tl_req;
  target_locator::Response tl_res;
  tl_req.allowable_cost_per_observation = req.allowable_cost_per_observation;
  tl_req.roi = req.roi;

  if (req.initial_pose.orientation.x == 0 && req.initial_pose.orientation.y == 0 &&
      req.initial_pose.orientation.z == 0 && req.initial_pose.orientation.w == 0 && req.initial_pose.position.x == 0 &&
      req.initial_pose.position.y == 0 && req.initial_pose.position.z == 0)
  {
    Pose6d TtoC = target_to_camera_TI_->pullTransform();  // this listens to the transform from TF
    TtoC.getQuaternion(tl_req.initial_pose.orientation.x, tl_req.initial_pose.orientation.y,
                       tl_req.initial_pose.orientation.z, tl_req.initial_pose.orientation.w);
    tl_req.initial_pose.position.x = TtoC.x;
    tl_req.initial_pose.position.y = TtoC.y;
    tl_req.initial_pose.position.z = TtoC.z;
  }
  else
  {
    tl_req.initial_pose = req.initial_pose;
  }

  if (!executeCallBack(tl_req, tl_res))
  {
    ROS_ERROR("Pose Estimation of Target Failed");
    res.position_error = -1.0;  // set to negative 1 since the pose is not computed
    res.cost_per_observation = tl_res.cost_per_observation;
    res.success = false;
  }
  else
  {
    res.cost_per_observation = tl_res.cost_per_observation;
    // compare to saved position
    std::string file_name = req.file_name.data;
    if (!target_to_camera_TI_->loadPose(0, file_name))
    {
      ROS_ERROR("could not load the pose from %s", file_name.c_str());
    }
    Pose6d P = target_to_camera_TI_->getCurrentPose();
    double sqd = (P.x - tl_res.final_pose.position.x) * (P.x - tl_res.final_pose.position.x) +
                 (P.y - tl_res.final_pose.position.y) * (P.y - tl_res.final_pose.position.y) +
                 (P.z - tl_res.final_pose.position.z) * (P.z - tl_res.final_pose.position.z);
    double position_error = sqrt(sqd);
    res.position_error = position_error;
    if (position_error < req.max_error)
    {
      res.success = true;
    }
    else
    {
      ROS_ERROR("Validation Failed old position:[%6.3lf %6.3lf %6.3lf], new [%6.3lf %6.3lf %6.3lf] error = %6.3lf max "
                "error=%6.3lf ",
                P.x, P.y, P.z, tl_res.final_pose.position.x, tl_res.final_pose.position.y, tl_res.final_pose.position.z,
                position_error, req.max_error);
      res.success = false;
    }
  }
  return true;
}

bool TargetLocatorService::saveLocCallBack(target_save_location::Request& req, target_save_location::Response& res)
{
  // call executeCallback and compare results to that found in the saved pose in the file
  target_locator::Request tl_req;
  target_locator::Response tl_res;
  res.success = false;
  tl_req.allowable_cost_per_observation = req.allowable_cost_per_observation;
  tl_req.roi = req.roi;

  if (req.initial_pose.orientation.x == 0 && req.initial_pose.orientation.y == 0 &&
      req.initial_pose.orientation.z == 0 && req.initial_pose.orientation.w == 0 && req.initial_pose.position.x == 0 &&
      req.initial_pose.position.y == 0 && req.initial_pose.position.z == 0)
  {
    Pose6d TtoC = target_to_camera_TI_->pullTransform();  // this listens to the transform from TF
    TtoC.getQuaternion(tl_req.initial_pose.orientation.x, tl_req.initial_pose.orientation.y,
                       tl_req.initial_pose.orientation.z, tl_req.initial_pose.orientation.w);
    tl_req.initial_pose.position.x = TtoC.x;
    tl_req.initial_pose.position.y = TtoC.y;
    tl_req.initial_pose.position.z = TtoC.z;
  }
  else
  {
    tl_req.initial_pose = req.initial_pose;
  }

  if (executeCallBack(tl_req, tl_res))
  {
    res.success = true;
    Pose6d P;
    P.x = tl_res.final_pose.position.x;
    P.y = tl_res.final_pose.position.y;
    P.z = tl_res.final_pose.position.z;
    P.setQuaternion(tl_res.final_pose.orientation.x, tl_res.final_pose.orientation.y, tl_res.final_pose.orientation.z,
                    tl_res.final_pose.orientation.w);
    target_to_camera_TI_->setCurrentPose(P);
    target_to_camera_TI_->saveCurrentPose(0, req.file_name.data);
  }
  res.cost_per_observation = tl_res.cost_per_observation;
  return true;
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_locator_service");
  ros::NodeHandle node_handle("~/target_locator");
  TargetLocatorService target_locator(node_handle);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
