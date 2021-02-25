/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2021, Southwest Research Institute
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
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
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
using industrial_extrinsic_cal::ROSBroadcastTransInterface;
using industrial_extrinsic_cal::ROSCameraObserver;
using industrial_extrinsic_cal::Target;
using std::string;

class TargetTracker
{
public:
  TargetTracker(ros::NodeHandle nh);

  ~TargetTracker(){};

  void initMCircleTarget(int rows, int cols, double circle_dia, double spacing);

  void dynReConfCallBack(target_finder::target_finderConfig& config, uint32_t level);

  bool imageCB(const sensor_msgs::Image& image);

  void update();

  void sendTransform();

  bool solvePnPOpencv(CameraObservations& camera_observations);
  bool solvePnPCeres(CameraObservations& camera_observations);

private:
  ros::Publisher debug_pub_;
  ros::NodeHandle nh_;
  ros::Subscriber image_sub_;
  shared_ptr<Target> target_;
  string image_topic_;
  string camera_name_;
  string camera_frame_;
  string data_directory_;
  string target_frame_;
  bool use_circle_detector_;
  bool publish_rviz_markers_;
  size_t expected_num_observations_;
  double allowable_cost_per_observation_;
  boost::shared_ptr<dynamic_reconfigure::Server<target_finder::target_finderConfig> > reconf_srv_;
  dynamic_reconfigure::Server<target_finder::target_finderConfig>::CallbackType reconf_CB_;
  shared_ptr<ROSBroadcastTransInterface> target_to_camera_TI_;
  tf::StampedTransform transform_;
  shared_ptr<ROSCameraObserver> camera_observer_;
  double fx_, fy_, cx_, cy_, k1_, k2_, k3_, p1_, p2_;
  int width_, height_;
  Roi input_roi_;
};

TargetTracker::TargetTracker(ros::NodeHandle nh)
{
  nh_ = nh;
  ros::NodeHandle pnh("~");

  // ROS parameters
  // image_topic, camera_name, use_circle_detector, target_frame, camera_frame, data_directory

  // Dynamic Reconfigure parameters
  // target_tracker/target_rows, target_tracker/target_cols, target_tracker/target_circle_dia,
  // target_tracker/target_spacing

  if (!pnh.getParam("image_topic", image_topic_))
  {
    ROS_ERROR("Must set param:  image_topic");
  }

  if (!pnh.getParam("camera_name", camera_name_))  // need this to get the intrinsics
  {
    ROS_ERROR("Must set param: camera_name");
  }

  if (!pnh.getParam("target_frame", target_frame_))
  {
    ROS_ERROR("Must set param: target_frame");
  }

  if (!pnh.getParam("camera_frame", camera_frame_))
  {
    ROS_ERROR("Must set param: camera_frame");
  }

  if (!pnh.getParam("data_directory", data_directory_))
  {
    ROS_ERROR("Must set param: data_directory");
  }

  if (!pnh.getParam("use_circle_detector", use_circle_detector_))
  {
    use_circle_detector_ = true;
  }

  if (!pnh.getParam("publish_rviz_markers", publish_rviz_markers_))
  {
    publish_rviz_markers_ = false;
  }

  if (!pnh.getParam("allowable_cost_per_observation", allowable_cost_per_observation_))
  {
    allowable_cost_per_observation_ = .25;
  }

  // initialize the transform interface it listens from the frame in the constructor to the reference frame
  target_to_camera_TI_ = shared_ptr<ROSBroadcastTransInterface>(new ROSBroadcastTransInterface(target_frame_));
  target_to_camera_TI_->setReferenceFrame(camera_frame_);
  target_to_camera_TI_->setDataDirectory(data_directory_);
  target_to_camera_TI_->setImmediate(true);

  camera_observer_ = shared_ptr<ROSCameraObserver>(new ROSCameraObserver(image_topic_, camera_name_));
  camera_observer_->pullCameraInfo(fx_, fy_, cx_, cy_, k1_, k2_, k3_, p1_, p2_, width_, height_);
  camera_observer_->use_circle_detector_ = use_circle_detector_;
  input_roi_.x_min = 0;
  input_roi_.y_min = 0;
  input_roi_.x_max = width_;
  input_roi_.y_max = height_;

  // initialize the dynamic reconfigure callback
  reconf_srv_.reset(new dynamic_reconfigure::Server<target_finder::target_finderConfig>(nh_));
  dynamic_reconfigure::Server<target_finder::target_finderConfig>::CallbackType f;
  f = boost::bind(&TargetTracker::dynReConfCallBack, this, _1, _2);
  reconf_srv_->setCallback(f);

  // subscribe to image for updating with each image
  camera_observer_->startTargetTrack();
  debug_pub_ = nh_.advertise<geometry_msgs::Point>("delta_pnp", 1, true);
}

void TargetTracker::dynReConfCallBack(target_finder::target_finderConfig& config, uint32_t level)
{
  // resize target
  initMCircleTarget(config.target_rows, config.target_cols, config.target_circle_dia, config.target_spacing);
}

void TargetTracker::initMCircleTarget(int rows, int cols, double circle_dia, double spacing)
{
  target_ = make_shared<industrial_extrinsic_cal::Target>();

  // constant parameters
  target_->target_name_ = "MCircle";
  target_->target_frame_ = target_frame_;
  target_->target_type_ = pattern_options::ModifiedCircleGrid;
  target_->is_moving_ = false;
  target_->pub_rviz_vis_ = publish_rviz_markers_;
  target_->circle_grid_parameters_.is_symmetric = true;
  target_->setTransformInterface(target_to_camera_TI_);

  // dynamic parameters
  target_->circle_grid_parameters_.pattern_rows = rows;
  target_->circle_grid_parameters_.pattern_cols = cols;
  target_->circle_grid_parameters_.circle_diameter = circle_dia;
  target_->circle_grid_parameters_.spacing = spacing;
  expected_num_observations_ = static_cast<size_t>(rows * cols);
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
  camera_observer_->clearTargets();
  industrial_extrinsic_cal::Cost_function cost_type =
      industrial_extrinsic_cal::cost_functions::CircleCameraReprjErrorPK;
  camera_observer_->addTarget(target_, input_roi_, cost_type);
}

bool TargetTracker::solvePnPCeres(CameraObservations& camera_observations)
{
  Problem problem;
  for (size_t i = 0; i < camera_observations.size(); i++)
  {
    double image_x = camera_observations[i].image_loc_x;
    double image_y = camera_observations[i].image_loc_y;
    Point3d point = target_->pts_[i];  // assume the correct ordering
    CostFunction* cost_function =
        industrial_extrinsic_cal::CameraReprjErrorPK::Create(image_x, image_y, fx_, fy_, cx_, cy_, point);
    problem.AddResidualBlock(cost_function, NULL, target_->pose_.pb_pose);
  }
  Solver::Options options;
  Solver::Summary summary;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 1000;
  ceres::Solve(options, &problem, &summary);

  double cost_per_observation = summary.final_cost / expected_num_observations_;
  bool rtn = false;
  if (summary.termination_type != ceres::NO_CONVERGENCE)
  {
    if (cost_per_observation <= allowable_cost_per_observation_)
    {
      ROS_INFO("number of succesful steps = %d", summary.num_successful_steps);
      rtn = true;
    }
    else
    {
      ROS_ERROR("cost exceeded allowable %f > %f", cost_per_observation, allowable_cost_per_observation_);
    }
  }
  else
  {
    ROS_ERROR("NO CONVERGENCE");
  }
  return rtn;
}

bool TargetTracker::solvePnPOpencv(CameraObservations& camera_observations)
{
  size_t num_obs = camera_observations.size();
  cv::Mat object_pts = cv::Mat(num_obs, 3, CV_64F);
  cv::Mat image_pts = cv::Mat(num_obs, 2, CV_64F);
  cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F);
  cv::Mat distCoeffs;  // = cv::Mat(5, 1, CV_64F);
  cv::Mat rvec = cv::Mat(4, 1, CV_64F);
  cv::Mat tvec = cv::Mat(3, 1, CV_64F);

  for (size_t i = 0; i < camera_observations.size(); i++)
  {
    image_pts.at<double>(i, 0) = camera_observations[i].image_loc_x;
    image_pts.at<double>(i, 1) = camera_observations[i].image_loc_y;
    object_pts.at<double>(i, 0) = target_->pts_[i].x;
    object_pts.at<double>(i, 1) = target_->pts_[i].y;
    object_pts.at<double>(i, 2) = target_->pts_[i].z;
  }
  cameraMatrix.at<double>(0, 0) = fx_;
  cameraMatrix.at<double>(0, 1) = 0.0;
  cameraMatrix.at<double>(0, 2) = cx_;
  cameraMatrix.at<double>(1, 0) = 0.0;
  cameraMatrix.at<double>(1, 1) = fy_;
  cameraMatrix.at<double>(1, 2) = cy_;
  cameraMatrix.at<double>(2, 0) = 0.0;
  cameraMatrix.at<double>(2, 1) = 0.0;
  cameraMatrix.at<double>(2, 2) = 1.0;

  //  distCoeffs.at<double>(0) = k1_;
  //  distCoeffs.at<double>(1) = k2_;
  //  distCoeffs.at<double>(2) = k3_;
  //  distCoeffs.at<double>(3) = p1_;
  //  distCoeffs.at<double>(4) = p2_;
  if (!cv::solvePnP(object_pts, image_pts, cameraMatrix, distCoeffs, rvec, tvec))
  {
    ROS_ERROR("opencv did not solve");
    return false;
  }

  tf::Matrix3x3 R3;
  R3[0][0] = rvec.at<double>(0, 0);
  R3[0][1] = rvec.at<double>(0, 1);
  R3[0][2] = rvec.at<double>(0, 2);
  R3[1][0] = rvec.at<double>(1, 0);
  R3[1][1] = rvec.at<double>(1, 1);
  R3[1][2] = rvec.at<double>(1, 2);
  R3[2][0] = rvec.at<double>(2, 0);
  R3[2][1] = rvec.at<double>(2, 1);
  R3[2][2] = rvec.at<double>(2, 2);
  target_->pose_.setBasis(R3);
  target_->pose_.setOrigin(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

  return true;
}
void TargetTracker::update()
{
  if (!camera_observer_->observationsDone()) return;
  camera_observer_->clearObservations();
  CameraObservations camera_observations;
  if (!camera_observer_->getObservations(camera_observations))
  {
    return;  // error is already reported through ROS_ERROR
  }
  if (!camera_observer_->checkObservationProclivity(camera_observations))
  {
    ROS_ERROR("Proclivity Error");
    return;
  }
  if (camera_observations.size() == expected_num_observations_)
  {
    if (!solvePnPCeres(camera_observations))
    {
      if (!solvePnPOpencv(camera_observations))
      {
        ROS_ERROR("PnP not solved");
      }
    }
    else
    {
      static int do_once = 1;
      if (do_once)
      {
        displayRvizTarget(target_);
        do_once = 0;
      }
      target_->pushTransform();
    }
  }
  else
  {
    ROS_ERROR("number of observations != expected %ld != %ld", camera_observations.size(), expected_num_observations_);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_tracker");
  ros::NodeHandle node_handle("~/target_tracker");
  TargetTracker target_tracker(node_handle);

  while (ros::ok())
  {
    target_tracker.update();
    ros::spinOnce();
  }

  ros::waitForShutdown();
  return 0;
}
