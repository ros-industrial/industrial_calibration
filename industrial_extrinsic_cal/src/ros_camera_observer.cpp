/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <industrial_extrinsic_cal/ros_camera_observer.h>

namespace industrial_extrinsic_cal
{

ROSCameraObserver::ROSCameraObserver(std::string &camera_topic) :
    sym_circle_(true), pattern_(pattern_options::Chessboard), pattern_rows_(0), pattern_cols_(0)
{
  image_topic_ = camera_topic;
  results_pub_ = nh_.advertise<sensor_msgs::Image>("observer_results_image", 100);
}

bool ROSCameraObserver::addTarget(boost::shared_ptr<Target> targ, Roi &roi)
{
  //set pattern based on target
  ROS_INFO_STREAM("Target type: "<<targ->target_type);
  instance_target_ = targ;
  switch (targ->target_type)
  {
    case pattern_options::Chessboard:
      pattern_ = pattern_options::Chessboard;
      break;
    case pattern_options::CircleGrid:
      pattern_ = pattern_options::CircleGrid;
      break;
    case pattern_options::ARtag:
      pattern_ = pattern_options::ARtag;
      break;
    default:
      ROS_ERROR_STREAM("target_type does not correlate to a known pattern option (Chessboard, CircleGrid or ARTag)");
      return false;
      break;
  }

  if (pattern_ != 0 && pattern_ != 1 && pattern_ != 2)
  {
    ROS_ERROR_STREAM("Unknown pattern, based on target_type");
    return false;
  }

  //set pattern rows/cols based on target
  switch (pattern_)
  {
    case pattern_options::Chessboard:
      pattern_rows_ = targ->checker_board_parameters.pattern_rows;
      pattern_cols_ = targ->checker_board_parameters.pattern_cols;
      break;
    case pattern_options::CircleGrid:
      pattern_rows_ = targ->circle_grid_parameters.pattern_rows;
      pattern_cols_ = targ->circle_grid_parameters.pattern_cols;
      sym_circle_ = targ->circle_grid_parameters.is_symmetric;
      break;
    case pattern_options::ARtag:
      ROS_ERROR_STREAM("AR Tag recognized but pattern not supported yet");
      return false;
      break;
    default:
      ROS_ERROR_STREAM("pattern_ does not correlate to a known pattern option (Chessboard, CircleGrid or ARTag)");
      return false;
      break;
  }

  sensor_msgs::ImageConstPtr recent_image = ros::topic::waitForMessage<sensor_msgs::Image>(image_topic_);
  try
  {
    input_bridge_ = cv_bridge::toCvCopy(recent_image, "mono8");
    output_bridge_ = cv_bridge::toCvCopy(recent_image, "bgr8");
    out_bridge_ = cv_bridge::toCvCopy(recent_image, "mono8");
    ROS_INFO_STREAM("cv image created based on ros image");
  }
  catch (cv_bridge::Exception& ex)
  {
    ROS_ERROR("Failed to convert image");
    ROS_WARN_STREAM("cv_bridge exception: "<<ex.what());
    return false;
  }

  cv::Rect input_ROI(roi.x_min, roi.y_min, roi.x_max - roi.x_min, roi.y_max - roi.y_min); //Rect takes in x,y,width,height
  //ROS_INFO_STREAM("image ROI region created");
  if (input_bridge_->image.cols < input_ROI.width || input_bridge_->image.rows < input_ROI.height)
  {
    ROS_ERROR_STREAM("ROI too big for image size");
    return false;
  }

  image_roi_ = input_bridge_->image(input_ROI);

  output_bridge_->image = output_bridge_->image(input_ROI);
  out_bridge_->image = image_roi_;
  ROS_INFO_STREAM("output image size: " <<output_bridge_->image.rows<<" x "<<output_bridge_->image.cols);
  results_pub_.publish(out_bridge_->toImageMsg());
  return true;
}

void ROSCameraObserver::clearTargets()
{
  instance_target_.reset();
}

void ROSCameraObserver::clearObservations()
{
  camera_obs_.observation.clear();
}

int ROSCameraObserver::getObservations(CameraObservations &cam_obs)
{
  bool successful_find = false;

  switch (pattern_)
  {
    case pattern_options::Chessboard:
      ROS_INFO_STREAM("Finding Chessboard Corners...");
      successful_find = cv::findChessboardCorners(image_roi_, cv::Size(pattern_rows_, pattern_cols_), observation_pts_,
                                                  cv::CALIB_CB_ADAPTIVE_THRESH);
      break;
    case pattern_options::CircleGrid:
      if (sym_circle_)
      {
        ROS_INFO_STREAM("Finding Circles in grid, symmetric...");
        successful_find = cv::findCirclesGrid(image_roi_, cv::Size(pattern_rows_, pattern_cols_), observation_pts_,
                                              cv::CALIB_CB_SYMMETRIC_GRID);
      }
      else
      {
        ROS_INFO_STREAM("Finding Circles in grid, asymmetric...");
        successful_find = cv::findCirclesGrid(image_roi_, cv::Size(pattern_rows_, pattern_cols_), observation_pts_,
                                              cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
      }
      break;
  }
  if (!successful_find)
  {
    ROS_WARN_STREAM("Pattern not found for pattern: "<<pattern_ <<" with symmetry: "<< sym_circle_);
  }

  ROS_INFO_STREAM("Number of points found on board: "<<observation_pts_.size());
  camera_obs_.observation.resize(observation_pts_.size());
  for (int i = 0; i < observation_pts_.size(); i++)
  {
    camera_obs_.observation.at(i).target = instance_target_;
    camera_obs_.observation.at(i).point_id = i;
    camera_obs_.observation.at(i).image_loc_x = observation_pts_.at(i).x;
    camera_obs_.observation.at(i).image_loc_y = observation_pts_.at(i).y;
  }

  cam_obs = camera_obs_;
  if (successful_find && camera_obs_.observation.size() != 0)
  {
    return 1;
  }
  else
    return 0;
}

} //industrial_extrinsic_cal
