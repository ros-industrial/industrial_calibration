/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
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
#include <industrial_extrinsic_cal/circle_detector.hpp>
#include <image_transport/image_transport.h>

using cv::CircleDetector;
namespace industrial_extrinsic_cal
{
ROSCameraObserver::ROSCameraObserver(const std::string& camera_topic, const std::string& camera_name)
  : sym_circle_(true)
  , pattern_(pattern_options::Chessboard)
  , pattern_rows_(0)
  , pattern_cols_(0)
  , new_image_collected_(false)
  , store_observation_images_(false)
  , load_observation_images_(false)
  , normalize_calibration_image_(false)
  , image_directory_("")
  , image_number_(0)
  , camera_name_(camera_name)
{
  image_topic_ = camera_topic;
  results_pub_ = nh_.advertise<sensor_msgs::Image>("observer_results_image", 100);
  debug_pub_ = nh_.advertise<sensor_msgs::Image>("observer_raw_image", 100);
  std::string set_camera_info_service = camera_name_ + "/set_camera_info";
  client_ = nh_.serviceClient<sensor_msgs::SetCameraInfo>(set_camera_info_service);

  ros::NodeHandle pnh("~");
  pnh.getParam("image_directory", image_directory_);
  pnh.getParam("store_observation_images", store_observation_images_);
  pnh.getParam("load_observation_images", load_observation_images_);
  pnh.getParam("normalize_calibration_image", normalize_calibration_image_);

  // set up blob/circle detectors parameters are dynamic

  cv::SimpleBlobDetector::Params simple_blob_params;
  if (!pnh.getParam("white_blobs", white_blobs_))
  {
    white_blobs_ = false;
  }
  if (!pnh.getParam("use_circle_detector", use_circle_detector_))
  {
    use_circle_detector_ = false;
  }

  circle_detector_ptr_ = cv::CircleDetector::create();
  blob_detector_ptr_ = cv::SimpleBlobDetector::create(simple_blob_params);

  std::string recon_node_name = "~/" + camera_name;
  rnh_ = new ros::NodeHandle(recon_node_name.c_str());
  server_ = new dynamic_reconfigure::Server<industrial_extrinsic_cal::circle_grid_finderConfig>(*rnh_);

  dynamic_reconfigure::Server<industrial_extrinsic_cal::circle_grid_finderConfig>::CallbackType f;

  f = boost::bind(&ROSCameraObserver::dynReConfCallBack, this, _1, _2);
  server_->setCallback(f);
}

bool ROSCameraObserver::addTarget(boost::shared_ptr<Target> targ, Roi& roi, Cost_function cost_type)
{
  // TODO make a list of targets so that the camera may make more than one set of observations at a time
  // This was what was inteneded by the interface definition, I'm not sure why the first implementation didn't do it.

  cost_type_ = cost_type;

  // set pattern based on target
  ROS_DEBUG_STREAM("Target type: " << targ->target_type_);
  instance_target_ = targ;
  switch (targ->target_type_)
  {
    case pattern_options::Chessboard:
      pattern_ = pattern_options::Chessboard;
      pattern_rows_ = targ->checker_board_parameters_.pattern_rows;
      pattern_cols_ = targ->checker_board_parameters_.pattern_cols;
      break;
    case pattern_options::CircleGrid:
      pattern_ = pattern_options::CircleGrid;
      pattern_rows_ = targ->circle_grid_parameters_.pattern_rows;
      pattern_cols_ = targ->circle_grid_parameters_.pattern_cols;
      sym_circle_ = targ->circle_grid_parameters_.is_symmetric;
      break;
    case pattern_options::ModifiedCircleGrid:
      pattern_ = pattern_options::ModifiedCircleGrid;
      pattern_rows_ = targ->circle_grid_parameters_.pattern_rows;
      pattern_cols_ = targ->circle_grid_parameters_.pattern_cols;
      sym_circle_ = targ->circle_grid_parameters_.is_symmetric;
      break;
    case pattern_options::ARtag:
      pattern_ = pattern_options::ARtag;
      ROS_ERROR_STREAM("AR Tag recognized but pattern not supported yet");
      break;
    case pattern_options::Balls:
      pattern_ = pattern_options::Balls;
      pattern_rows_ = 1;
      pattern_cols_ = targ->num_points_;
      break;
    case pattern_options::SingleBall:
      pattern_ = pattern_options::SingleBall;
      pattern_rows_ = 1;
      pattern_cols_ = 1;
      break;
    default:
      ROS_ERROR_STREAM("target_type does not correlate to a known pattern option (Chessboard, CircleGrid, Balls, "
                       "SingleBall or ARTag)");
      return false;
      break;
  }

  input_roi_.x = roi.x_min;
  input_roi_.y = roi.y_min;
  input_roi_.width = roi.x_max - roi.x_min;
  input_roi_.height = roi.y_max - roi.y_min;
  ROS_DEBUG("ROSCameraObserver added target and roi");

  return true;
}

void ROSCameraObserver::clearTargets()
{
  instance_target_.reset();
}

void ROSCameraObserver::clearObservations()
{
  camera_obs_.clear();
  new_image_collected_ = false;
}

int ROSCameraObserver::getObservations(CameraObservations& cam_obs)
{
  bool successful_find = false;
  bool flipped_successful_find = false;

  if (input_bridge_->image.cols < input_roi_.width || input_bridge_->image.rows < input_roi_.height)
  {
    ROS_ERROR("ROI too big for image size ( image = %d by %d roi= %d %d )", input_bridge_->image.cols,
              input_bridge_->image.rows, input_roi_.width, input_roi_.height);
    return 0;
  }
  ROS_DEBUG("roi size = %d %d", input_roi_.height, input_roi_.width);
  ROS_DEBUG("image size = %d %d", input_bridge_->image.rows, input_bridge_->image.cols);
  image_roi_ = input_bridge_->image(input_roi_);

  // Do a min/max normalization for maximum contrast
  // input_bridge_ is known to be a mono8 image (max = 255)
  if (normalize_calibration_image_)
  {
    cv::Mat norm_img = image_roi_.clone();
    cv::normalize(image_roi_, norm_img, 0, 255, cv::NORM_MINMAX);
    image_roi_ = norm_img;
  }

  ROS_DEBUG("image_roi_ size = %d %d", image_roi_.rows, image_roi_.cols);
  observation_pts_.clear();
  std::vector<cv::KeyPoint> key_points;
  ROS_DEBUG("Pattern type %d, rows %d, cols %d", pattern_, pattern_rows_, pattern_cols_);

  cv::Point large_point;
  cv::Size pattern_size(pattern_cols_, pattern_rows_);  // note they use cols then rows for some unknown reason
  int start_1st_row = 0;
  int end_1st_row = pattern_cols_ - 1;
  int start_last_row = pattern_rows_ * pattern_cols_ - pattern_cols_;
  int end_last_row = pattern_rows_ * pattern_cols_ - 1;

  cv::Size pattern_size_flipped(pattern_rows_, pattern_cols_);  // note they use cols then rows for some unknown reason
  switch (pattern_)
  {
    case pattern_options::Chessboard:
      ROS_DEBUG_STREAM("Finding Chessboard Corners...");
      successful_find =
          cv::findChessboardCorners(image_roi_, pattern_size, observation_pts_, cv::CALIB_CB_ADAPTIVE_THRESH);
      break;
    case pattern_options::CircleGrid:
      if (sym_circle_)  // symetric circle grid
      {
        ROS_DEBUG_STREAM("Finding Circles in grid, symmetric...");

        if (use_circle_detector_)
        {
          successful_find = cv::findCirclesGrid(image_roi_, pattern_size, observation_pts_, cv::CALIB_CB_SYMMETRIC_GRID,
                                                circle_detector_ptr_);
          if (!successful_find)
          {
            successful_find = cv::findCirclesGrid(image_roi_, pattern_size_flipped, observation_pts_,
                                                  cv::CALIB_CB_SYMMETRIC_GRID, circle_detector_ptr_);
            flipped_successful_find = successful_find;
          }
        }
        else
        {
          successful_find =
              cv::findCirclesGrid(image_roi_, pattern_size, observation_pts_, cv::CALIB_CB_SYMMETRIC_GRID);
          if (!successful_find)
          {
            successful_find =
                cv::findCirclesGrid(image_roi_, pattern_size_flipped, observation_pts_, cv::CALIB_CB_SYMMETRIC_GRID);
            flipped_successful_find = successful_find;
          }
        }
      }
      else  // asymetric circle grid
      {
        ROS_DEBUG_STREAM("Finding Circles in grid, asymmetric...");
        if (use_circle_detector_)
        {
          successful_find =
              cv::findCirclesGrid(image_roi_, pattern_size, observation_pts_,
                                  cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, circle_detector_ptr_);
          if (!successful_find)
          {
            successful_find =
                cv::findCirclesGrid(image_roi_, pattern_size_flipped, observation_pts_,
                                    cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, circle_detector_ptr_);
            flipped_successful_find = successful_find;
          }
        }
        else
        {
          successful_find = cv::findCirclesGrid(image_roi_, pattern_size, observation_pts_,
                                                cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
          if (!successful_find)
          {
            successful_find = cv::findCirclesGrid(image_roi_, pattern_size_flipped, observation_pts_,
                                                  cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
            flipped_successful_find = successful_find;
          }
        }
      }
      break;
    case pattern_options::ModifiedCircleGrid:
    {  // contain the scope of automatic variables
      // modified circle grids have one circle at the origin which is 1.5 times larger in diameter than the rest
      std::vector<cv::Point2f> centers;
      if (use_circle_detector_)
      {
        ROS_DEBUG("using circle_detector, to find %dx%d modified circle grid", pattern_rows_, pattern_cols_);
        successful_find =
            cv::findCirclesGrid(image_roi_, pattern_size, centers,
                                cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, circle_detector_ptr_);
        if (!successful_find)
        {
          successful_find =
              cv::findCirclesGrid(image_roi_, pattern_size_flipped, centers,
                                  cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, circle_detector_ptr_);
          flipped_successful_find = successful_find;
        }
      }
      else
      {
        ROS_DEBUG("using simple_blob_detector, to find %dx%d modified grid", pattern_rows_, pattern_cols_);
        successful_find =
            cv::findCirclesGrid(image_roi_, pattern_size, centers,
                                cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blob_detector_ptr_);
        if (!successful_find)
        {
          successful_find =
              cv::findCirclesGrid(image_roi_, pattern_size_flipped, centers,
                                  cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blob_detector_ptr_);
          flipped_successful_find = successful_find;
        }
      }
      if (!successful_find)
      {
        ROS_ERROR("couldn't find %dx%d modified circle target in %s, found only %zu pts", pattern_rows_, pattern_cols_,
                  image_topic_.c_str(), centers.size());
      }
      else
      {
        // Note, this is the same method called in the beginning of findCirclesGrid, unfortunately, they don't return
        // their keypoints
        // Should OpenCV change their method, the keypoint locations may not match, this has a risk of failing with
        // updates to OpenCV
        std::vector<cv::KeyPoint> keypoints;

        if (use_circle_detector_) circle_detector_ptr_->detect(image_roi_, keypoints);
        if (!use_circle_detector_) blob_detector_ptr_->detect(image_roi_, keypoints);

        ROS_DEBUG("found %d keypoints", (int)keypoints.size());

        // if a flipped pattern is found, flip the rows/columns
        int temp_rows = flipped_successful_find ? pattern_cols_ : pattern_rows_;
        int temp_cols = flipped_successful_find ? pattern_rows_ : pattern_cols_;

        // determine which circle is the largest,
        double start_last_row_size = -1.0;
        double start_1st_row_size = -1.0;
        double end_1st_row_size = -1.0;
        double end_last_row_size = -1.0;
        for (int i = 0; i < (int)keypoints.size(); i++)
        {
          double x = keypoints[i].pt.x;
          double y = keypoints[i].pt.y;
          double ksize = keypoints[i].size;
          if (x == centers[start_last_row].x && y == centers[start_last_row].y) start_last_row_size = ksize;
          if (x == centers[end_last_row].x && y == centers[end_last_row].y) end_last_row_size = ksize;
          if (x == centers[start_1st_row].x && y == centers[start_1st_row].y) start_1st_row_size = ksize;
          if (x == centers[end_1st_row].x && y == centers[end_1st_row].y) end_1st_row_size = ksize;
        }

        ROS_DEBUG("start_last_row  %f %f %f", centers[start_last_row].x, centers[start_last_row].y,
                  start_last_row_size);
        ROS_DEBUG("end_last_row %f %f %f", centers[end_last_row].x, centers[end_last_row].y, end_last_row_size);
        ROS_DEBUG("start_1st_row %f %f %f", centers[start_1st_row].x, centers[start_1st_row].y, start_1st_row_size);
        ROS_DEBUG("end_1st_row %f %f %f", centers[end_1st_row].x, centers[end_1st_row].y, end_1st_row_size);
        if (start_last_row_size < 0.0 || start_1st_row_size < 0.0 || end_1st_row_size < 0.0 || end_last_row_size < 0.0)
        {
          ROS_ERROR("No keypoint match for one or more corners");
          return (false);
        }

        // determine if ordering is usual by computing cross product of two vectors normal ordering has z axis positive
        // in cross
        bool usual_ordering =
            true;  // the most common ordering is with points going from left to right then top to bottom
        double v1x, v1y, v2x, v2y;
        v1x = centers[end_last_row].x - centers[start_last_row].x;
        v1y = -centers[end_last_row].y + centers[start_last_row].y;  // reverse because y is positive going down
        v2x = centers[end_1st_row].x - centers[end_last_row].x;
        v2y = -centers[end_1st_row].y + centers[end_last_row].y;
        double cross = v1x * v2y - v1y * v2x;
        if (cross < 0.0)
        {
          usual_ordering = false;
        }
        observation_pts_.clear();

        // largest circle at start of last row
        //       ......   This is a simple picture of the grid with the largest circle indicated by the letter o
        //       o....
        if (start_last_row_size > start_1st_row_size && start_last_row_size > end_1st_row_size &&
            start_last_row_size > end_last_row_size)
        {
          ROS_DEBUG("large circle at start of last row");
          large_point.x = centers[start_last_row].x;
          large_point.y = centers[start_last_row].y;
          if (usual_ordering)
          {  // right side up, no rotation, order is natural, starting from upper left, reads like book
            for (int i = 0; i < (int)centers.size(); i++)
              observation_pts_.push_back(centers[i]);
          }
          else
          {  // unusual ordering
            for (int c = temp_cols - 1; c >= 0; c--)
            {
              for (int r = temp_rows - 1; r >= 0; r--)
              {
                observation_pts_.push_back(centers[r * temp_cols + c]);
              }
            }
          }  // end unusual ordering
        }    // end largest circle at start
        // largest circle at end of 1st row
        //       .....o
        //       ......
        else if (end_1st_row_size > end_last_row_size && end_1st_row_size > start_last_row_size &&
                 end_1st_row_size > start_1st_row_size)
        {
          ROS_DEBUG("large circle at end of 1st row");
          large_point.x = centers[end_1st_row].x;
          large_point.y = centers[end_1st_row].y;
          if (usual_ordering)
          {  // reversed points
            for (int i = (int)centers.size() - 1; i >= 0; i--)
            {
              observation_pts_.push_back(centers[i]);
            }
          }
          else
          {  // unusual ordering
            for (int c = 0; c < temp_cols; c++)
            {
              for (int r = 0; r < temp_rows; r++)
              {
                observation_pts_.push_back(centers[r * temp_cols + c]);
              }
            }
          }  // end unusual ordering
        }    // end largest circle at end of 1st row

        // largest_circle at end of last row
        //       ......
        //       ....o
        else if (end_last_row_size > start_last_row_size && end_last_row_size > end_1st_row_size &&
                 end_last_row_size > start_1st_row_size)
        {
          ROS_DEBUG("large circle at end of last row");
          large_point.x = centers[end_last_row].x;
          large_point.y = centers[end_last_row].y;

          if (usual_ordering)
          {  // 90 80 ... 0, 91 81 ... 1
            for (int c = 0; c < temp_cols; c++)
            {
              for (int r = temp_rows - 1; r >= 0; r--)
              {
                observation_pts_.push_back(centers[r * temp_cols + c]);
              }
            }
          }  // end normal ordering
          else
          {  // unusual ordering 9 8 7 .. 0, 19 18 17 10, 29 28
            for (int c = 0; c < temp_cols; c++)
            {
              for (int r = 0; r < temp_rows; r++)
              {
                observation_pts_.push_back(centers[r * temp_cols + c]);
              }
            }
          }  // end unusual ordering
        }    // end large at end of last row

        // largest circle at start of first row
        // largest_circle at end of last row
        //       o.....
        //       .......
        else if (start_1st_row_size > end_last_row_size && start_1st_row_size > end_1st_row_size &&
                 start_1st_row_size > start_last_row_size)
        {
          ROS_DEBUG("large circle at start of 1st row");
          large_point.x = centers[start_1st_row].x;
          large_point.y = centers[start_1st_row].y;
          if (usual_ordering)
          {  // 9 19 29 ... 99, 8 18 ... 98,
            for (int c = temp_cols - 1; c >= 0; c--)
            {
              for (int r = 0; r < temp_rows; r++)
              {
                observation_pts_.push_back(centers[r * temp_cols + c]);
              }
            }
          }  // end normal ordering
          else
          {  // unusual ordering  90 91 92 ... 99, 80 81 ... 89
            for (int c = temp_cols - 1; c >= 0; c--)
            {
              for (int r = temp_rows - 1; r >= 0; r--)
              {
                observation_pts_.push_back(centers[r * temp_cols + c]);
              }
            }
          }
        }  // end large at start of 1st row
        else
        {
          ROS_ERROR("None of the observed corner circles are bigger than all the others");
          successful_find = false;
        }
      }
    }
    break;  // end modified circle grid case
    case pattern_options::ARtag:
    {
      ROS_ERROR_STREAM("AR Tag recognized but pattern not supported yet");
    }
    break;
    case pattern_options::Balls:
    {  // needed to contain scope of automatic variables to this case
      int rows = last_raw_image_.rows;
      int cols = last_raw_image_.cols;
      const cv::Mat sub_image = last_raw_image_(input_roi_);
      cv::Mat hsv_image;
      cv::cvtColor(sub_image, hsv_image, CV_BGR2HSV);
      cv::Mat red_binary_image(rows, cols, CV_8UC1);
      cv::Mat green_binary_image(rows, cols, CV_8UC1);
      cv::Mat yellow_binary_image(rows, cols, CV_8UC1);
      ros::NodeHandle pnh("~");
      int red_h_max, red_h_min;
      int red_s_min, red_s_max;
      int red_v_min, red_v_max;
      int yellow_h_max, yellow_h_min;
      int yellow_s_min, yellow_s_max;
      int yellow_v_min, yellow_v_max;
      int green_h_max, green_h_min;
      int green_s_min, green_s_max;
      int green_v_min, green_v_max;
      pnh.getParam("red_h_max", red_h_max);
      pnh.getParam("red_h_min", red_h_min);
      pnh.getParam("red_s_min", red_s_min);
      pnh.getParam("red_s_max", red_s_max);
      pnh.getParam("red_v_min", red_v_min);
      pnh.getParam("red_v_max", red_v_max);
      pnh.getParam("yellow_h_max", yellow_h_max);
      pnh.getParam("yellow_h_min", yellow_h_min);
      pnh.getParam("yellow_s_min", yellow_s_min);
      pnh.getParam("yellow_s_max", yellow_s_max);
      pnh.getParam("yellow_v_min", yellow_v_min);
      pnh.getParam("yellow_v_max", yellow_v_max);
      pnh.getParam("green_h_max", green_h_max);
      pnh.getParam("green_h_min", green_h_min);
      pnh.getParam("green_s_min", green_s_min);
      pnh.getParam("green_s_max", green_s_max);
      pnh.getParam("green_v_min", green_v_min);
      pnh.getParam("green_v_max", green_v_max);
      cv::Scalar R_min(red_h_min, red_s_min, red_v_min);
      cv::Scalar R_max(red_h_max, red_s_max, red_v_max);
      cv::Scalar Y_min(yellow_h_min, yellow_s_min, yellow_v_min);
      cv::Scalar Y_max(yellow_h_max, yellow_s_max, yellow_v_max);
      cv::Scalar G_min(green_h_min, green_s_min, green_v_min);
      cv::Scalar G_max(green_h_max, green_s_max, green_v_max);
      cv::inRange(sub_image, R_min, R_max, red_binary_image);
      cv::inRange(sub_image, Y_min, Y_max, yellow_binary_image);
      cv::inRange(sub_image, G_min, G_max, green_binary_image);

      int erosion_type = cv::MORPH_RECT;   // MORPH_RECT MORPH_CROSS MORPH_ELLIPSE
      int dilation_type = cv::MORPH_RECT;  // MORPH_RECT MORPH_CROSS MORPH_ELLIPSE
      int morph_size;
      pnh.getParam("morph_size", morph_size);
      int erosion_size = morph_size;
      int dilation_size = morph_size;
      cv::Mat erosion_element = getStructuringElement(
          erosion_type, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), cv::Point(erosion_size, erosion_size));
      cv::Mat dilation_element = getStructuringElement(
          erosion_type, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), cv::Point(erosion_size, erosion_size));

      // Apply the erosion operation
      erode(red_binary_image, red_binary_image, erosion_element);
      erode(yellow_binary_image, yellow_binary_image, erosion_element);
      erode(green_binary_image, green_binary_image, erosion_element);
      dilate(red_binary_image, red_binary_image, dilation_element);
      dilate(yellow_binary_image, yellow_binary_image, dilation_element);
      dilate(green_binary_image, green_binary_image, dilation_element);
      std::vector<cv::Point2f> centers;
      std::vector<cv::KeyPoint> keypoints;
      if (!use_circle_detector_) blob_detector_ptr_->detect(red_binary_image, keypoints);
      if (use_circle_detector_) circle_detector_ptr_->detect(red_binary_image, keypoints);
      ROS_ERROR("Red keypoints: %d", (int)keypoints.size());
      if (keypoints.size() == 1)
      {
        observation_pts_.push_back(keypoints[0].pt);
        large_point.x = keypoints[0].pt.x;
        large_point.y = keypoints[0].pt.y;
      }
      else
      {
        ROS_ERROR("found %d red blobs, expected one", (int)keypoints.size());
      }
      if (!use_circle_detector_) blob_detector_ptr_->detect(green_binary_image, keypoints);
      if (use_circle_detector_) circle_detector_ptr_->detect(green_binary_image, keypoints);
      ROS_ERROR("Green keypoints: %d", (int)keypoints.size());
      if (keypoints.size() == 1)
      {
        observation_pts_.push_back(keypoints[0].pt);
      }  // end of outer loop
      else
      {
        ROS_ERROR("found %d green blobs, expected one", (int)keypoints.size());
      }
      if (!use_circle_detector_) blob_detector_ptr_->detect(yellow_binary_image, keypoints);
      if (use_circle_detector_) circle_detector_ptr_->detect(yellow_binary_image, keypoints);
      ROS_ERROR("Blue keypoints: %d", (int)keypoints.size());
      if (keypoints.size() == 1)
      {
        observation_pts_.push_back(keypoints[0].pt);
      }  // end of outer loop
      else
      {
        ROS_ERROR("found %d yellow blobs, expected  one", (int)keypoints.size());
      }
      if (observation_pts_.size() != 3)
      {
        bool debug_green, debug_red, debug_yellow;
        pnh.getParam("debug_red", debug_red);
        pnh.getParam("debug_green", debug_green);
        pnh.getParam("debug_yellow", debug_yellow);
        if (debug_yellow && debug_red && debug_green)
        {
          out_bridge_->image = yellow_binary_image | red_binary_image | green_binary_image;
        }
        else if (debug_yellow && debug_red)
        {
          out_bridge_->image = yellow_binary_image | red_binary_image;
        }
        else if (debug_yellow && debug_green)
        {
          out_bridge_->image = yellow_binary_image | green_binary_image;
        }
        else if (debug_red && debug_green)
        {
          out_bridge_->image = red_binary_image | green_binary_image;
        }
        else if (debug_red)
          out_bridge_->image = red_binary_image;
        else if (debug_green)
          out_bridge_->image = green_binary_image;
        else if (debug_yellow)
          out_bridge_->image = yellow_binary_image;
        if (debug_red | debug_green | debug_yellow) debug_pub_.publish(out_bridge_->toImageMsg());
        return false;
      }
      else
      {
        successful_find = true;
      }
    }

    break;
    case pattern_options::SingleBall:
    {  // needed to contain scope of automatic variables to this case
    }
    default:
      ROS_ERROR_STREAM("target_type does not correlate to a known pattern option ");
      return false;
      break;
  }  // end of main switch

  ROS_DEBUG("Number of keypoints found: %d ", (int)observation_pts_.size());

  // account for shift due to input_roi_
  for (int i = 0; i < (int)observation_pts_.size(); i++)
  {
    observation_pts_[i].x += input_roi_.x;
    observation_pts_[i].y += input_roi_.y;
  }

  // draw larger circle at large point
  large_point.x += input_roi_.x;
  large_point.y += input_roi_.y;
  circle(input_bridge_->image, large_point, 3.0, 255, 5);

  // next block of code for publishing the roi as an image, when target is found, circles are placed on image, with a
  // line between pt1 and pt2
  for (int i = 0; i < (int)observation_pts_.size(); i++)
  {
    cv::Point p;
    p.x = observation_pts_[i].x;
    p.y = observation_pts_[i].y;
    if (i == 0)
    {
      circle(input_bridge_->image, p, 2.0, cv::Scalar(0, 0, 0), 5);
    }
    else
    {
      circle(input_bridge_->image, p, 1.0, 255, 5);
    }
  }

  // Draw line through first column of observe points. These correspond to the first set of point in the target
  if (observation_pts_.size() >= pattern_cols_)
  {
    cv::Point p1, p2;
    p1.x = observation_pts_[start_1st_row].x;
    p1.y = observation_pts_[start_1st_row].y;
    p2.x = observation_pts_[end_1st_row].x;
    p2.y = observation_pts_[end_1st_row].y;
    line(input_bridge_->image, p1, p2, 255, 3);
  }

  if (successful_find)
  {  // copy the points found into a camera observation structure indicating their corresponece with target points
    camera_obs_.resize(observation_pts_.size());
    for (int i = 0; i < observation_pts_.size(); i++)
    {
      camera_obs_.at(i).target = instance_target_;
      camera_obs_.at(i).point_id = i;
      camera_obs_.at(i).image_loc_x = observation_pts_.at(i).x;
      camera_obs_.at(i).image_loc_y = observation_pts_.at(i).y;
      camera_obs_.at(i).cost_type = cost_type_;
    }
    cam_obs = camera_obs_;
  }
  else
  {
    ROS_WARN_STREAM("Pattern not found for pattern: " << pattern_);
    if (!sym_circle_) ROS_ERROR("not a symetric target????");
    cv::Point p;
    p.x = image_roi_.cols / 2;
    p.y = image_roi_.rows / 2;
    circle(input_bridge_->image, p, 1.0, 255, 10);
  }

  debug_pub_.publish(input_bridge_->toImageMsg());
  out_bridge_->image = image_roi_;
  results_pub_.publish(out_bridge_->toImageMsg());

  return successful_find;
}

void ROSCameraObserver::triggerCamera()
{
  if (load_observation_images_)
  {
    char number_string[100];
    sprintf(number_string, "%d", image_number_);
    std::string image_name = image_directory_ + "/" + image_topic_ + number_string;
    cv::Mat loaded_color_image = cv::imread(image_name.c_str(), CV_LOAD_IMAGE_COLOR);
    last_raw_image_ = loaded_color_image.clone();
    cv::Mat loaded_mono_image = cv::imread(image_name.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    input_bridge_->image = loaded_mono_image;
    output_bridge_->image = loaded_color_image;
    out_bridge_->image = loaded_mono_image;
    if (loaded_color_image.data && loaded_mono_image.data)
    {
      ROS_DEBUG("Loaded it");
    }
    image_number_++;
  }
  else
  {
    ROS_DEBUG("rosCameraObserver, waiting for image from topic %s", image_topic_.c_str());
    bool done = false;
    while (!done)
    {
      sensor_msgs::ImageConstPtr recent_image =
          ros::topic::waitForMessage<sensor_msgs::Image>(image_topic_, ros::Duration(5.0));
      if (!recent_image)
      {
        ROS_ERROR("No image acquired on topic '%s'", image_topic_.c_str());
        break;
      }

      ROS_DEBUG("captured image in trigger");
      try
      {
        input_bridge_ = cv_bridge::toCvCopy(recent_image, "mono8");
        output_bridge_ = cv_bridge::toCvCopy(recent_image, "bgr8");
        last_raw_image_ = output_bridge_->image.clone();
        out_bridge_ = cv_bridge::toCvCopy(recent_image, "mono8");
        new_image_collected_ = true;
        ROS_DEBUG("cv image created based on ros image");
        done = true;
        ROS_DEBUG("height = %d width=%d step=%d encoding=%s", recent_image->height, recent_image->width,
                  recent_image->step, recent_image->encoding.c_str());
      }
      catch (cv_bridge::Exception& ex)
      {
        ROS_ERROR("Failed to convert image");
        ROS_ERROR("height = %d width=%d step=%d encoding=%s", recent_image->height, recent_image->width,
                  recent_image->step, recent_image->encoding.c_str());
        ROS_WARN_STREAM("cv_bridge exception: " << ex.what());
      }
    }
    if (done) image_number_++;
  }
}

bool ROSCameraObserver::observationsDone()
{
  if (!new_image_collected_)
  {
    return false;
  }
  return true;
}
cv::Mat ROSCameraObserver::getLastImage()
{
  return (last_raw_image_);
}
bool ROSCameraObserver::pushCameraInfo(double& fx, double& fy, double& cx, double& cy, double& k1, double& k2,
                                       double& k3, double& p1, double& p2)

{
  if (camera_name_.empty())
  {
    ROS_ERROR("camera name is not set, not pushing camera info to topic");
    return (false);
  }
  // must pull to get all the header information that we don't compute
  std::string camera_info_topic = camera_name_ + "/camera_info";
  const sensor_msgs::CameraInfoConstPtr& info_msg =
      ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic);
  srv_.request.camera_info.distortion_model = info_msg->distortion_model;
  srv_.request.camera_info.header = info_msg->header;
  srv_.request.camera_info.height = info_msg->height;
  srv_.request.camera_info.width = info_msg->width;
  srv_.request.camera_info.D = info_msg->D;
  srv_.request.camera_info.K = info_msg->K;
  srv_.request.camera_info.R = info_msg->R;
  srv_.request.camera_info.P = info_msg->P;

  // set distortion parameters
  srv_.request.camera_info.D[0] = k1;
  srv_.request.camera_info.D[1] = k2;
  srv_.request.camera_info.D[2] = p1;
  srv_.request.camera_info.D[3] = p2;
  srv_.request.camera_info.D[4] = k3;

  // set camera matrix assuming tx=ty=0.0
  srv_.request.camera_info.K[0] = fx;
  srv_.request.camera_info.K[1] = 0.0;
  srv_.request.camera_info.K[2] = cx;
  srv_.request.camera_info.K[3] = 0.0;
  srv_.request.camera_info.K[4] = fy;
  srv_.request.camera_info.K[5] = cy;
  srv_.request.camera_info.K[6] = 0.0;
  srv_.request.camera_info.K[7] = 0.0;
  srv_.request.camera_info.K[8] = 1.0;

  // set projection matrix for rectified image
  srv_.request.camera_info.P[0] = fx;
  srv_.request.camera_info.P[1] = 0.0;
  srv_.request.camera_info.P[2] = cx;
  srv_.request.camera_info.P[3] = 0.0;
  srv_.request.camera_info.P[4] = 0.0;
  srv_.request.camera_info.P[5] = fy;
  srv_.request.camera_info.P[6] = cy;
  srv_.request.camera_info.P[7] = 0.0;
  srv_.request.camera_info.P[8] = 0.0;
  srv_.request.camera_info.P[9] = 0.0;
  srv_.request.camera_info.P[10] = 1.0;
  srv_.request.camera_info.P[11] = 0.0;

  client_.call(srv_);
  if (!srv_.response.success)
  {
    ROS_ERROR("set request failed: %s", srv_.response.status_message.c_str());
    return (false);
  }
  return (true);
}

bool ROSCameraObserver::pullCameraInfo(double& fx, double& fy, double& cx, double& cy, double& k1, double& k2,
                                       double& k3, double& p1, double& p2)
{
  if (camera_name_.empty())
  {
    ROS_ERROR("camera name is not set, cannot pull camera info from topic");
    return (false);
  }

  // Some cameras have a nested namespace (i.e. /camera/rgb/image_rect_color), so camera info lies on topic
  // '/camera/rgb/camera_info'.
  // Remove the last item after the last '/', and substitute with "camera_info" to get the right camera_info topic
  std::string topic;
  int pos = image_topic_.find_last_of('/');
  topic = image_topic_.substr(0, pos);
  std::string camera_info_topic = topic + "/camera_info";

  const sensor_msgs::CameraInfoConstPtr& info_msg =
      ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic);
  if (info_msg->K[0] == 0)
  {
    ROS_ERROR("camera info msg not correct");
    return (false);
  }
  k1 = info_msg->D[0];
  k2 = info_msg->D[1];
  p1 = info_msg->D[2];
  p2 = info_msg->D[3];
  k3 = info_msg->D[4];
  fx = info_msg->K[0];
  cx = info_msg->K[2];
  fy = info_msg->K[4];
  cy = info_msg->K[5];
  return (true);
}

bool ROSCameraObserver::pullCameraInfo(double& fx, double& fy, double& cx, double& cy, double& k1, double& k2,
                                       double& k3, double& p1, double& p2, int& width, int& height)
{
  if (camera_name_.empty())
  {
    ROS_ERROR("camera name is not set, cannot pull camera info from topic");
    return (false);
  }
  std::string camera_info_topic = "/" + camera_name_ + "/camera_info";
  const sensor_msgs::CameraInfoConstPtr& info_msg =
      ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic, ros::Duration(10.0));
  if (info_msg == NULL)
  {
    ROS_ERROR("camera info message not available for camera %s on topic %s", camera_name_.c_str(),
              camera_info_topic.c_str());
    return (false);
  }
  if (info_msg->K[0] == 0)
  {
    ROS_ERROR("camera info message not correct for camera %s on topic %s", camera_name_.c_str(),
              camera_info_topic.c_str());
    return (false);
  }
  k1 = info_msg->D[0];
  k2 = info_msg->D[1];
  p1 = info_msg->D[2];
  p2 = info_msg->D[3];
  k3 = info_msg->D[4];
  fx = info_msg->K[0];
  cx = info_msg->K[2];
  fy = info_msg->K[4];
  cy = info_msg->K[5];
  width = info_msg->width;
  height = info_msg->height;
  return (true);
}

void ROSCameraObserver::dynReConfCallBack(industrial_extrinsic_cal::circle_grid_finderConfig& config, uint32_t level)
{
  CircleDetector::Params circle_params;
  circle_params.thresholdStep = 10;
  circle_params.minThreshold = config.min_threshold;
  circle_params.maxThreshold = config.max_threshold;
  circle_params.minRepeatability = 2;
  circle_params.minDistBetweenCircles = config.min_distance;
  circle_params.minRadiusDiff = 10;

  circle_params.filterByColor = false;
  if (white_blobs_) circle_params.circleColor = 200;
  if (!white_blobs_) circle_params.circleColor = 0;

  circle_params.filterByArea = config.filter_by_area;
  circle_params.minArea = config.min_area;
  circle_params.maxArea = config.max_area;

  circle_params.filterByCircularity = config.filter_by_circularity;
  circle_params.minCircularity = config.min_circularity;
  circle_params.maxCircularity = config.max_circularity;

  circle_params.filterByInertia = false;
  circle_params.minInertiaRatio = 0.1f;
  circle_params.maxInertiaRatio = std::numeric_limits<float>::max();

  circle_params.filterByConvexity = false;
  circle_params.minConvexity = 0.95f;
  circle_params.maxConvexity = std::numeric_limits<float>::max();

  circle_detector_ptr_ = cv::CircleDetector::create(circle_params);

  cv::SimpleBlobDetector::Params blob_params;
  blob_params.thresholdStep = 10;
  blob_params.minThreshold = config.min_threshold;
  blob_params.maxThreshold = config.max_threshold;
  blob_params.minRepeatability = 2;
  blob_params.minDistBetweenBlobs = config.min_distance;

  blob_params.filterByColor = true;
  if (white_blobs_) blob_params.blobColor = 200;
  if (!white_blobs_) blob_params.blobColor = 0;

  blob_params.filterByArea = config.filter_by_area;
  blob_params.minArea = config.min_area;
  blob_params.maxArea = config.max_area;

  blob_params.filterByCircularity = config.filter_by_circularity;
  blob_params.minCircularity = config.min_circularity;
  blob_params.maxCircularity = config.max_circularity;

  blob_params.filterByInertia = false;
  blob_params.minInertiaRatio = 0.1f;
  blob_params.maxInertiaRatio = std::numeric_limits<float>::max();

  blob_params.filterByConvexity = false;
  blob_params.minConvexity = 0.95f;
  blob_params.maxConvexity = std::numeric_limits<float>::max();

  blob_detector_ptr_ = cv::SimpleBlobDetector::create(blob_params);

  blob_detector_ptr_ = cv::SimpleBlobDetector::create(blob_params);
}

}  // industrial_extrinsic_cal
