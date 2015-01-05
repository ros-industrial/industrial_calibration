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
using cv::CircleDetector;
namespace industrial_extrinsic_cal
{

ROSCameraObserver::ROSCameraObserver(const std::string &camera_topic) :
  sym_circle_(true), pattern_(pattern_options::Chessboard), pattern_rows_(0), pattern_cols_(0), new_image_collected_(false), 
  store_observation_images_(false), load_observation_images_(false), image_directory_(""), image_number_(0)
{
  image_topic_ = camera_topic;  
  results_pub_ = nh_.advertise<sensor_msgs::Image>("observer_results_image", 100);
  junk_pub_ = nh_.advertise<sensor_msgs::Image>("observer_raw_image", 100);
 
  ros::NodeHandle pnh("~");
  pnh.getParam("image_directory", image_directory_);
  pnh.getParam("store_observation_images", store_observation_images_);
  pnh.getParam("load_observation_images", load_observation_images_);

  // set up the circle detector
  CircleDetector::Params params;
  params.thresholdStep = 10;
  params.minThreshold = 50;
  params.maxThreshold = 220;
  params.minRepeatability = 2;
  params.minDistBetweenCircles = 2.0;
  params.minRadiusDiff = 10;

  params.filterByColor = false;
  params.circleColor = 0;
  
  params.filterByArea = false;
  params.minArea = 25;
  params.maxArea = 5000;
  
  params.filterByCircularity = false;
  params.minCircularity = 0.8f;
  params.maxCircularity = std::numeric_limits<float>::max();
  
  params.filterByInertia = false;
  params.minInertiaRatio = 0.1f;
  params.maxInertiaRatio = std::numeric_limits<float>::max();
  
  params.filterByConvexity = false;
  params.minConvexity = 0.95f;
  params.maxConvexity = std::numeric_limits<float>::max();

  // set up and create the detector using the parameters
  //  circle_detector_ptr_ = new cv::CircleDetector(params);
  circle_detector_ptr_ = new cv::SimpleBlobDetector();
}

bool ROSCameraObserver::addTarget(boost::shared_ptr<Target> targ, Roi &roi, Cost_function cost_type)
{
  // TODO make a list of targets so that the camera may make more than one set of observations at a time
  // This was what was inteneded by the interface definition, I'm not sure why the first implementation didn't do it.

  cost_type_ = cost_type; 

  //set pattern based on target
  ROS_DEBUG_STREAM("Target type: "<<targ->target_type_);
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
      pattern_cols_  = targ->circle_grid_parameters_.pattern_cols;
      sym_circle_    = targ->circle_grid_parameters_.is_symmetric;
      break;
    case pattern_options::ARtag:
      pattern_ = pattern_options::ARtag;
      ROS_ERROR_STREAM("AR Tag recognized but pattern not supported yet");
      break;
    default:
      ROS_ERROR_STREAM("target_type does not correlate to a known pattern option (Chessboard, CircleGrid or ARTag)");
      return false;
      break;
  }

  input_roi_.x=roi.x_min;
  input_roi_.y= roi.y_min;
  input_roi_.width= roi.x_max - roi.x_min;
  input_roi_.height= roi.y_max - roi.y_min;

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
int ROSCameraObserver::getObservations(CameraObservations &cam_obs)
{
  bool successful_find = false;


  if (input_bridge_->image.cols < input_roi_.width || input_bridge_->image.rows < input_roi_.height)
  {
    ROS_ERROR("ROI too big for image size ( image = %d by %d roi= %d %d )", 
	      input_bridge_->image.cols, input_bridge_->image.rows, input_roi_.width, input_roi_.height );
    return 0;
  }
  ROS_DEBUG("roi size = %d %d", input_roi_.height, input_roi_.width);
  ROS_DEBUG("image size = %d %d", input_bridge_->image.rows, input_bridge_->image.cols);
  image_roi_ = input_bridge_->image(input_roi_);
  ROS_DEBUG("image_roi_ size = %d %d", image_roi_.rows, image_roi_.cols);

  observation_pts_.clear();
  std::vector<cv::KeyPoint> key_points;
  ROS_DEBUG("Pattern type %d, rows %d, cols %d",pattern_,pattern_rows_,pattern_cols_);
  
  cv::Size pattern_size(pattern_cols_, pattern_rows_); // note they use cols then rows for some unknown reason
  switch (pattern_)
    {
    case pattern_options::Chessboard:
      ROS_DEBUG_STREAM("Finding Chessboard Corners...");
      successful_find = cv::findChessboardCorners(image_roi_, pattern_size, observation_pts_, cv::CALIB_CB_ADAPTIVE_THRESH);
      break;
    case pattern_options::CircleGrid:
      if (sym_circle_) // symetric circle grid
	{
	  ROS_DEBUG_STREAM("Finding Circles in grid, symmetric...");
	  successful_find = cv::findCirclesGrid(image_roi_, pattern_size, observation_pts_, cv::CALIB_CB_SYMMETRIC_GRID);
	}
      else         // asymetric circle grid
	{
	  ROS_DEBUG_STREAM("Finding Circles in grid, asymmetric...");
	  successful_find = cv::findCirclesGrid(image_roi_, pattern_size , observation_pts_, 
						cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
	}
      break;
    case pattern_options::ModifiedCircleGrid:
      // modified circle grids have one circle at the origin which is 1.5 times larger in diameter than the rest
      ROS_DEBUG_STREAM("Finding Circles in modified symetric grid");
      std::vector<cv::Point2f> centers;
      successful_find = cv::findCirclesGrid(image_roi_, pattern_size, centers, 
					    cv::CALIB_CB_SYMMETRIC_GRID,
					    circle_detector_ptr_);
      if(!successful_find){
	ROS_ERROR("couldn't find");
	out_bridge_->image = image_roi_;
	junk_pub_.publish(out_bridge_->toImageMsg());
	return 0;
      }
      // Note, this is the same method called in the beginning of findCirclesGrid, unfortunately, they don't return their keypoints
      // Should OpenCV change their method, the keypoint locations may not match, this has a risk of failing with
      // updates to OpenCV
      std::vector<cv::KeyPoint> keypoints;
      circle_detector_ptr_->detect(image_roi_, keypoints);
      ROS_DEBUG("found %d keypoints", keypoints.size());
      if(successful_find){ // determine orientation, and sort points to correct correspondence
	int lw_lt_index = pattern_rows_*pattern_cols_ - pattern_cols_; // lower right point's index
	int lw_rt_index = pattern_rows_*pattern_cols_ -1; // lower left point's index
	int up_lt_index = 0;	// upper left point's index
	int up_rt_index = pattern_cols_-1; // upper right point's index

	double lw_lt_size = -1.0; // lower left point's size in pixels
	double up_lt_size = -1.0;// upper left point's size in pixels
	double up_rt_size = -1.0;// upper right point's size in pixels
	double lw_rt_size = -1.0;// lower right point's size in pixels
	for(int i=0; i<(int)keypoints.size(); i++){

	  double x = keypoints[i].pt.x;
	  double y = keypoints[i].pt.y;
	  double ksize = keypoints[i].size;
	  if(x == centers[lw_lt_index].x && y == centers[lw_lt_index].y) lw_lt_size = ksize;
	  if(x == centers[lw_rt_index].x && y == centers[lw_rt_index].y) lw_rt_size = ksize;
	  if(x == centers[up_lt_index].x && y == centers[up_lt_index].y) up_lt_size = ksize;
	  if(x == centers[up_rt_index].x && y == centers[up_rt_index].y) up_rt_size = ksize;
	}
	ROS_DEBUG("lower_left  %f %f %f", centers[lw_lt_index].x, centers[lw_lt_index].y, lw_lt_size);
	ROS_DEBUG("lower_right %f %f %f", centers[lw_rt_index].x, centers[lw_rt_index].y, lw_rt_size);
	ROS_DEBUG("upper_left  %f %f %f", centers[up_lt_index].x, centers[up_lt_index].y, lw_lt_size);
	ROS_DEBUG("uper_right %f %f %f", centers[up_lt_index].x, centers[up_lt_index].y, lw_lt_size);
	if(lw_lt_size <0.0 || up_lt_size < 0.0 || up_rt_size <0.0 || lw_rt_size < 0.0){
	  ROS_ERROR("No keypoint match for one or more corners");
	  return(false);
	}
	
	observation_pts_.clear();
	// lower left
	if(lw_lt_size >up_lt_size && lw_lt_size > up_rt_size && lw_lt_size > lw_rt_size){
	  ROS_DEBUG("large lower left");
	  // right side up, no rotation, order is natural, starting from upper left, read like book
	  for(int i=0; i<(int) centers.size(); i++) observation_pts_.push_back(centers[i]);
	}
	// upper right
	else if( up_rt_size > lw_rt_size && up_rt_size > lw_lt_size && up_rt_size > up_lt_size){
	  ROS_DEBUG("large upper right");
	  // 180 degree rotation reverse order
	  for(int i=(int) centers.size()-1; i>= 0; i--){
	    observation_pts_.push_back(centers[i]);
	  }
	}
	// lower right
	else if( lw_rt_size > lw_lt_size && lw_rt_size > up_rt_size && lw_rt_size > up_lt_size){
	  ROS_DEBUG("large lower right");
	  // -90 degree rotation
	  for(int c=0; c<pattern_cols_; c++){
	    for(int r=pattern_rows_-1; r>=0; r--){
	      observation_pts_.push_back(centers[r*pattern_cols_ +c]);
	    }
	  }
	}
	// upper left
	else if(up_lt_size > lw_rt_size && up_lt_size > up_rt_size && up_lt_size > lw_lt_size){
	  ROS_DEBUG("large upper left");
	  // 90 degree rotation
	  for(int c = pattern_cols_ -1; c>=0; c--){
	    for(int r=0; r<pattern_rows_; r++){
	      observation_pts_.push_back(centers[r*pattern_cols_ + c]);
	    }
	  }
	}
	else
	  {
	    ROS_ERROR("None of the observed corner circles are bigger than all the others");
	    successful_find = false;
	  }
      }// end of successful find within this case
      break;// end modified circle grid case
    }// end of main switch
  
  ROS_DEBUG("Number of keypoints found: %d ", (int)observation_pts_.size());

  // account for shift due to input_roi_
  for(int i=0; i<(int)observation_pts_.size(); i++){
    observation_pts_[i].x += input_roi_.x;
    observation_pts_[i].y += input_roi_.y;
  }

  
  // next block of code for publishing the roi as an image, when target is found, circles are placed on image, with a line between pt1 and pt2
  for(int i=0;i<(int)observation_pts_.size();i++){
    cv::Point p;
    p.x = observation_pts_[i].x;
    p.y = observation_pts_[i].y;
    circle(input_bridge_->image,p,1.0,255,5);
  }
  
  // Draw line through first column of observe points. These correspond to the first set of point in the target
  if(observation_pts_.size()>pattern_cols_){
    cv::Point p1,p2;
    p1.x = observation_pts_[0].x; 
    p1.y = observation_pts_[0].y; 
    p2.x = observation_pts_[pattern_cols_-1].x; 
    p2.y = observation_pts_[pattern_cols_-1].y; 
    line(input_bridge_->image,p1,p2,255,3);
  }
  out_bridge_->image = image_roi_;

  
  junk_pub_.publish(input_bridge_->toImageMsg());
  if(!successful_find){
    ROS_WARN_STREAM("Pattern not found for pattern: "<<pattern_);
    if(!sym_circle_) ROS_ERROR("not a symetric target????");
    cv::Point p;
    p.x = image_roi_.cols/2;
    p.y = image_roi_.rows/2;
    circle(input_bridge_->image,p,1.0,255,10);
    out_bridge_->image = image_roi_;
    results_pub_.publish(out_bridge_->toImageMsg());
    return 0;
  }
  else{
    results_pub_.publish(out_bridge_->toImageMsg());
  }


  // copy the points found into a camera observation structure indicating their corresponece with target points
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
  return 1;
}


void ROSCameraObserver::triggerCamera()
{
  if(load_observation_images_){
    char  number_string[100];
    sprintf(number_string, "%d", image_number_);
    std::string image_name = image_directory_ + "/" + image_topic_ + number_string;
    cv::Mat loaded_color_image = cv::imread(image_name.c_str(), CV_LOAD_IMAGE_COLOR);
    last_raw_image_ = loaded_color_image.clone();
    cv::Mat loaded_mono_image = cv::imread(image_name.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    input_bridge_->image = loaded_mono_image;
    output_bridge_->image = loaded_color_image;
    out_bridge_->image = loaded_mono_image;
    if(loaded_color_image.data && loaded_mono_image.data){
      ROS_DEBUG("Loaded it");
    }
  }
  else{
    ROS_DEBUG("rosCameraObserver, waiting for image from topic %s",image_topic_.c_str());
    bool done=false;
    while(!done){
      sensor_msgs::ImageConstPtr recent_image = ros::topic::waitForMessage<sensor_msgs::Image>(image_topic_);
      
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
	  ROS_DEBUG("height = %d width=%d step=%d encoding=%s", 
		    recent_image->height, 
		    recent_image->width, 
		    recent_image->step,  
		    recent_image->encoding.c_str());

	}
      catch (cv_bridge::Exception& ex)
	{
	  ROS_ERROR("Failed to convert image");
	  ROS_ERROR("height = %d width=%d step=%d encoding=%s", 
		    recent_image->height, 
		    recent_image->width, 
		    recent_image->step,  
		    recent_image->encoding.c_str());
	  ROS_WARN_STREAM("cv_bridge exception: "<<ex.what());
	}
    }
  }
  image_number_++;
}

bool ROSCameraObserver::observationsDone()
{
  if(!new_image_collected_)
  {
    return false;
  }
  return true;
}
cv::Mat ROSCameraObserver::getLastImage()
{
  return(last_raw_image_);
}
} //industrial_extrinsic_cal
