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
  debug_pub_ = nh_.advertise<sensor_msgs::Image>("observer_raw_image", 100);
 
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
  cv::SimpleBlobDetector::Params simple_blob_params;
  bool white_blobs = false;
  pnh.getParam("WhiteBlobs", white_blobs);
  if(white_blobs){
    simple_blob_params.minThreshold = 40;
    simple_blob_params.maxThreshold = 60;
    simple_blob_params.thresholdStep = 5;
    simple_blob_params.minArea = 100;
    simple_blob_params.minConvexity = 0.3;
    simple_blob_params.maxConvexity = 0.3;//circularity ($\frac4*\pi*Area* perimeter$) .
    simple_blob_params.minInertiaRatio = 0.01;
    simple_blob_params.maxInertiaRatio = 0.01;
    simple_blob_params.minArea = 30.0; //float
    simple_blob_params.maxArea = 8000.0;
    simple_blob_params.maxConvexity = 10;
    simple_blob_params.filterByColor = false;
    simple_blob_params.blobColor = (uchar) 128; // 255=light 0=dark blobs
    simple_blob_params.filterByCircularity = true;
    simple_blob_params.minCircularity= 0.8; // float
    simple_blob_params.maxCircularity= 1.0; //float
    simple_blob_params.minDistBetweenBlobs = 10; // float
    simple_blob_params.minRepeatability = (size_t) 128; // don't know what it means
  }
  
  int use_circle_detector=false;
  pnh.getParam("use_circle_detector", use_circle_detector);
  if(use_circle_detector){
    circle_detector_ptr_ = new cv::SimpleBlobDetector();
  }
  else{
    circle_detector_ptr_ = new cv::SimpleBlobDetector(simple_blob_params);
  }

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
    case pattern_options::Balls:
      pattern_ = pattern_options::Balls;
      pattern_rows_ = 1;
      pattern_cols_  = targ->num_points_;
      ROS_ERROR_STREAM("FourBall recognized but pattern not supported yet");
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
  
  cv::Point large_point;
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
      { //contain the scope of automatic variables
	// modified circle grids have one circle at the origin which is 1.5 times larger in diameter than the rest
	ROS_DEBUG_STREAM("Finding Circles in modified symetric grid");
	std::vector<cv::Point2f> centers;
	successful_find = cv::findCirclesGrid(image_roi_, pattern_size, centers, 
					      cv::CALIB_CB_SYMMETRIC_GRID);
					      //	      circle_detector_ptr_);
	if(!successful_find){
	  ROS_ERROR("couldn't find %dx%d modified circle target in %s", pattern_rows_, pattern_cols_,  image_topic_.c_str());
	  out_bridge_->image = image_roi_;
	  debug_pub_.publish(out_bridge_->toImageMsg());
	  return 0;
	}
	// Note, this is the same method called in the beginning of findCirclesGrid, unfortunately, they don't return their keypoints
	// Should OpenCV change their method, the keypoint locations may not match, this has a risk of failing with
	// updates to OpenCV
	std::vector<cv::KeyPoint> keypoints;
	circle_detector_ptr_->detect(image_roi_, keypoints);
	ROS_DEBUG("found %d keypoints", (int) keypoints.size());

	// determine which circle is the largest, 
	if(successful_find){ // determine orientation, and sort points to correct correspondence
	  int start_last_row = pattern_rows_*pattern_cols_ - pattern_cols_; 
	  int end_last_row = pattern_rows_*pattern_cols_ -1; 
	  int start_1st_row = 0;	
	  int end_1st_row = pattern_cols_-1; 
	  
	  double start_last_row_size = -1.0; 
	  double start_1st_row_size = -1.0;
	  double end_1st_row_size = -1.0;
	  double end_last_row_size = -1.0;
	  for(int i=0; i<(int)keypoints.size(); i++){
	    double x = keypoints[i].pt.x;
	    double y = keypoints[i].pt.y;
	    double ksize = keypoints[i].size;
	    if(x == centers[start_last_row].x && y == centers[start_last_row].y) start_last_row_size = ksize;
	    if(x == centers[end_last_row].x && y == centers[end_last_row].y) end_last_row_size = ksize;
	    if(x == centers[start_1st_row].x && y == centers[start_1st_row].y) start_1st_row_size = ksize;
	    if(x == centers[end_1st_row].x && y == centers[end_1st_row].y) end_1st_row_size = ksize;
	  }
	  ROS_DEBUG("start_last_row  %f %f %f", centers[start_last_row].x, centers[start_last_row].y, start_last_row_size);
	  ROS_DEBUG("end_last_row %f %f %f", centers[end_last_row].x, centers[end_last_row].y, end_last_row_size);
	  ROS_DEBUG("start_1st_row %f %f %f", centers[start_1st_row].x, centers[start_1st_row].y, start_1st_row_size);
	  ROS_DEBUG("end_1st_row %f %f %f", centers[end_1st_row].x, centers[end_1st_row].y, end_1st_row_size);
	  if(start_last_row_size <0.0 || start_1st_row_size < 0.0 || end_1st_row_size <0.0 || end_last_row_size < 0.0){
	    ROS_ERROR("No keypoint match for one or more corners");
	    return(false);
	  }
	  
	  // determine if ordering is usual by computing cross product of two vectors normal ordering has z axis positive in cross
	  bool usual_ordering = true; // the most common ordering is with points going from left to right then top to bottom
	  double v1x, v1y, v2x, v2y;
	  v1x = centers[end_last_row].x - centers[start_last_row].x;
	  v1y = -centers[end_last_row].y + centers[start_last_row].y; // reverse because y is positive going down
	  v2x = centers[end_1st_row].x - centers[end_last_row].x;
	  v2y = -centers[end_1st_row].y + centers[end_last_row].y;
	  double cross = v1x*v2y - v1y*v2x;
	  if(cross <0.0){
	    usual_ordering = false;
	  }
	  observation_pts_.clear();

	  // largest circle at start of last row
	  //       ......
	  //       o....
	  if(start_last_row_size >start_1st_row_size && start_last_row_size > end_1st_row_size && start_last_row_size > end_last_row_size){
	    ROS_DEBUG("large circle in start of last row");
	    large_point.x = centers[start_last_row].x;
	    large_point.y = centers[start_last_row].y;
	    if(usual_ordering){ // right side up, no rotation, order is natural, starting from upper left, reads like book
	      for(int i=0; i<(int) centers.size(); i++) observation_pts_.push_back(centers[i]);
	    }
	    else{ // unusual ordering
	      for(int r=0; r<pattern_rows_;r++){
		for(int c=0; c<pattern_cols_; c++){
		  observation_pts_.push_back(centers[r*pattern_cols_ +c]);
		}
	      }
	    } // end unsual ordering
	  }// end largest circle at start
	  // largest circle at end of 1st row
	  //       .....o
	  //       ......
	  else if( end_1st_row_size > end_last_row_size && end_1st_row_size > start_last_row_size && end_1st_row_size > start_1st_row_size){
	    ROS_DEBUG("large at end of 1st row");
	    large_point.x = centers[end_1st_row].x;
	    large_point.y = centers[end_1st_row].y;
	    if(usual_ordering){ // reversed points
	      for(int i=(int) centers.size()-1; i>= 0; i--){
		observation_pts_.push_back(centers[i]);
	      }
	    }
	    else{// unusual ordering
	      for(int c=0; c<pattern_cols_; c++){
		for(int r=0; r<pattern_rows_; r++){
		  observation_pts_.push_back(centers[r*pattern_cols_ +c]);
		}
	      }
	    }// end unusual ordering
	  }// end largest circle at end of 1st row

	  // largest_circle at end of last row
	  //       ......
	  //       ....o
	  else if( end_last_row_size > start_last_row_size && end_last_row_size > end_1st_row_size && end_last_row_size > start_1st_row_size){
	    ROS_DEBUG("large end of last row");
	    large_point.x = centers[end_last_row].x;
	    large_point.y = centers[end_last_row].y;
	    
	    if(usual_ordering){ // 90 80 ... 0, 91 81 ... 1
	      for(int c=0; c<pattern_cols_; c++){
		for(int r=pattern_rows_-1; r>=0; r--){
		  observation_pts_.push_back(centers[r*pattern_cols_ +c]);
		}
	      }
	    }// end normal ordering
	    else{ // unusual ordering 9 8 7 .. 0, 19 18 17 10, 29 28
	      for(int r=0; r<pattern_rows_; r++){ 
		for(int c=pattern_cols_ -1; c>=0; c--){
		  observation_pts_.push_back(centers[r*pattern_cols_ +c]);
		}
	      }
	    }// end unusual ordering
	  }// end large at end of last row
	  
	  // largest circle at start of first row
	  // largest_circle at end of last row
	  //       o.....
	  //       .......
	  else if(start_1st_row_size > end_last_row_size && start_1st_row_size > end_1st_row_size && start_1st_row_size > start_last_row_size){
	    ROS_DEBUG("large at start of 1st row");
	    large_point.x = centers[start_1st_row].x;
	    large_point.y = centers[start_1st_row].y;
	    if(usual_ordering){ // 9 19 29 ... 99, 8 18 ... 98, 
	      for(int c = pattern_cols_ -1; c>=0; c--){
		for(int r=0; r<pattern_rows_; r++){
		  observation_pts_.push_back(centers[r*pattern_cols_ + c]);
		}
	      }
	    }// end normal ordering
	    else{ // unusual ordering  90 91 92 ... 99, 80 81 ... 89
	      for(int r=pattern_rows_-1; r>=0; r--){
		for(int c =0; c<pattern_cols_; c++){
		  observation_pts_.push_back(centers[r*pattern_cols_ + c]);
		}
	      }
	    }
	  } // end large at start of 1st row
	  else
	    {
	      ROS_ERROR("None of the observed corner circles are bigger than all the others");
	      successful_find = false;
	    }
	}// end of successful find
      }
      break;// end modified circle grid case
    case pattern_options::ARtag:
      {
	ROS_ERROR_STREAM("AR Tag recognized but pattern not supported yet");
      }
      break;
    case pattern_options::Balls:
      {// needed to contain scope of automatic variables to this case
	ROS_ERROR_STREAM("FourBall target finder running");
	std::vector<cv::Point2f> centers;
	std::vector<cv::KeyPoint> keypoints;
	circle_detector_ptr_->detect(image_roi_, keypoints);
	observation_pts_.clear();
	if(keypoints.size() == pattern_cols_){
	  ROS_DEBUG("found %d keypoints", pattern_cols_);
	  // sort by size using a dumb method
	  for(int j=0; j<pattern_cols_; j++){
	    float max_size = 0.0;
	    int max_index = 0;
	    for(int i=0;i<(int)keypoints.size();i++){ // for each remaining keypoint
	      if(keypoints[i].size > max_size){ // see if its bigger
		max_size = keypoints[i].size;// save the biggest size
		max_index = i; // save the index
	      }
	    }
	    observation_pts_.push_back(keypoints[max_index].pt);
	    keypoints.erase(keypoints.begin() + max_index);
	  }// end of outer loop
	  large_point.x = observation_pts_[0].x;
	  large_point.y = observation_pts_[0].y;
	}
	else{
	  ROS_ERROR("found %d keypoints but expected only 4", (int) keypoints.size());
	}
      }
      break;
      default:
	ROS_ERROR_STREAM("target_type does not correlate to a known pattern option ");
	return false;
	break;
      }// end of main switch
      
  ROS_DEBUG("Number of keypoints found: %d ", (int)observation_pts_.size());

  // account for shift due to input_roi_
  for(int i=0; i<(int)observation_pts_.size(); i++){
    observation_pts_[i].x += input_roi_.x;
    observation_pts_[i].y += input_roi_.y;
  }

  // draw larger circle at large point
  large_point.x += input_roi_.x;
  large_point.y += input_roi_.y;
  circle(input_bridge_->image, large_point, 3.0, 255, 5);

  // next block of code for publishing the roi as an image, when target is found, circles are placed on image, with a line between pt1 and pt2
  for(int i=0;i<(int)observation_pts_.size();i++){
    cv::Point p;
    p.x = observation_pts_[i].x;
    p.y = observation_pts_[i].y;
    if(i==0){
      circle(input_bridge_->image, p, 2.0, cv::Scalar(0,0,0), 5);
    }
    else{
      circle(input_bridge_->image,p,1.0,255,5);
    }
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

  
  debug_pub_.publish(input_bridge_->toImageMsg());
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
