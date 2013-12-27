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

#ifndef ROS_CAMERA_OBSERVER_H_
#define ROS_CAMERA_OBSERVER_H_

#include <industrial_extrinsic_cal/camera_observer.hpp>
#include <industrial_extrinsic_cal/basic_types.h>

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>

/**
 *  @brief enumerator containing three options for the type of pattern to detect
 */
namespace pattern_options
{
enum pattern_options_
{
  Chessboard = 0, CircleGrid = 1, ARtag = 2
};
}
typedef pattern_options::pattern_options_ PatternOption;

namespace industrial_extrinsic_cal
{

class ROSCameraObserver : public CameraObserver
{
public:

  /**
   * @brief constructor
   * @param image_topic name of published image topic
   */
  ROSCameraObserver(std::string &image_topic);

  /**
   * @brief Default destructor
   */
  ~ROSCameraObserver()
  {
  }
  ;

  /**
   * @brief add a target to look for and region to look in
   * @param targ a target to look for
   * @param roi Region of interest for target
   * @return true if successful, false if error in setting target or roi
   */
  bool addTarget(boost::shared_ptr<Target> targ, Roi &roi);

  /**
   * @brief remove all targets
   */
  void clearTargets();

  /**
   * @brief clear all previous observations
   */
  void clearObservations();

  /**
   * @brief return observations
   * @param camera_observations output observations of targets defined
   * @return 0 if failed to get observations, 1 if successful
   */
  int getObservations(CameraObservations &camera_observations);

private:

  PatternOption pattern_;
  /**
   * @brief topic name for image which is input at constructor
   */
  std::string image_topic_;
  /**
   *  @brief cropped image based on original image and region of interest
   */
  cv::Mat image_roi_;
  /**
   *  @brief target pattern grid number of rows
   */
  int pattern_rows_;
  /**
   *  @brief target pattern grid number of columns
   */
  int pattern_cols_;
  /**
   *  @brief circle grid target pattern true=symmetric
   */
  bool sym_circle_;
  /**
   *  @brief 2D values of corner/circle locations returned from cv methods
   */
  std::vector<cv::Point2f> observation_pts_;
  /**
   *  @brief private target which is initialized to input target
   */
  boost::shared_ptr<Target> instance_target_;
  /**
   *  @brief private CameraObservations which are set at the end of getObservations and cleared
   */
  CameraObservations camera_obs_;

  //ROS specific params
  /**
   *  @brief ROS node handle for initializing point cloud publisher
   */
  ros::NodeHandle nh_;
  /**
   *  @brief ROS subscriber to image_topic_
   */
  ros::Subscriber image_sub_;
  /**
   *  @brief ROS publisher of out_bridge_ or output_bridge_
   */
  ros::Publisher results_pub_;

  // Structures for interacting with ROS/CV messages
  /**
   *  @brief cv_bridge image for input image from ROS topic image_topic_
   */
  cv_bridge::CvImagePtr input_bridge_;
  /**
   *  @brief cv_bridge image for cropped color output image
   */
  cv_bridge::CvImagePtr output_bridge_;
  /**
   *  @brief cv_bridge image for cropped mono output image
   */
  cv_bridge::CvImagePtr out_bridge_;

};

} //end industrial_extrinsic_cal namespace

#endif /* ROS_CAMERA_OBSERVER_H_ */
