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

#ifndef ROS_CAMERA_OBSERVER_H_
#define ROS_CAMERA_OBSERVER_H_

#include <industrial_extrinsic_cal/camera_observer.hpp>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/ceres_costs_utils.h>
#include <dynamic_reconfigure/server.h>
#include <industrial_extrinsic_cal/circle_grid_finderConfig.h>

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
#include <sensor_msgs/SetCameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <industrial_extrinsic_cal/circle_detector.hpp>

/**
 *  @brief enumerator containing three options for the type of pattern to detect
 */
namespace pattern_options
{
enum pattern_options_
{
  Chessboard = 0,
  CircleGrid = 1,
  ModifiedCircleGrid = 2,
  ARtag = 3,
  Balls = 4,
  SingleBall = 5
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
  ROSCameraObserver(const std::string& image_topic, const std::string& camera_name = "");

  /**
   * @brief Default destructor
   */
  ~ROSCameraObserver(){};

  /**
   * @brief add a target to look for and region to look in
   * @param targ a target to look for
   * @param roi Region of interest for target
   * @param cost_type type of cost function for observations of this target
   * @return true if successful, false if error in setting target or roi
   */
  bool addTarget(boost::shared_ptr<Target> targ, Roi& roi, Cost_function cost_type);

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
  int getObservations(CameraObservations& camera_observations);

  /** @brief tells observer to process next incomming image to find the targets in list */
  void triggerCamera();

  /** @brief tells when camera has completed its observations */
  bool observationsDone();

private:
  /**
   * @brief name of pattern being looked for
   */
  PatternOption pattern_;

  /**
   * @brief topic name for image which is input at constructor
   */
  std::string image_topic_;

  /**
   * @brief topic name for camera_info which is input at constructor
   */
  std::string camera_name_;

  /**
   *  @brief cropped image based on original image and region of interest
   */
  cv::Mat image_roi_;

  /*!
   *  @brief cv rectangle region to crop image into
   */
  cv::Rect input_roi_;

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
   * @brief cost type string
   */
  Cost_function cost_type_;

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

  // ROS specific params
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

  /**
   *  @brief ROS publisher used for publishing images for debugging
   */
  ros::Publisher debug_pub_;

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

  /**
   *  @brief circle_detector_ptr_ is a custom blob detector which localizes circles better than simple blob detection
   */
  cv::Ptr<cv::CircleDetector> circle_detector_ptr_;

  /**
   *  @brief blob_detector_ptr_ is a simple blob detector
   */
  cv::Ptr<cv::FeatureDetector> blob_detector_ptr_;

  /**
   *  @brief new_image_collected, set after the trigger is done
   */
  bool new_image_collected_;

  /**
   *  @brief store_observation_images_ flag to save images for later use
   */
  bool store_observation_images_;

  /**
   *  @brief load_observation_images_ flag to load images from image_directory rather than from subscribed topic
   */
  bool load_observation_images_;

  /**
   *  @brief Flag to indicate if the image used in pattern detection should be normalized.
   */
  bool normalize_calibration_image_;

  /**
   *  @brief image_directory_ place to save images
   */
  std::string image_directory_;

  /**
   *  @brief getImageNumber
   */
  int getImageNumber()
  {
    return (image_number_);
  }
  /**
   *  @brief setImageNumber allows users of the load feature to switch to a different image for the next observation
   */
  void setImageNumber(int image_number)
  {
    image_number_ = image_number;
  }

  /**
   *  @brief get last image
   *  @return the most recent image
   */
  cv::Mat getLastImage();

public:
  /**
   *  @brief push the computed camera parameters out to the camera driver
   *  @param fx the focal length in x
   *  @param fx the focal length in y
   *  @param cx the optical center in x
   *  @param cy the optical center in y
   *  @param k1 the radial distortion 2nd order term
   *  @param k2 the radial distortion 4th order term
   *  @param k3 the radial distortion 6th order term
   *  @param p1 the decentering distortion p1
   *  @param p2 the decentering distortion p2
   */
  bool pushCameraInfo(double& fx, double& fy, double& cx, double& cy, double& k1, double& k2, double& k3, double& p1,
                      double& p2);

  /**
   *  @brief pull the computed camera parameters out to the camera driver
   *  @param fx the focal length in x
   *  @param fx the focal length in y
   *  @param cx the optical center in x
   *  @param cy the optical center in y
   *  @param k1 the radial distortion 2nd order term
   *  @param k2 the radial distortion 4th order term
   *  @param k3 the radial distortion 6th order term
   *  @param p1 the decentering distortion p1
   *  @param p2 the decentering distortion p2
   */
  bool pullCameraInfo(double& fx, double& fy, double& cx, double& cy, double& k1, double& k2, double& k3, double& p1,
                      double& p2);

  /**
   *  @brief pull the computed camera parameters out to the camera driver
   *  @param fx the focal length in x
   *  @param fx the focal length in y
   *  @param cx the optical center in x
   *  @param cy the optical center in y
   *  @param k1 the radial distortion 2nd order term
   *  @param k2 the radial distortion 4th order term
   *  @param k3 the radial distortion 6th order term
   *  @param p1 the decentering distortion p1
   *  @param p2 the decentering distortion p2
   *  @param width the image width
   *  @param height the image height
   */
  bool pullCameraInfo(double& fx, double& fy, double& cx, double& cy, double& k1, double& k2, double& k3, double& p1,
                      double& p2, int& width, int& height);

  void dynReConfCallBack(industrial_extrinsic_cal::circle_grid_finderConfig& config, uint32_t level);

private:
  int image_number_;       /**< a counter of images recieved */
  cv::Mat last_raw_image_; /**< the image last received */
  bool use_circle_detector_;
  bool white_blobs_;
  ros::ServiceClient client_;
  sensor_msgs::SetCameraInfo srv_;
  ros::NodeHandle* rnh_;
  dynamic_reconfigure::Server<industrial_extrinsic_cal::circle_grid_finderConfig>* server_;
};

}  // end industrial_extrinsic_cal namespace

#endif /* ROS_CAMERA_OBSERVER_H_ */
