/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2015, Southwest Research Institute
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

#ifndef DEPTH_CALIBRATION_H
#define DEPTH_CALIBRATION_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include "pcl_ros/transforms.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <yaml-cpp/yaml.h>
#include <boost/thread/mutex.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <Eigen/StdVector>

#include "ceres/ceres.h"

template <typename T>
void calculateResidualError(T& a, T& b, T& c, T& d, T& dk, T pt[3], T dp[2], T& error);
template <typename T>
inline void calculateResidualError(T& a, T& b, T& c, T& d, T& dk, T pt[3], T dp[2], T& residual)
{
  // depth correction amount
  T dc = dk * exp(dp[0] + dp[1] * pt[2]);

  T z = T(0.0);
  if (pt[2] == 0.0)
  {
    residual = T(0.0);
  }
  else
  {
    // calculated depth using plane equation (ax + by + cz + d = 0)
    z = (-d - a * pt[0] - b * pt[1]) / c;

    // residual = ( (calculated depth) - (measured depth + depth correction) )^2
    residual = abs(z - (pt[2] + dc));
  }

  return;
}

class DepthError
{
public:
  DepthError(double a, double b, double c, double d, double dk, pcl::PointXYZ pt)
    : a_(a), b_(b), c_(c), d_(d), dk_(dk), pt_(pt)
  {
  }

  template <typename T>

  bool operator()(const T* const d_params, /** depth coefficients */
                  T* residual) const
  {
    /**  */
    T a = T(a_);
    T b = T(b_);
    T c = T(c_);
    T d = T(d_);
    T dk = T(dk_);
    T pt[3];
    pt[0] = T(pt_.x);
    pt[1] = T(pt_.y);
    pt[2] = T(pt_.z);

    T depth_params[2];
    depth_params[0] = d_params[0];
    depth_params[1] = d_params[1];

    calculateResidualError(a, b, c, d, dk, pt, depth_params, *residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double a, const double b, const double c, const double d, const double dk,
                                     const pcl::PointXYZ pt)
  {
    return (new ceres::AutoDiffCostFunction<DepthError, 1, 2>(new DepthError(a, b, c, d, dk, pt)));
  }

  double a_;         /** plane equation parameter a */
  double b_;         /** plane equation parameter b */
  double c_;         /** plane equation parameter c */
  double d_;         /** plane equation parameter d */
  double dk_;        /** depth correction value */
  pcl::PointXYZ pt_; /** point cloud (x,y,z) location */
};

class DepthCalibrator
{
public:
  DepthCalibrator(ros::NodeHandle& nh);

  /**
     * @brief If pixel depth calibration has been performed, and points clouds have been stored, performs depth
   * correction calculations
     *
     * Calculates the depth correction coefficients d1 and d2 using the point clouds previously stored.
     * Depth correction for each pixel is of the form D*e^(d1 + d2 * z), where D is the pixel depth error found from the
     * calibrateCameraPixelDepth callback, z is the depth of the given pixel, and d1 and d2 are the depth coefficients
   * to be solved for.
     *
     *
     * @param[in] request Empty
     * @param[out] response Empty
     * @return True if the call succeeded
     */
  bool calibrateCameraDepth(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /**
     * @brief Finds the depth error for each pixel in the point cloud
     *
     * Finds the calibration target and compares each pixel depth value to the expected depth value based upon the
     * pose of the target found.  Stores the resulting depth compensation values as a point cloud for use in the
     * depth correction nodelet.
     *
     * @param[in] request Empty
     * @param[out] response Empty
     * @return True if the call succeeded
     */
  bool calibrateCameraPixelDepth(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /**
     * @brief Sets the flag to store the next point cloud retrieved for later use when performing depth correction
   * calculations
     *
     * @param[in] request Empty
     * @param[out] response Empty
     * @return True if the call succeeded
     */
  bool setStoreCloud(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /**
     * @brief Updates the last_cloud_ data, and if the store_point_cloud_ boolean is set, stores the latest cloud,
   * image, and target pose
     *
     * @param[in] cloud The latest point cloud received
     * @param[in] image The latest RGB image received
     * @return True if the call succeeded
     */
  void updateInputData(const sensor_msgs::PointCloud2ConstPtr& cloud, const sensor_msgs::ImageConstPtr& image);

private:
  const static unsigned int VERSION_NUMBER_; /**< @brief Version number for the calibration package  */

  ros::NodeHandle nh_; /**< @brief ROS node handle */
  bool save_data_;     /**< @brief Flag to determine whether to save calibration results or not */

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> PolicyType;
  typedef message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloudSubscriberType;
  typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriberType;
  typedef message_filters::Synchronizer<PolicyType> SynchronizerType;
  boost::shared_ptr<PointCloudSubscriberType> point_cloud_sub_; /**< @brief Point cloud subscriber */
  boost::shared_ptr<ImageSubscriberType> image_sub_;            /**< @brief RGB image subscriber */
  boost::shared_ptr<SynchronizerType> synchronizer_; /**< @brief Syncronizer for point cloud and image data */

  ros::ServiceClient get_target_pose_;       /**< @brief Service to call target locator to get target pose */
  ros::ServiceServer calibrate_depth_;       /**< @brief Service to compute depth coefficients d1 and d2 */
  ros::ServiceServer calibrate_pixel_depth_; /**< @brief Service to calculate pixel depth error */
  ros::ServiceServer set_store_cloud_;       /**< @brief Service to set and store next point cloud available */

  int num_views_;        /**< @brief Number of times to find the calibration target to get average pose */
  int num_attempts_;     /**< @brief Number of attempts allowed for finding calibration target before failing */
  int num_point_clouds_; /**< @brief Number of point clouds to save and check when performing pixel depth error
                            calculations */
  std::string filename_; /**< @brief Name of the calibration files to save */
  std::string filepath_; /**< @brief Pathway to the location to save calibration files */
  geometry_msgs::Pose target_initial_pose_; /**< @brief The initial pose guess of the calibration target for the target
                                             * finder service
                                               */

  double std_dev_error_;          /**< @brief The standard deviation error allowed for finding the target pose */
  double depth_error_threshold_;  /**< @brief The depth error allowed for calculating the pixel depth error map */
  boost::mutex data_lock_;        /**< @brief Lock for data subscription */
  sensor_msgs::Image last_image_; /**< @brief The last color image received */
  pcl::PointCloud<pcl::PointXYZ> last_cloud_;       /**< @brief The last point cloud received */
  pcl::PointCloud<pcl::PointXYZ> correction_cloud_; /**< @brief The point cloud containing the depth correction values
                                                       */
  std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > saved_clouds_;
  /**< @brief The vector of stored point clouds used to compute the distance coefficients */
  std::vector<std::vector<double> > plane_equations_; /**< @brief A vector of plane equations for each point cloud in
                                                         saved_clouds_ */
  std::vector<cv::Mat> saved_images_; /**< @brief A vector of rgb images for each point cloud in saved_clouds_ */
  std::vector<geometry_msgs::Pose> saved_target_poses_; /**< @brief A vector of target poses for each point cloud in
                                                           saved_clouds_ */

  /**
     * @brief Stores the calibration results in a YAML formated file
     *
     * @param[in] yaml_file The name and pathway of the file to be saved
     * @param[in] dp The depth correction coefficients to be saved
     */
  void storeCalibration(const std::string& yaml_file, const double dp[2]);

  /**
     * @brief Populates and calls the target finder service
     *
     * @param[in] final_cost The desired final cost for the service call
     * @param[out] target_pose The pose of the target returned from the service call
     * @return True if the service call succeeds
     */
  bool findTarget(const double& final_cost, geometry_msgs::Pose& target_pose);

  /**
     * @brief Calls the findTarget function multiple times (num_views) and returns the average plane equation and the
   * last target pose found
     *
     * @param[out] plane_eq The average plane equation results found from averaging the results from all of the target
   * poses found
     * @param[out] target_pose The pose of the target found from the last service call
     * @return True if the target was successfully found before the number of failures (num_attempts) was reached
     */
  bool findAveragePlane(std::vector<double>& plane_eq, geometry_msgs::Pose& target_pose);

  bool findAveragePointCloud(pcl::PointCloud<pcl::PointXYZ>& final_cloud);
};

#endif  // DEPTH_CALIBRATION_H
