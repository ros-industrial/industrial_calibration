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

template<typename T> void calculateResidualError(T& a, T& b, T& c, T& d, T& dk,
                                                 T pt[3], T dp[2], T& error);
template<typename T> inline void calculateResidualError(T& a, T& b, T& c, T& d, T& dk,
                                                        T pt[3], T dp[2], T& residual)
{
  // depth correction amount
  T dc = dk * exp(dp[0] + dp[1] * pt[2]);


  T z = T(0.0);
  if(pt[2] == 0.0)
  {
    residual = T(0.0);
  }
  else
  {
    // calculated depth using plane equation (ax + by + cz + d = 0)
    z = (-d - a*pt[0] - b*pt[1]) / c;

    // residual = ( (calculated depth) - (measured depth + depth correction) )^2
    residual = abs( z - (pt[2] + dc) );
  }

  return;
}

class DepthError
{
public:

  DepthError(double a, double b, double c, double d, double dk, pcl::PointXYZ pt) :
    a_(a), b_(b), c_(c), d_(d), dk_(dk), pt_(pt)
  {
  }

  template<typename T>

  bool operator()(const T* const d_params, /** depth polynomial coefficients */
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
  static ceres::CostFunction* Create(const double a, const double b,
             const double c, const double d, const double dk, const pcl::PointXYZ pt)
  {
    return ( new ceres::AutoDiffCostFunction<DepthError, 1, 2>(new DepthError(a, b, c, d, dk, pt)) );
  }

  double a_;  /**  */
  double b_;  /**  */
  double c_;  /**  */
  double d_;  /**  */
  double dk_; /**  */
  pcl::PointXYZ pt_; /**  */

};


class DepthCalibrator
{

public:

  DepthCalibrator(ros::NodeHandle& nh);

  /**
     * @brief
     *
     * @param[in] request Empty
     * @param[out] response Empty
     * @return True if the call succeeded
     */
  bool calibrateCameraDepth(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

  bool calibrateCameraPixelDepth(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

  bool calibrateCameraDistortion(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

  bool setStoreCloud(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

  void updateInputData(const sensor_msgs::PointCloud2ConstPtr& cloud, const sensor_msgs::ImageConstPtr& image);

private:
  const static unsigned int VERSION_NUMBER_;

  ros::NodeHandle nh_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> PolicyType;
  typedef message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloudSubscriberType;
  typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriberType;
  typedef message_filters::Synchronizer<PolicyType> SynchronizerType;
  boost::shared_ptr<PointCloudSubscriberType> point_cloud_sub_;
  boost::shared_ptr<ImageSubscriberType> image_sub_;
  boost::shared_ptr<SynchronizerType> synchronizer_;

  ros::ServiceClient get_target_pose_; /**< @brief  */
  ros::ServiceServer calibrate_depth_; /**< @brief  */
  ros::ServiceServer calibrate_pixel_depth_; /**< @brief  */
  ros::ServiceServer set_store_cloud_; /**< @brief  */
  ros::Subscriber point_cloud_;

  bool store_point_cloud_;
  int num_views_;
  int num_attempts_;
  int num_point_clouds_;
  std::string filename_;
  std::string filepath_;
  geometry_msgs::Pose target_initial_pose_;

  boost::mutex data_lock_; /**< @brief Lock for data subscription */
  pcl::PointCloud<pcl::PointXYZ> last_cloud_;
  pcl::PointCloud<pcl::PointXYZ> correction_cloud_;
  std::vector< pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > saved_clouds_;
  std::vector<std::vector<double> > plane_equations_;
  std::vector<cv::Mat> saved_images_;
  std::vector<geometry_msgs::Pose> saved_target_poses_;

  void storeCalibration(const std::string & yaml_file, const double dp[2]);

  bool findTarget(const double& final_cost, geometry_msgs::Pose& target_pose);

  bool findAveragePlane(std::vector<double> &plane_eq, geometry_msgs::Pose& target_pose);

};

#endif // DEPTH_CALIBRATION_H
