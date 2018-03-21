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

#ifndef RUNTIME_UTILS_H_
#define RUNTIME_UTILS_H_

#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/calibration_job_definition.h>

//#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <Eigen/Geometry>
#include <Eigen/Core>

namespace industrial_extrinsic_cal
{
class ROSRuntimeUtils
{
public:
  /**
   * @brief constructor
   */
  ROSRuntimeUtils(){};

  /**
   * @brief Default destructor
   */
  ~ROSRuntimeUtils(){

  };

  /**
   * @brief function to convert CalibrationJob P_BLOCK to a pose which can be broadcasted as a transform
   * @param optimized_input
   * @return Tranform to be published/broadcasted
   */
  tf::Transform pblockToPose(industrial_extrinsic_cal::P_BLOCK& optimized_input);
  tf::Transform pblockToPose2(industrial_extrinsic_cal::P_BLOCK& optimized_input);
  /**
   * @brief saved final calibrated transforms as launch file
   * @param package_name directory to package path
   * @param file_name name to save under package path
   * @return true if tf's successfully written to file
   */
  bool store_tf_broadcasters(std::string& package_name, std::string& file_name);
  /**
   * @brief file containing camera definition parameters
   */
  std::string camera_file_;
  /**
   * @brief file containing target definition parameters
   */
  std::string target_file_;
  /**
   *  @brief file containing Calibration Job definition parameters
   */
  std::string caljob_file_;
  /**
   *  @brief name of overall reference frame
   */
  std::string world_frame_;
  /**
   *  @brief name frame for target points
   */
  std::vector<std::string> target_frame_;
  /**
   *  @brief name of camera frame in which observations were made
   */
  std::vector<std::string> camera_optical_frame_;
  /**
   *  @brief name of camera frame which links camera optical frame to reference frame
   */
  std::vector<std::string> camera_intermediate_frame_;

  // CalJob specific
  /**
   *  @brief set of initial extrinsics of camera(s)
   */
  std::vector<industrial_extrinsic_cal::P_BLOCK> initial_extrinsics_;
  /**
   *  @brief set of calibrated extrinsics of camera(s)
   */
  std::vector<industrial_extrinsic_cal::P_BLOCK> calibrated_extrinsics_;
  /**
   *  @brief set of calibrated extrinsics of target(s)
   */
  std::vector<industrial_extrinsic_cal::P_BLOCK> target_poses_;

  // TF specific
  /**
   *  @brief set of initial transform of camera(s)
   */
  std::vector<tf::Transform> initial_transforms_;
  /**
   *  @brief set of resultant calibrated transform of camera(s)
   */
  std::vector<tf::Transform> calibrated_transforms_;
  /**
   *  @brief set of transforms of from camera intermediate to camera optical frames
   */
  std::vector<tf::StampedTransform> camera_internal_transforms_;
  /**
   *  @brief set of transforms of from camera intermediate to camera optical frames
   */
  std::vector<tf::StampedTransform> points_to_world_transforms_;
  /**
   *  @brief set target transforms
   */
  std::vector<tf::Transform> target_transforms_;
  /**
   *  @brief set of broadcasters for transform of camera(s) and target(s)
   */
  std::vector<tf::TransformBroadcaster> broadcasters_;
  /**
   *  @brief set of listeners for transform from camera intermediate to camera optical frames
   */
  tf::TransformListener listener_;
};

}  // end industrial_extrinsic_cal namespace

#endif /* RUNTIME_UTILS_H_ */
