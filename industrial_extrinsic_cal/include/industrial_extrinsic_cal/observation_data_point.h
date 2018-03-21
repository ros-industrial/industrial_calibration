/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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
 * limitations under the License
 */

#ifndef OBSERVATION_DATA_POINT_H_
#define OBSERVATION_DATA_POINT_H_

#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/ceres_costs_utils.h>

namespace industrial_extrinsic_cal
{
/** @brief An observation data point contains all the information necessary to construct the cost function to be added
 *to the
 *              bundle adjustment problem to be solved by ceres. When a camera makes an observation, it fills out this
 *object
 **/
class ObservationDataPoint
{
public:
  /** @brief constructor
   * @param c_name camera name
   * @param t_name target name
   * @param t_type target type
   * @param s_id   scene id
   * @param c_intrinsics  camera's intrinsic parameter block
   * @param c_extrinsics  camera's extrinsic parameter block
   * @param point_id      id of point in target's list of points
   * @param t_pose        targets pose parameter block
   * @param p_position    position of point parameter block
   * @param image_x       image location x
   * @param image_y       image location y
   * @param cost_type     Type of cost function to use
   * @param circle_dia    diameter of circles in the target, if it is a circle target
   */
  ObservationDataPoint(const std::string& c_name, const std::string& t_name, const int& t_type, const int s_id,
                       const P_BLOCK& c_intrinsics, const P_BLOCK& c_extrinsics, const int& point_id,
                       const P_BLOCK& t_pose, const P_BLOCK& p_position, const double& image_x, const double& image_y,
                       const Cost_function cost_type, const Pose6d& intermediate_frame, const double& circle_dia = 0.0)
  {
    camera_name_ = c_name;
    target_name_ = t_name;
    target_type_ = t_type;
    scene_id_ = s_id;
    camera_intrinsics_ = c_intrinsics;
    camera_extrinsics_ = c_extrinsics;
    target_pose_ = t_pose;
    point_id_ = point_id;
    point_position_ = p_position;
    image_x_ = image_x;
    image_y_ = image_y;
    cost_type_ = cost_type;
    circle_dia_ = circle_dia;
    intermediate_frame_ = intermediate_frame;
  };

  /** @brief Destructor */
  ~ObservationDataPoint(){};

  std::string camera_name_;   /**< name of camera */
  std::string target_name_;   /**< name of target */
  unsigned int target_type_;  /**<type of target */
  int scene_id_;              /**< scene's identifier */
  int point_id_;              /**< idetifier of point  */
  P_BLOCK camera_extrinsics_; /**< pointer to block of camera's extrinsic parameters */
  P_BLOCK camera_intrinsics_; /**< pointer to block of camera's interinsic parameters */
  P_BLOCK target_pose_;       /**< pointer to block of target's pose parameters */
  P_BLOCK point_position_;    /**< pointer to block of point's position parameters */
  double image_x_;            /**< location of point in image (observation) */
  double image_y_;            /**< location of point in image (observation) */
  Cost_function cost_type_;   /**< type of cost function */
  double circle_dia_;         /**< diameter of circle being observed (only appies to circular fiducials) */
  Pose6d intermediate_frame_; /**< indentity unless camera was mounted on robot link */
};
// end of class ObservationDataPoint

/**
 * @brief a list of observation data points which allows all the collected information about the observations to be
 * easily submitted to ceres
 *        It also allows the problem data to be printed to files for debugging and external analysis
 */
class ObservationDataPointList
{
public:
  /**
   * @brief constructor
   */
  ObservationDataPointList();

  /**
   * @brief destructor
   */
  ~ObservationDataPointList();

  /** @brief add an observation point to the list
   *   @param new_data_point the observation to add to the list
   */
  void addObservationPoint(ObservationDataPoint new_data_point);

  /** @brief vector of observations */
  std::vector<ObservationDataPoint> items_;
};

}  // end namespace industrial_extrinsic_cal

#endif /* OBSERVATION_DATA_POINT_H_ */
