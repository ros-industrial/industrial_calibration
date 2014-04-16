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


namespace industrial_extrinsic_cal
{


class ObservationDataPoint
{
public:

  /**
   *
   * @param c_name camera name
   * @param t_name target name
   * @param s_id   scene id
   * @param c_intrinsics  camera's intrinsic parameter block
   * @param c_extrinsics  camera's extrinsic parameter block
   * @param point_id      id of point in target's list of points
   * @param t_pose        targets pose parameter block
   * @param p_position    position of point parameter block
   * @param image_x       image location x
   * @param image_y       image location y
   */
  ObservationDataPoint(std::string c_name, std::string t_name, int t_type,
		       int s_id, P_BLOCK c_intrinsics, P_BLOCK c_extrinsics,
                       int point_id, P_BLOCK t_pose, P_BLOCK p_position, double image_x, double image_y, double circle_dia=0.0)
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
    circle_dia_ = circle_dia;
  }
  ;

  ~ObservationDataPoint()
  {
  }
  ;

  std::string camera_name_;
  std::string target_name_;
  unsigned int target_type_;
  int scene_id_;
  int point_id_;
  P_BLOCK camera_extrinsics_;
  P_BLOCK camera_intrinsics_;
  P_BLOCK target_pose_;
  P_BLOCK point_position_;
  double image_x_;
  double image_y_;
  double circle_dia_;
};
// end of class ObservationDataPoint

/**
 * @brief a list of observation data points which allows all the collected information about the observations to be easily submitted to ceres
 *        It also allows the problem data to be printed to files for debugging and external analysis
 */
class ObservationDataPointList
{
public:
  ObservationDataPointList();

  ~ObservationDataPointList();

  void addObservationPoint(ObservationDataPoint new_data_point);

  std::vector<ObservationDataPoint> items;
};


}//end namespace industrial_extrinsic_cal

#endif /* OBSERVATION_DATA_POINT_H_ */
