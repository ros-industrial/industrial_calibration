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
 * limitations under the License.
 */

#ifndef TARGET_H_
#define TARGET_H_

#include <industrial_extrinsic_cal/basic_types.h> /* Pose6d,Roi,Observation,CameraObservations */
#include <industrial_extrinsic_cal/transform_interface.hpp> 
namespace industrial_extrinsic_cal
{
  /*! \brief A target's information */
  class Target
  {
  public:
    Target(){};
    ~Target(){};
    void push_transform();
    void pull_transform();
    void set_transform_interface(boost::shared_ptr<TransformInterface> tranform_interface);
    boost::shared_ptr<TransformInterface> get_transform_interface();

    std::string target_name;
    unsigned int target_type;
  
    // TODO make these each a derived class of a basic target
    union
    {
      CheckerBoardParameters checker_board_parameters;
      CircleGridParameters circle_grid_parameters;
      ARTargetParameters ar_target_parameters;
    };
    bool is_moving; /**< observed in multiple locations or it fixed to ref frame */
    Pose6d pose;
    unsigned int num_points; /**< number of points in the point array */
    std::vector<Point3d> pts; /**< an array of points expressed relative to Pose p. */
    bool fixed_pose; /**< is the location of the target known? **/
    bool fixed_points; /**< are the locations of the points within the target known */

  private:
    boost::shared_ptr<TransformInterface>  transform_interface_;
  } ;
  /*! \brief moving  need a new pose with each scene in which they are used */
  typedef struct MovingTarget
  {
    boost::shared_ptr<Target> targ; // must hold a copy of the target pose parameters,
    int scene_id; // but point parameters may be unused duplicates
  } MovingTarget;

}// end of namespace
#endif
