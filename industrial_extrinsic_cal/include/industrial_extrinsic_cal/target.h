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
  /*! \brief constructor*/
  Target(){};

  /*! \brief destructor*/
  ~Target(){};

  /*! \brief sends pose_ to the transform interface*/
  void pushTransform();

  /*! \brief gets pose_ from the transform interface*/
  void pullTransform();

  /*! \brief sets the transform interface */
  void setTransformInterface(boost::shared_ptr<TransformInterface> tranform_interface);

  /*! \brief gets the transform interface, why we have this for a public member, I don't know*/
  boost::shared_ptr<TransformInterface> getTransformInterface();

  /*! \brief sets the transform interface, reference frame*/
  void setTIReferenceFrame(std::string ref_frame);

  /*! \brief generates the points given rows, cols and spacing in correct order*/
  void generatePoints();
  
  // TODO make these each a derived class of a base target class
  union
  {
    CheckerBoardParameters checker_board_parameters_;
    CircleGridParameters circle_grid_parameters_;
    ARTargetParameters ar_target_parameters_;
  };

  // TODO make many of these private with interfaces
  Pose6d pose_;              /**< pose of target */
  unsigned int num_points_;  /**< number of points in the point array */
  std::vector<Point3d> pts_; /**< an array of points expressed relative to Pose p. */
  bool is_moving_;           /**< observed in multiple locations or it fixed to ref frame */
  bool pub_rviz_vis_;        /**< publish an rviz visualization of the target */
  std::string target_name_;  /**< Name of target */
  std::string target_frame_; /**< name of target's coordinate frame */
  unsigned int target_type_; /**< Type of target */
  boost::shared_ptr<TransformInterface> transform_interface_; /**< interface to transform */

private:
};
/*! \brief moving  need a new pose with each scene in which they are used */
typedef struct MovingTarget
{
  boost::shared_ptr<Target> targ_;  // must hold a copy of the target pose parameters,
  int scene_id_;                    // but point parameters may be unused duplicates
} MovingTarget;

}  // end of namespace
#endif
