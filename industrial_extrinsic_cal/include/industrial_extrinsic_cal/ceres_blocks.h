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

#ifndef CERES_BLOCKS_H_
#define CERES_BLOCKS_H_

#include <ros/console.h>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/camera_definition.h>
#include "boost/make_shared.hpp"

namespace industrial_extrinsic_cal
{

/** \brief These blocks of data hold the ceres parameters upon which the optimizaition proceeds
 *   Static cameras have a block of parameters for their 6Dof Pose
 *                  they have a block of 4 parameters for pinhole projection model intrinsics
 *                  they have a block of 10 parameters for their distortion projection intrinsics
 *                  The 1st 4 parameters of the 10 distortion model are the same variables
 *   Moving cameras have identical sets of */
class CeresBlocks
{
public:
  /** \brief Constructor */
  CeresBlocks();
  /** \brief Destructor */
  ~CeresBlocks();

  /*! \brief clear all static and moving cameras and targets */
  void clearCamerasTargets();

  /*! \brief adds a static camera to job's list of cameras
   *  \param camera_to_add this is the camera added to the list
   *  \return true on success
   */
  bool addStaticCamera(boost::shared_ptr<Camera> camera_to_add);

  /*! \brief adds a static target to job's list of static targets
   *  \param target_to_add this is the target added to the list
   *  \return true on success
   */
  bool addStaticTarget(boost::shared_ptr<Target> target_to_add);

  /*! \brief adds a moving camera to job's list of cameras
   *  \param camera_to_add this is the camera added to the list
   *  \param scene_id the scene's id, only one camera of given name existe in each scene
   *  \return true on success
   */
  bool addMovingCamera(boost::shared_ptr<Camera> camera_to_add, int scene_id);

  /*! \brief adds a static target to job's list of static targets
   *  \param target_to_add this is the target added to the list
   *  \param scene_id the scene's id, only one target of given name exist in each scene
   *  \return true on success
   */
  bool addMovingTarget(boost::shared_ptr<Target> target_to_add, int scene_id);

  /*! @brief gets a pointer to the intrinsic parameters of a static camera
   *  @param camera_name the camera's name
   *  @return pointer to the only existing set of intrinsics for this camera
   */
  P_BLOCK getStaticCameraParameterBlockIntrinsics(std::string camera_name);

  /*! @brief gets a pointer to the intrinsic parameters of a moving camera
   *  @param camera_name the camera's name
   *  @return pointer to the intrinsic parameters for this camera, note that
   *          a number of copies of the intrinsics exist, because the camera
   *          was duplicated in each scene it made an observation
   *          however, the first scene in which the camera was used contains the
   *          only set of intrinsics to be adjusted, and therefore is the only one returned
   */
  P_BLOCK getMovingCameraParameterBlockIntrinsics(std::string camera_name);

  /*! @brief gets a pointer to the extrinsics parameters of a static camera
   *  @param camera_name the camera's name
   *  @return pointer to the only existing set of extrinsics for this camera
   */
  P_BLOCK getStaticCameraParameterBlockExtrinsics(std::string camera_name);

  /*! @brief gets a pointer to the extrisic parameters of a moving camera
   *  @param camera_name the camera's name
   *  @return pointer to the intrinsic parameters for this camera, note that
   *          a number of copies of the intrinsics exist, because the camera
   *          was duplicated in each scene it made an observation.
   *          Therefore, the extrinsic parameters must be the ones from
   *          the correct scene.
   */
  P_BLOCK getMovingCameraParameterBlockExtrinsics(std::string camera_name, int scene_id);

  /*! @brief gets a pointer to the pose parameters of a static target
   *  @param target_name the target's name
   *  @return pointer to the only existing set of pose parameters for this target
   */
  P_BLOCK getStaticTargetPoseParameterBlock(std::string target_name);

  /*! @brief gets a pointer to the position parameters of a static target's point
   *  @param target_name the target's name
   *  @param point_id id of point on target
   *  @return pointer to the only existing set of position parameters for this point
   */
  P_BLOCK getStaticTargetPointParameterBlock(std::string target_name, int point_id);

  /*! @brief gets a pointer to the targets pose parameters
   *  @param target_name moving target's name
   *  @param point_id id of point on target
   *  @param scene_id id of scene where target was imaged
   *  @return pointer to the pose parameters of the target
   *          a number of copies of the pose parameters might exist for a moving target
   *          because the target was duplicated in each scene it was observed.
   *          Therefore, the desired set is the only copy with the given scene_id
   */
  P_BLOCK getMovingTargetPoseParameterBlock(std::string target_name, int scene_id);

  /*! @brief gets a pointer to the point's position parameters
   *  @param target_name moving target's name
   *  @param point_id id of point on target
   *  @return pointer to the position parameters of the point
   *          a number of copies of the point parameters might exist
   *          because the target was duplicated in each scene it was observed.
   *          However, the coordinates of the point in the target frame should not change
   *          Therefore, only the first occurance (first scene) is used
   */
  P_BLOCK getMovingTargetPointParameterBlock(std::string target_name, int pnt_id);

  //private:
  std::vector<boost::shared_ptr<Camera> > static_cameras_; /*!< all non-moving cameras in job */
  std::vector<boost::shared_ptr<MovingCamera> > moving_cameras_; /*! only one camera of a given name per scene */
  std::vector<boost::shared_ptr<Target> > static_targets_; /*!< all non-moving targets in job */
  std::vector<boost::shared_ptr<MovingTarget> > moving_targets_; /*! only one target of a given name per scene */
};//end class

}// end namespace industrial_extrinsic_cal

#endif /* CERES_BLOCKS_H_ */
