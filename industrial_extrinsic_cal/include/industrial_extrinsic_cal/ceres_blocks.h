/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
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
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <iostream>
#include <fstream>

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
   *  \param scene_id the scene's id, only one camera of given name exist in each scene
   *  \return true on success
   */
  bool addMovingCamera(boost::shared_ptr<Camera> camera_to_add, int scene_id);

  /*! \brief adds a static target to job's list of static targets
   *  \param target_to_add this is the target added to the list
   *  \param scene_id the scene's id, only one target of given name exist in each scene
   *  \return true on success
   */
  bool addMovingTarget(boost::shared_ptr<Target> target_to_add, int scene_id);

  /*! \brief grabs a camera from the camera list given the camera name
   *  \param camera_name is the name of the camera
   *  \param camera this is the camera from the list, either moving or static
   *  \return true on success
   */
  const boost::shared_ptr<Camera> getCameraByName(const std::string& camera_name);

  /*!
   * \brief grabs a target from the target list given the target name
   * @param target_name is the name of the target
   * @param target is the target from the list, either moving or static
   * @param scene_id, when a target is moving, it has a separate object for each scene
   * @return shared pointer to target
   */
  const boost::shared_ptr<Target> getTargetByName(const std::string& target_name, int scene_id = 0);

  /*! @brief gets a pointer to the intrinsic parameters of a static camera
   *  @param camera_name the camera's name
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

  /*! @brief writes a single launch file with all the static tranforms
   *  @param filepath  the full path to the launch file being created
   */
  bool writeAllStaticTransforms(std::string filepath);

  /*! @brief writes info from static cameras to terminal */
  void displayStaticCameras();

  /*! @brief writes info from moving cameras to terminal */
  void displayMovingCameras();

  /*! @brief writes info from static targets to terminal */
  void displayStaticTargets();

  /*! @brief writes info from moving cameras to terminal */
  void displayMovingTargets();

  /*! @brief writes info from all cameras and targets to terminal */
  void displayAllCamerasAndTargets()
  {
    displayStaticCameras();
    displayMovingCameras();
    displayStaticTargets();
    displayMovingTargets();
  };

  /*! @brief sends transform to the interface*/
  void pushTransforms();

  /*! @brief gets transform from interface
   *    @param scene_id the current scene's id. Pulls from all static cameras, static targets, and those from this scene
   */
  void pullTransforms(int scene_id);

  /*! @brief sets reference transform from interface, and may start a timer for broadcasting*/
  void setReferenceFrame(std::string ref_frame);

  /*! @brief get reference frame name from the interface */
  std::string getReferenceFrame()
  {
    return (reference_frame_);
  };

  std::vector<boost::shared_ptr<Camera> > static_cameras_;       /*!< all non-moving cameras in job */
  std::vector<boost::shared_ptr<MovingCamera> > moving_cameras_; /*! only one camera of a given name per scene */
  std::vector<boost::shared_ptr<Target> > static_targets_;       /*!< all non-moving targets in job */
  std::vector<boost::shared_ptr<MovingTarget> > moving_targets_; /*! only one target of a given name per scene */
  std::string reference_frame_; /*! name of reference frame, typically a ROS tf frame */

};  // end class

// dangling debugging functions
/*! @brief displays a pose from the extrinsic parameters
 *   @param extrinsics an array of parameters representing the 6D pose
 *   @param message a string that describes the pose to be shown
 */
void showPose(P_BLOCK extrinsics, std::string message);
/*! @brief displays a pose
 *   @param pose the 6D pose to be shown
 *   @param message a string that describes the pose to be shown
 */
void showPose(Pose6d pose, std::string message);
/*! @brief displays the intrinsic parameters
 *   @param intrinsics an array of parameters representing the focal length, optical center, and distortion parameters
 *   @param num_param, sometimes there is no distortion, so this is either 4 or 9 depending
 */
void showIntrinsics(P_BLOCK intrinsics, int num_param);

}  // end namespace industrial_extrinsic_cal

#endif /* CERES_BLOCKS_H_ */
