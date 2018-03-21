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

#ifndef CAMERA_CLASS_H_
#define CAMERA_CLASS_H_

#include <ros/console.h>
#include <industrial_extrinsic_cal/camera_observer.hpp>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/trigger.h>
#include <industrial_extrinsic_cal/transform_interface.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include "ceres/ceres.h"

namespace industrial_extrinsic_cal
{
/*! \brief a high level camera wrapper including its parameters, and its observer */
class Camera
{
public:
  /*! \brief default Constructor */
  Camera();

  /*! \brief Constructor
   *  \param name name of camera
   *  \param camera_parameters intrinsic and extrinsic camera parameters
   *  \param is_moving static camera = false, moving camera=true
   */
  Camera(std::string name, CameraParameters camera_parameters, bool is_moving);

  /*! \brief default destructor, nothing to do really */
  ~Camera();

  /*! \brief is the camera's location fixed or not?
   *   moving cameras get multiple pose parameters
   *   static cameras get but one set of pose parameters
   */
  bool isMoving();

  /*! \brief send the camera parameters to the transform interface
   *  Note: some interfaces do nothing on the operation
   */
  void pushTransform();

  /*! \brief get the camera parameters from the transform interface
   *  Note: some interfaces do nothing on this operaton
   */
  void pullTransform();

  /*! \brief set the transform interface
   *  Transform interfaces allow hardware or simulation to provide or accept pose information
   *  depending on whether they are a broadcaster or a listener
   */
  void setTransformInterface(boost::shared_ptr<TransformInterface> tranform_interface);

  /*! \brief set the reference frame for the tranform interface.
   *  Because transform interfaces need to get created when the camera or target is created, we often don't know
   *  what the reference frame is for the interface until later, so we need to set it
   */
  void setTIReferenceFrame(std::string& ref_frame);

  /*! \brief get the observations from the observer
   *  @param camera_observations a vector of observations
     * @return 0 if failed to get observations, 1 if successful
   */
  int getObservations(CameraObservations& camera_observations);

  /*! \brief get the transform interface,
   *  Because transform interfaces need to get created when the camera or target is created, we often don't know
   *  what the reference frame is for the interface until later, so we need to set it
   * TODO since we have a get and set, why is it public?
   */
  boost::shared_ptr<TransformInterface> getTransformInterface();
  boost::shared_ptr<CameraObserver> camera_observer_;         /*!< processes images, does CameraObservations */
  boost::shared_ptr<Trigger> trigger_;                        /*!< pointer to the trigger mechanism for this camera*/
  CameraParameters camera_parameters_;                        /*!< The intrinsic and extrinsic parameters */
  std::string camera_name_;                                   /*!< string camera_name_ unique name of a camera */
  boost::shared_ptr<TransformInterface> transform_interface_; /**< interface to transform, tf for example  */
  Pose6d intermediate_frame_; /**< Sometimes there is an intermediate transform from ref to origin of intrinsics */
  bool is_moving_;            /*!< bool is_moving_  false for static cameras */
};
// end of class Camera

/*! @brief unique observation command */
typedef struct
{
  boost::shared_ptr<Camera> camera;
  boost::shared_ptr<Target> target;
  Roi roi;
  Cost_function cost_type;
} ObservationCmd;

/*! \brief moving cameras need a new pose with each scene in which they are used */
/*! \brief moving cameras need a new pose with each scene in which they are used */
typedef struct MovingCamera
{
  boost::shared_ptr<Camera> cam;  // must hold a copy of the camera extrinsic parameters
  int scene_id;                   // but copy of intrinsics may be and unused duplicate
} MovingCamera;

}  // end namespace industrial_extrinsic_cal

#endif /* CAMERA_CLASS_H_ */
