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

#ifndef TRANSFORM_INTERFACE_HPP_
#define TRANSFORM_INTERFACE_HPP_

#include <industrial_extrinsic_cal/basic_types.h> /* Pose6d,Roi,Observation,CameraObservations */
#include <ros/ros.h>
namespace industrial_extrinsic_cal
{
class TransformInterface
{
public:
  /** @brief Default destructor */
  virtual ~TransformInterface(){};

  /** @brief push the transform to the hardware or display
   *   @return true if successful, false if not
   */
  virtual bool pushTransform(Pose6d& pose) = 0;

  /** @brief get the transform from the hardware or display */
  virtual Pose6d pullTransform() = 0;

  /** @brief output the results to a file
   *   @param filePath full pathname of file to store resutlts in
   */
  virtual bool store(std::string& filePath) = 0;

  /** @brief get the transform reference frame of transform
   *   @param ref_frame the reference frame name
  */
  virtual void setReferenceFrame(std::string& ref_frame) = 0;

  /** @brief set the transform reference frame of transform */
  std::string getReferenceFrame()
  {
    return (ref_frame_);
  };

  /** @brief set the transform reference frame of transform
   *   @param transform_frame the transform frame name
   */
  void setTransformFrame(std::string& transform_frame)
  {
    transform_frame_ = transform_frame;
  };

  /** @brief set the transform reference frame of transform
   *    @return the transform frame name
   */
  std::string getTransformFrame()
  {
    return (transform_frame_);
  };

  /** @brief get an intermediate pose of a transform by name
   *    @param name of desired transform
   *    @return the pose
   */
  virtual Pose6d getIntermediateFrame()
  {
    Pose6d Identity;  // default constructor is all zero's for translation and anlge axis.
    return (Identity);
  };

  /** @brief  checks to see if the reference frame has been initialized */
  bool isRefFrameInitialized()
  {
    return (ref_frame_initialized_);
  }

protected:
  std::string ref_frame_;       /*!< name of reference frame for transform (parent frame_id in  Rviz) */
  std::string transform_frame_; /*!< name of frame being defined (frame_id in Rviz) */
  bool ref_frame_initialized_;  /*!< can't interact with a transform interface until the reference frame is set */
};

class DefaultTransformInterface : public TransformInterface
{
public:
  /** @brief  constructor
   *   @param pose the pose associated with the transform interface
   */
  DefaultTransformInterface(){};

  /** @brief  destructor */
  ~DefaultTransformInterface(){};

  /** @brief push the transform to the hardware or display
   *   @param pose the pose associated with the transform interface
   */
  bool pushTransform(Pose6d& pose)
  {
    pose_ = pose;
  };

  /** @brief get the transform from the hardware or display
   *    @return the pose
   */
  Pose6d pullTransform()
  {
    return (pose_);
  };

  /** @brief typically outputs the results to a file, but here does nothing
   *    @param the file path name, a dummy argument in this case
   *    @return   always true
   */
  bool store(std::string& filePath)
  {
    return (true);
  };

  /** @brief sets the reference frame of the transform interface, sometimes not used */
  void setReferenceFrame(std::string& ref_frame)
  {
    ref_frame_ = ref_frame;
    ref_frame_initialized_ = true;
  }

protected:
  Pose6d pose_; /*!< 6dof pose  */
};              // end of default tranform interface

}  // end of namespace
#endif
