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

  /** @brief saves the latest image to the image_directory_ with the provided filename
   *  if the filename is a null string, the name is built 
   *  from the image_directory_/transform_frame_ + underscore + scene_number.jpg
  */
  /** @param scene, the scene number */
  /** @param filename, the file name */
  bool saveCurrentPose(int scene, std::string& filename);

  /** @brief load pose from the data_directory_ with the provided filename
   *  if the filename is a null string, the name is built 
   *  from the data_directory_/transform_frame_ + underscore + scene_number.yaml
  */
  /** @param scene, the scene number */
  /** @param filename, the file name */
  bool loadPose(int scene, std::string& filename);

  bool loadPoseYAML(std::string &file);
  /**
   *  @brief get current Pose
   *  @return the current Pose
   */
  Pose6d getCurrentPose();

  /**
   *  @brief sets Pose
   *  @param P set the current pose to P
   */
  void setCurrentPose(const Pose6d P);

  /** @brief sets directory for saving and restoring images */
  /** @param dir_name, the directory path */
  void setDataDirectory(std::string dir_name);

  /** @brief gets directory for saving and restoring images */
  /** @param dir_name, the directory path */
  std::string getDataDirectory();


protected:
  Pose6d pose_; /*!< 6dof pose  */
  std::string data_directory_; /*! directory for storing and retrieving transform information for each scene */
  std::string ref_frame_;       /*!< name of reference frame for transform (parent frame_id in  Rviz) */
  std::string transform_frame_; /*!< name of frame being defined (frame_id in Rviz) */
  bool ref_frame_initialized_;  /*!< can't interact with a transform interface until the reference frame is set */

};

}  // end of namespace
#endif
