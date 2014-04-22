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

namespace industrial_extrinsic_cal
{

  class TransformInterface
  {
  public:
    /** @brief Default destructor */
    ~TransformInterface(){} ;

    /** @brief push the transform to the hardware or display */
    virtual bool pushTransform(Pose6d & pose)=0;

    /** @brief get the transform from the hardware or display */
    virtual Pose6d pullTransform()=0;

    /** @brief output the results to a file */
    virtual bool store(std::string filePath)=0;

    /** @brief get the transform reference frame of transform  */
    virtual void setReferenceFrame(std::string ref_frame) { ref_frame_ = ref_frame;};

    /** @brief set the transform reference frame of transform */
    std::string getReferenceFrame(std::string ref_frame) { return(ref_frame_);};

    /** @brief set the transform reference frame of transform */
    void setTransformFrame(std::string ref_frame) { transform_frame_ = ref_frame;};

    /** @brief set the transform reference frame of transform */
    std::string getTransformFrame(std::string ref_frame) { return(transform_frame_);};

  protected:
    std::string ref_frame_; /*!< name of reference frame for transform (parent frame_id in  Rviz) */
    std::string transform_frame_; /*!< name of frame being defined (frame_id in Rviz) */
    bool ref_frame_initialized_;
  };


  class DefaultTransformInterface: public TransformInterface
  {
  public:

    /** @brief  constructor */
    DefaultTransformInterface(const Pose6d &pose){ pose_ = pose;};

    /** @brief  destructor */
    ~DefaultTransformInterface(){} ;

    /** @brief push the transform to the hardware or display */
    bool pushTransform(Pose6d & pose){ pose_ = pose;};

    /** @brief get the transform from the hardware or display */
    Pose6d pullTransform(){ return(pose_);};

    /** @brief typically outputs the results to a file, but here does nothing */
    bool store(std::string filePath){ return(true);}; 

  protected:
    Pose6d pose_; /*!< 6dof pose  */
  };// end of default tranform interface

} // end of namespace
#endif
