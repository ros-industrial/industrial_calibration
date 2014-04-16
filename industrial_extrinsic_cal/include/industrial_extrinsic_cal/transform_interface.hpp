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
    virtual ~TransformInterface(){ }  ;

    /** @brief push the transform to the hardware or display */
    virtual bool push_transform(Pose6d & pose)=0;

    /** @brief get the transform from the hardware or display */
    virtual Pose6d pull_transform()=0;

    /** @brief get the transform reference frame of transform  */
    void set_ref_frame(std::string ref_frame) { ref_frame_ = ref_frame;};

    /** @brief set the transform reference frame of transform */
    std::string get_ref_frame(std::string ref_frame) { return(ref_frame_);};

    /** @brief set the transform reference frame of transform */
    void set_transform_frame(std::string ref_frame) { transform_frame_ = ref_frame;};

    /** @brief set the transform reference frame of transform */
    std::string get_transform_frame(std::string ref_frame) { return(transform_frame_);};

  protected:
    std::string ref_frame_; /*!< name of reference frame for transform (parent frame_id in  Rviz) */
    std::string transform_frame_; /*!< name of frame being defined (frame_id in Rviz) */
  };

} // end of namespace
#endif
