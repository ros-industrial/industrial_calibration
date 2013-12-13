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


#ifndef CAMERA_OBSERVER_HPP_
#define CAMERA_OBSERVER_HPP_

#include <industrial_extrinsic_cal/basic_types.h> /* Target,Roi,Observation,CameraObservations */
#include <boost/shared_ptr.h>

namespace industrial_extrinsic_cal {

  using boost::shared_ptr;

  class CameraObserver{ 
  public:
    /** @brief constructor */
    /** @param source_name name of image topic */
    CameraObserver(std::string source_name); 

    /** @brief Destructor */
    ~CameraObserver();

    /** @brief add a target to look for */
    /** @param targ a target to look for */
    /** @param roi Region of interest for target */
    virtual void addTarget(shared_ptr<Target> targ, Roi roi);

    /** @brief remove all targets */
    virtual void clearTargets();	

    /** @brief clear all previous observations */
    virtual void clearObservations(); 
  
    /** @brief return observations */
    /** @param output all observations of targets defined */
    virtual int getObservations(CameraObservations &camera_observations); 

    /** @brief print this object TODO */
    virtual ::std::ostream& operator<<(::std::ostream& os, const CameraObserver& camera);
  };

} // end of namespace
#endif
