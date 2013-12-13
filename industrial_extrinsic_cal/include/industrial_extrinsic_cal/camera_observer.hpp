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

namespace industrial_extrinsic_cal {

  class CameraObserver{ 
  public:
    /** @brief Default destructor */
    virtual ~CameraObserver(){};

    /** @brief add a target to look for */
    /** @param targ a target to look for */
    /** @param roi Region of interest for target */
    virtual void addTarget(boost::shared_ptr<Target> targ, Roi roi)=0;

    /** @brief remove all targets */
    virtual void clearTargets()=0;	

    /** @brief clear all previous observations */
    virtual void clearObservations()=0; 
  
    /** @brief return observations */
    /** @param output all observations of targets defined */
    virtual int getObservations(CameraObservations &camera_observations)=0; 

    /** @brief print this object TODO */
    //    virtual ::std::ostream& operator<<(::std::ostream& os, const CameraObserver& camera);
  };

  class DummyCameraObserver: public CameraObserver{
  public:

    /** @brief constructor */
    /** @param source_name name of image topic */
    DummyCameraObserver(std::string camera_topic="NONE"){}; 

    /** @brief Default destructor */
    ~DummyCameraObserver(){};

    /** @brief add a target to look for */
    /** @param targ a target to look for */
    /** @param roi Region of interest for target */
    void addTarget(boost::shared_ptr<Target> targ, Roi roi){};

    /** @brief remove all targets */
    void clearTargets(){};	

    /** @brief clear all previous observations */
    void clearObservations(){}; 
  
    /** @brief return observations */
    /** @param output all observations of targets defined */
    int getObservations(CameraObservations &camera_observations){return(1);}; 
  };
} // end of namespace
#endif
