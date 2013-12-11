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

#include <industrial_extrinsic_cal/basic_types.h> /** needed for target,Roi, & Observation */
#include <boost/shared_ptr.h>

namespace industrial_extrinsic_cal {

using boost::shared_ptr;
typedef struct { /** Structure returned by Camera Observer */
  std::vector<Observation> obs;
}Camera_Observations;

class CameraObserver{ 
public:
  CameraObserver(std::string source_name); /** @brief constructor */
                                           /** @param source_name name of image topic */
  virtual void addTarget(shared_pter<Target> T, Roi R);/** @brief add a target to look for */
                                            /** @param T a target to look for */
                                            /** @param R Region of interest for target */
  virtual void clearTargets();	/** @brief remove all targets */
  virtual void clearObservations(); /** @brief clear all previous observations */
  virtual int getObservations(Observation &CO); /** @brief perform the observation */
                                                 /** @param output all observations */
  virtual ::std::ostream& operator<<(::std::ostream& os, const CameraObserver& C);
};

} // end of namespace
#endif
