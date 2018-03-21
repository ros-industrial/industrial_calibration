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

#ifndef CERES_COSTS_UTILS_H_
#define CERES_COSTS_UTILS_H_

#include <string>
namespace industrial_extrinsic_cal
{
  // create an enumeration of all the cost functions
  namespace cost_functions{
    enum Cost_function{
      CameraReprjErrorWithDistortion,
      CameraReprjErrorWithDistortionPK,
      CameraReprjError,
      CameraReprjErrorPK,
      TriangulationError,
      TargetCameraReprjError,
      TargetCameraReprjErrorPK,
      LinkTargetCameraReprjError,
      LinkTargetCameraReprjErrorPK,
      PosedTargetCameraReprjErrorPK,
      LinkCameraTargetReprjError,
      LinkCameraTargetReprjErrorPK,
      CircleCameraReprjErrorWithDistortion,
      CircleCameraReprjErrorWithDistortionPK,
      CircleCameraReprjError,
      CircleCameraReprjErrorPK,
      CircleTargetCameraReprjErrorWithDistortion,
      CircleTargetCameraReprjErrorWithDistortionPK,
      FixedCircleTargetCameraReprjErrorWithDistortionPK,
      SimpleCircleTargetCameraReprjErrorWithDistortionPK,
      CircleTargetCameraReprjError,
      CircleTargetCameraReprjErrorPK,
      LinkCircleTargetCameraReprjError,
      LinkCircleTargetCameraReprjErrorPK,
      LinkCameraCircleTargetReprjError,
      LinkCameraCircleTargetReprjErrorPK,
      FixedCircleTargetCameraReprjErrorPK,
      RailICal,
      RailICal3,
      RailSCal,
      StereoTargetLocator,
      DistortedCameraFinder,
      RailICalNoDistortion,
      RangeSensorExtrinsicCal,
      NullCostType
    };
  }// end of namespace cost_functions
  typedef cost_functions::Cost_function Cost_function;

// prototypes of functions

/*! @brief converts a string to a cost type
 *   @param cost_type_str The cost type string
 *   @returns The cost function type from the enumeration
 */
Cost_function string2CostType(std::string& cost_type_str);
/*! @brief converts a cost type to a string
 *   @param cost_type The cost type
 *   @returns The cost function type as a string
 */
std::string costType2String(Cost_function cost_type);

}  // end of namespace industrial_extrinsic_cal
#endif
