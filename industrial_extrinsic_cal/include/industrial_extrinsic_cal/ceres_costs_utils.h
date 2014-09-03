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

namespace industrial_extrinsic_cal
{
  // create an enumeration of all the cost functions
  namespace cost_functions{
    enum Cost_function{
      CameraReprjErrorWithDistortion,
      CameraReprjErrorWithDistortionPK,
      CameraReprjError,
      CameraReprjErrorPK,
      TargetCameraReprjError,
      TargetCameraReprjErrorPK,
      LinkTargetCameraReprjError,
      LinkTargetCameraReprjErrorPK,
      LinkCameraTargetReprjError,
      LinkCameraTargetReprjErrorPK,
      CircleCameraReprjErrorWithDistortion,
      CircleCameraReprjErrorWithDistortionPK,
      CircleCameraReprjError,
      CircleCameraReprjErrorPK,
      CircleTargetCameraReprjErrorWithDistortion,
      CircleTargetCameraReprjErrorWithDistortionPK,
      CircleTargetCameraReprjError,
      CircleTargetCameraReprjErrorPK,
      LinkCircleTargetCameraReprjError,
      LinkCircleTargetCameraReprjErrorPK,
      LinkCameraCircleTargetReprjError,
      LinkCameraCircleTargetReprjErrorPK,
      FixedCircleTargetCameraReprjErrorPK
    };
  }// end of namespace cost_functions
  typedef cost_functions::Cost_function Cost_function;

  Cost_function string2CostType(std::string &cost_type_str)
  {
    if(cost_type_str == "CameraReprjErrorWithDistortion") return(cost_functions::CameraReprjErrorWithDistortion);
    if(cost_type_str == "CameraReprjErrorWithDistortionPK") return(cost_functions::CameraReprjErrorWithDistortionPK);
    if(cost_type_str == "CameraReprjError") return(cost_functions::CameraReprjError);
    if(cost_type_str == "CameraReprjErrorPK") return(cost_functions::CameraReprjErrorPK);
    if(cost_type_str == "TargetCameraReprjError") return(cost_functions::TargetCameraReprjError);
    if(cost_type_str == "TargetCameraReprjErrorPK") return(cost_functions::TargetCameraReprjErrorPK);
    if(cost_type_str == "LinkTargetCameraReprjError") return(cost_functions::LinkTargetCameraReprjError);
    if(cost_type_str == "LinkTargetCameraReprjErrorPK") return(cost_functions::LinkTargetCameraReprjErrorPK);
    if(cost_type_str == "LinkCameraTargetReprjError") return(cost_functions::LinkCameraTargetReprjError);
    if(cost_type_str == "LinkCameraTargetReprjErrorPK") return(cost_functions::LinkCameraTargetReprjErrorPK);
    if(cost_type_str == "CircleCameraReprjErrorWithDistortion") return(cost_functions::CircleCameraReprjErrorWithDistortion);
    if(cost_type_str == "CircleCameraReprjErrorWithDistortionPK") return(cost_functions::CircleCameraReprjErrorWithDistortionPK);
    if(cost_type_str == "CircleCameraReprjError") return(cost_functions::CircleCameraReprjError);
    if(cost_type_str == "CircleCameraReprjErrorPK") return(cost_functions::CircleCameraReprjErrorPK);
    if(cost_type_str == "CircleTargetCameraReprjErrorWithDistortion") return(cost_functions::CircleTargetCameraReprjErrorWithDistortion);
    if(cost_type_str == "CircleTargetCameraReprjErrorWithDistortionPK") return(cost_functions::CircleTargetCameraReprjErrorWithDistortionPK);
    if(cost_type_str == "CircleTargetCameraReprjError") return(cost_functions::CircleTargetCameraReprjError);
    if(cost_type_str == "CircleTargetCameraReprjErrorPK") return(cost_functions::CircleTargetCameraReprjErrorPK);
    if(cost_type_str == "LinkCircleTargetCameraReprjError") return(cost_functions::LinkCircleTargetCameraReprjError);
    if(cost_type_str == "LinkCircleTargetCameraReprjErrorPK") return(cost_functions::LinkCircleTargetCameraReprjErrorPK);
    if(cost_type_str == "LinkCameraCircleTargetReprjError") return(cost_functions::LinkCameraCircleTargetReprjError);
    if(cost_type_str == "LinkCameraCircleTargetReprjErrorPK") return(cost_functions::LinkCameraCircleTargetReprjErrorPK);
    if(cost_type_str == "FixedCircleTargetCameraReprjErrorPK") return(cost_functions::FixedCircleTargetCameraReprjErrorPK);
  }
} // end of namespace industrial_extrinsic_cal
#endif
