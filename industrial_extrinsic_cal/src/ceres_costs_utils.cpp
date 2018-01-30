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

#include <industrial_extrinsic_cal/ceres_costs_utils.h>

namespace industrial_extrinsic_cal
{
/*! @brief converts a string to a cost type
 *   @param cost_type_str The cost type string
 *   @returns The cost function type from the enumeration
 */
Cost_function string2CostType(std::string& cost_type_str)
{
  if (cost_type_str == "CameraReprjErrorWithDistortion") return (cost_functions::CameraReprjErrorWithDistortion);
  if (cost_type_str == "CameraReprjErrorWithDistortionPK") return (cost_functions::CameraReprjErrorWithDistortionPK);
  if (cost_type_str == "CameraReprjError") return (cost_functions::CameraReprjError);
  if (cost_type_str == "CameraReprjErrorPK") return (cost_functions::CameraReprjErrorPK);
  if (cost_type_str == "TriangulationErrro") return (cost_functions::TriangulationError);
  if (cost_type_str == "TargetCameraReprjError") return (cost_functions::TargetCameraReprjError);
  if (cost_type_str == "TargetCameraReprjErrorPK") return (cost_functions::TargetCameraReprjErrorPK);
  if (cost_type_str == "LinkTargetCameraReprjError") return (cost_functions::LinkTargetCameraReprjError);
  if (cost_type_str == "LinkTargetCameraReprjErrorPK") return (cost_functions::LinkTargetCameraReprjErrorPK);
  if (cost_type_str == "PosedTargetCameraReprjErrorPK") return (cost_functions::PosedTargetCameraReprjErrorPK);
  if (cost_type_str == "LinkCameraTargetReprjError") return (cost_functions::LinkCameraTargetReprjError);
  if (cost_type_str == "LinkCameraTargetReprjErrorPK") return (cost_functions::LinkCameraTargetReprjErrorPK);
  if (cost_type_str == "CircleCameraReprjErrorWithDistortion")
    return (cost_functions::CircleCameraReprjErrorWithDistortion);
  if (cost_type_str == "CircleCameraReprjErrorWithDistortionPK")
    return (cost_functions::CircleCameraReprjErrorWithDistortionPK);
  if (cost_type_str == "CircleCameraReprjError") return (cost_functions::CircleCameraReprjError);
  if (cost_type_str == "CircleCameraReprjErrorPK") return (cost_functions::CircleCameraReprjErrorPK);
  if (cost_type_str == "CircleTargetCameraReprjErrorWithDistortion")
    return (cost_functions::CircleTargetCameraReprjErrorWithDistortion);
  if (cost_type_str == "CircleTargetCameraReprjErrorWithDistortionPK")
    return (cost_functions::CircleTargetCameraReprjErrorWithDistortionPK);
  if (cost_type_str == "FixedCircleTargetCameraReprjErrorWithDistortionPK")
    return (cost_functions::FixedCircleTargetCameraReprjErrorWithDistortionPK);
  if (cost_type_str == "SimpleCircleTargetCameraReprjErrorWithDistortionPK")
    return (cost_functions::SimpleCircleTargetCameraReprjErrorWithDistortionPK);
  if (cost_type_str == "CircleTargetCameraReprjError") return (cost_functions::CircleTargetCameraReprjError);
  if (cost_type_str == "CircleTargetCameraReprjErrorPK") return (cost_functions::CircleTargetCameraReprjErrorPK);
  if (cost_type_str == "LinkCircleTargetCameraReprjError") return (cost_functions::LinkCircleTargetCameraReprjError);
  if (cost_type_str == "LinkCircleTargetCameraReprjErrorPK")
    return (cost_functions::LinkCircleTargetCameraReprjErrorPK);
  if (cost_type_str == "LinkCameraCircleTargetReprjError") return (cost_functions::LinkCameraCircleTargetReprjError);
  if (cost_type_str == "LinkCameraCircleTargetReprjErrorPK")
    return (cost_functions::LinkCameraCircleTargetReprjErrorPK);
  if (cost_type_str == "FixedCircleTargetCameraReprjErrorPK")
    return (cost_functions::FixedCircleTargetCameraReprjErrorPK);
  return (cost_functions::NullCostType);
}

/*! @brief converts a cost type to a string
 *   @param cost_type The cost type
 *   @returns The cost function type as a string
 */
std::string costType2String(Cost_function cost_type)
{
  if (cost_type == cost_functions::CameraReprjErrorWithDistortion) return ("CameraReprjErrorWithDistortion");
  if (cost_type == cost_functions::CameraReprjErrorWithDistortionPK) return ("CameraReprjErrorWithDistortionPK");
  if (cost_type == cost_functions::CameraReprjError) return ("CameraReprjError");
  if (cost_type == cost_functions::CameraReprjErrorPK) return ("CameraReprjErrorPK");
  if (cost_type == cost_functions::TargetCameraReprjError) return ("TargetCameraReprjError");
  if (cost_type == cost_functions::TargetCameraReprjErrorPK) return ("TargetCameraReprjErrorPK");
  if (cost_type == cost_functions::LinkTargetCameraReprjError) return ("LinkTargetCameraReprjError");
  if (cost_type == cost_functions::LinkTargetCameraReprjErrorPK) return ("LinkTargetCameraReprjErrorPK");
  if (cost_type == cost_functions::PosedTargetCameraReprjErrorPK) return ("PosedTargetCameraReprjErrorPK");
  if (cost_type == cost_functions::LinkCameraTargetReprjError) return ("LinkCameraTargetReprjError");
  if (cost_type == cost_functions::LinkCameraTargetReprjErrorPK) return ("LinkCameraTargetReprjErrorPK");
  if (cost_type == cost_functions::CircleCameraReprjErrorWithDistortion)
    return ("CircleCameraReprjErrorWithDistortion");
  if (cost_type == cost_functions::CircleCameraReprjErrorWithDistortionPK)
    return ("CircleCameraReprjErrorWithDistortionPK");
  if (cost_type == cost_functions::CircleCameraReprjError) return ("CircleCameraReprjError");
  if (cost_type == cost_functions::CircleCameraReprjErrorPK) return ("CircleCameraReprjErrorPK");
  if (cost_type == cost_functions::CircleTargetCameraReprjErrorWithDistortion)
    return ("CircleTargetCameraReprjErrorWithDistortion");
  if (cost_type == cost_functions::CircleTargetCameraReprjErrorWithDistortionPK)
    return ("CircleTargetCameraReprjErrorWithDistortionPK");
  if (cost_type == cost_functions::CircleTargetCameraReprjError) return ("CircleTargetCameraReprjError");
  if (cost_type == cost_functions::CircleTargetCameraReprjErrorPK) return ("CircleTargetCameraReprjErrorPK");
  if (cost_type == cost_functions::LinkCircleTargetCameraReprjError) return ("LinkCircleTargetCameraReprjError");
  if (cost_type == cost_functions::LinkCircleTargetCameraReprjErrorPK) return ("LinkCircleTargetCameraReprjErrorPK");
  if (cost_type == cost_functions::LinkCameraCircleTargetReprjError) return ("LinkCameraCircleTargetReprjError");
  if (cost_type == cost_functions::LinkCameraCircleTargetReprjErrorPK) return ("LinkCameraCircleTargetReprjErrorPK");
  if (cost_type == cost_functions::FixedCircleTargetCameraReprjErrorPK) return ("FixedCircleTargetCameraReprjErrorPK");
  return ("NullCostType");
}
}  // end of namespace
