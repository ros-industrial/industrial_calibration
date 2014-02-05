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


#include <industrial_extrinsic_cal/camera_definition.h>

namespace industrial_extrinsic_cal
{

using std::string;
using boost::shared_ptr;
//using boost::make_shared;
using ceres::CostFunction;
Camera::Camera()
{
  camera_name_ = "NONE";
  is_moving_ = false;
}

Camera::Camera(string name, CameraParameters camera_parameters, bool is_moving) :
    camera_name_(name), camera_parameters_(camera_parameters), is_moving_(is_moving),
    fixed_intrinsics_(true), fixed_extrinsics_(false)
{
}

Camera::~Camera()
{
}

bool Camera::isMoving()
{
  return (is_moving_);
}

}//end namespace industrial_extrinsic_cal



