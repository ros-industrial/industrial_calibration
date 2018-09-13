/*
    Software License Agreement (Apache License)
    Copyright (c) 2014, Southwest Research Institute
    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#ifndef CONICAL_POSE_GENERATOR_H
#define CONICAL_POSE_GENERATOR_H

#include <Eigen/Dense>
#include <eigen_stl_containers/eigen_stl_vector_container.h>

EigenSTL::vector_Affine3d getConicalPoses(const int height_of_cone,
                                          //height of cone
                             const double standoff,
                             //top of cone radius
                             const double radius);

#endif // CONICAL_POSE_GENERATOR_H
