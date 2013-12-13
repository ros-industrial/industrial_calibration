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
#ifndef CERES_EXCAL_HPP_
#define CERES_EXCAL_HPP_
namespace industrial_extrinsic_cal {

/*! \brief ceres_excal.h defines structures for ceres
 *   These structures are all unions of two things
 *   The first is a set of meaningful variables (x,y,z,ax,ay,az describe a pose)
 *   The second is a parameter block which overlays all the variables
 *   A pointer to the second block is passed to the templated cost function
 *   which must use these variables to compute the cost, but can't accept arbitrary 
 *   but programmer friendly structures.
 *   In some cases a third parameter block is also supplied which groups multiple blocks
 *   together further reducing the number of parameters pointers sent to a cost function
 */




} \\end of namespace

#endif
