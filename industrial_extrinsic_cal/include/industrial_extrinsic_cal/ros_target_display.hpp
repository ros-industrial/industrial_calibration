/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROS_TARGET_DISPLAY_HPP
#define ROS_TARGET_DISPLAY_HPP


#include <industrial_extrinsic_cal/target.h>

  /*@brief display target in rviz
   * @param target target to be displayed
   */
void displayRvizTarget(boost::shared_ptr<industrial_extrinsic_cal::Target> target);
#endif
