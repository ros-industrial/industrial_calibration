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

#ifndef ROS_TRANSFORM_INTERFACE_H_
#define ROS_TRANSFORM_INTERFACE_H_

#include <stdio.h>
#include <ros/ros.h> 
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <industrial_extrinsic_cal/transform_interface.hpp>
#include <industrial_extrinsic_cal/basic_types.h>

namespace industrial_extrinsic_cal
{

  class ROSTransformInterface : public TransformInterface
  {
  public:

    /**
     * @brief constructor
     * @param image_topic name of published image topic
     */
    ROSTransformInterface(const std::string transform_frame, const std::string ref_frame);

    /**
     * @brief Default destructor
     */
    ~ROSTransformInterface()  {  } ;

    /** @brief  */
    bool push_transform(Pose6d Pose);

    /** @brief get the transform from the hardware or display */
    Pose6d pull_transform();

    /** @brief a timer callback that continuously broadcast the current pose as a tf */
    void timer_callback(const ros::TimerEvent & timer_event);
   
  private:
    Pose6d pose_;
    ros::Timer timer_;
    tf::TransformListener tf_listener_;
    tf::StampedTransform transform_;
    tf::TransformBroadcaster tf_broadcaster_;
  };

} //end industrial_extrinsic_cal namespace

#endif /* ROS_CAMERA_OBSERVER_H_ */
