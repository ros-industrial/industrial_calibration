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

#include <industrial_extrinsic_cal/ros_transform_interface.h>
namespace industrial_extrinsic_cal
{

  ROSTransformInterface::ROSTransformInterface(std::string transform_frame, std::string ref_frame) 
  {
    ref_frame_           = ref_frame;
    transform_frame_ = transform_frame;

    ros::NodeHandle nh;
    // set up timer so that the object broadcast the current version of the pose to tf every so often
    timer_ = nh.createTimer(ros::Duration(1.0),boost::bind(&ROSTransformInterface::timer_callback,this,_1), this);
  }				
  bool  ROSTransformInterface::push_transform(Pose6d pose)
  {
    pose_.ax = pose.ax;
    pose_.ay = pose.ay;
    pose_.az = pose.az;
    pose_.x  = pose.x;
    pose_.y  = pose.y;
    pose_.z  = pose.z;
  }
  Pose6d  ROSTransformInterface::pull_transform()
  {
    tf::StampedTransform transform;
    ros::Time now = ros::Time::now();
    while(! tf_listener_.waitForTransform(transform_frame_,ref_frame_, now, ros::Duration(1.0))){
      ROS_INFO("waiting for tranform: %s",transform_frame_.c_str());
    }
    tf_listener_.lookupTransform(transform_frame_,ref_frame_, now, transform);
    pose_.set_basis(transform.getBasis());
    pose_.set_origin(transform.getOrigin());
    return(pose_);
  }

  void  ROSTransformInterface::timer_callback(const ros::TimerEvent & timer_event)
  
  { // broadcast current value of pose as a transform each time called
    transform_.setBasis(pose_.get_basis());
    transform_.setOrigin(pose_.get_origin());
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), transform_frame_, ref_frame_));
  }
} // end of namespace industrial_extrinsic_cal

