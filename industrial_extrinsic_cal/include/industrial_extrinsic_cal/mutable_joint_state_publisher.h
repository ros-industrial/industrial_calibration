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

#ifndef MUTABLE_JOINT_STATE_PUBLISHER_H
#define MUTABLE_JOINT_STATE_PUBLISHER_H

#include <stdio.h>
#include <ros/ros.h>
#include <industrial_extrinsic_cal/set_mutable_joint_states.h>
#include <industrial_extrinsic_cal/get_mutable_joint_states.h>
#include <industrial_extrinsic_cal/store_mutable_joint_states.h>
#include <industrial_extrinsic_cal/yaml_utils.h>
#include <sensor_msgs/JointState.h>
namespace industrial_extrinsic_cal
{
/** @brief
 *        This object continuously broadcasts a vector of joint states
 *        It provides 3 services
 *        get the joint values
 *        set the joint values
 *        store the joint names and current values to a yaml file
 *        The intent is to provide a seamless interface for extrinsic calibration of "static" transforms
 *        The static transform publisher does not allow updating its values, typically this involves updating the urdf
 *        Instead, if the urdf defines a chain of x_trans, y_trans, z_trans, roll, pitch and yaw joints
 *        One may then use this node to continuously publish the values of the joints defining the transform
 *        Calibration routines call the service tied to setCallBack() so that path planners use updated pose info
 *        Calibration routines also call the service tied to storeCallBack() so that future launches also use updated
 *        pose info without having to modify their urdf/xacro files.
 */
class MutableJointStatePublisher
{
public:
  /**
   * @brief constructor
   * @param yaml_file_name contains the names of the joints and their default values
   */
  MutableJointStatePublisher(ros::NodeHandle nh);

  /**
   * @brief Default destructor
   */
  ~MutableJointStatePublisher(){};

  /** @brief  set the values of all 6 joints
   *
   */
  bool setCallBack(industrial_extrinsic_cal::set_mutable_joint_states::Request& req,
                   industrial_extrinsic_cal::set_mutable_joint_states::Response& res);

  /** @brief gets the mutable joint state's names and values */
  bool getCallBack(industrial_extrinsic_cal::get_mutable_joint_states::Request& req,
                   industrial_extrinsic_cal::get_mutable_joint_states::Response& res);

  /** @brief writes the mutable joint states to the yaml file*/
  bool storeCallBack(industrial_extrinsic_cal::store_mutable_joint_states::Request& req,
                     industrial_extrinsic_cal::store_mutable_joint_states::Response& res);

  /** @brief publishes the joint states as a joint state message*/
  bool publishJointStates();

private:
  bool loadFromYamlFile();
  std::string yaml_file_name_;
  std::map<std::string, double> joints_;
  ros::NodeHandle nh_;
  ros::Publisher joint_state_pub_;
  ros::ServiceServer get_server_;
  ros::ServiceServer set_server_;
  ros::ServiceServer store_server_;
  std::string node_name_;
};

}  // end industrial_extrinsic_cal namespace
#endif /* ROS_CAMERA_OBSERVER_H_ */
