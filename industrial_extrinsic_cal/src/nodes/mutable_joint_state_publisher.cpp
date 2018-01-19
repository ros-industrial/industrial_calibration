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

#include "yaml-cpp/yaml.h"
#include <industrial_extrinsic_cal/mutable_joint_state_publisher.h>
#include <industrial_extrinsic_cal/yaml_utils.h>
#include <iostream>
#include <fstream>
namespace industrial_extrinsic_cal
{
using std::string;
using std::vector;

MutableJointStatePublisher::MutableJointStatePublisher(ros::NodeHandle nh) : nh_(nh)
{
  ros::NodeHandle pnh("~");

  // get the name of the yaml file containing the mutable joints
  if (!pnh.getParam("mutable_joint_state_yaml_file", yaml_file_name_))
  {
    ROS_ERROR("MutableJointStatePublisher, must set mutable_joint_state_yaml_file parameter for this node");
    yaml_file_name_ = "default_mutable_joint_states.yaml";
  }

  // load the joint names and values from the yaml file
  if (!loadFromYamlFile())
  {
    ROS_ERROR("MutableJointStatePublisher constructor can't read yaml file %s", yaml_file_name_.c_str());
  }
  // advertise the services for getting, setting, and storing the mutable joint values
  set_server_ = nh_.advertiseService("set_mutable_joint_states", &MutableJointStatePublisher::setCallBack, this);
  get_server_ = nh_.advertiseService("get_mutable_joint_states", &MutableJointStatePublisher::getCallBack, this);
  store_server_ = nh_.advertiseService("store_mutable_joint_states", &MutableJointStatePublisher::storeCallBack, this);

  // advertise the topic for continious publication of all the mutable joint states
  int queue_size = 10;
  joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("mutable_joint_states", queue_size);
  if (!joint_state_pub_)
  {
    ROS_ERROR("ADVERTISE DID NOT RETURN A VALID PUBLISHER");
  }
}

bool MutableJointStatePublisher::setCallBack(industrial_extrinsic_cal::set_mutable_joint_states::Request& req,
                                             industrial_extrinsic_cal::set_mutable_joint_states::Response& res)
{
  bool return_val = true;
  for (int i = 0; i < (int)req.joint_names.size(); i++)
  {
    if (joints_.find(req.joint_names[i]) != joints_.end())
    {
      joints_[req.joint_names[i]] = req.joint_values[i];
      ROS_INFO("setting %s to %lf", req.joint_names[i].c_str(), req.joint_values[i]);
    }
    else
    {
      ROS_ERROR("%s does not have joint named %s", node_name_.c_str(), req.joint_names[i].c_str());
      return_val = false;
    }
  }

  return (return_val);
}

bool MutableJointStatePublisher::getCallBack(industrial_extrinsic_cal::get_mutable_joint_states::Request& req,
                                             industrial_extrinsic_cal::get_mutable_joint_states::Response& res)
{
  bool return_val = true;
  res.joint_names.clear();
  res.joint_values.clear();
  for (int i = 0; i < (int)req.joint_names.size(); i++)
  {  // for each name asked for
    if (joints_.find(req.joint_names[i]) != joints_.end())
    {                                                           // see if it can be found
      res.joint_names.push_back(req.joint_names[i]);            // use the asked for name in resposne
      res.joint_values.push_back(joints_[req.joint_names[i]]);  // use the value in the response
    }
    else
    {  // can't be found
      ROS_ERROR("%s does not have joint named %s", node_name_.c_str(), req.joint_names[i].c_str());
      return_val = false;
    }
  }
  return (return_val);
}

bool MutableJointStatePublisher::storeCallBack(industrial_extrinsic_cal::store_mutable_joint_states::Request& req,
                                               industrial_extrinsic_cal::store_mutable_joint_states::Response& res)
{
  std::string new_file_name = yaml_file_name_;
  ros::NodeHandle pnh("~");
  bool overwrite = false;
  pnh.getParam("overwrite_mutable_values", overwrite);
  if (!overwrite) new_file_name = yaml_file_name_ + "new";

  std::ofstream fout(new_file_name.c_str());
  YAML::Emitter yaml_emitter;
  yaml_emitter << YAML::BeginMap;
  for (std::map<std::string, double>::iterator it = joints_.begin(); it != joints_.end(); ++it)
  {
    yaml_emitter << YAML::Key << it->first.c_str() << YAML::Value << it->second;
    ROS_INFO("mutable joint %s has value %lf", it->first.c_str(), it->second);
  }
  yaml_emitter << YAML::EndMap;
  fout << yaml_emitter.c_str();
  fout.close();
  return (true);
}

bool MutableJointStatePublisher::loadFromYamlFile()
{
  ROS_INFO_STREAM(yaml_file_name_);
  // yaml file should have the followng format:
  // joint0_name: <float_value0>
  // joint1_name: <float_value1>
  // joint2_name: <float_value2>
  try
  {
    YAML::Node doc;
    if (!yamlNodeFromFileName(yaml_file_name_, doc))
    {
      ROS_ERROR("Can't read yaml file %s", yaml_file_name_.c_str());
    }
    for (YAML_ITERATOR it = doc.begin(); it != doc.end(); ++it)
    {
      std::string key;
      double value;
      parseKeyDValue(it, key, value);
      joints_[key.c_str()] = value;
    }
  }  // end try
  catch (YAML::ParserException& e)
  {
    ROS_ERROR("mutableJointStatePublisher: parsing joint states file");
    ROS_ERROR_STREAM("Failed with exception " << e.what());
    return (false);
  }
  if (joints_.size() == 0) ROS_ERROR("mutable_joint_state_publisher has no joints");

  // output so we know they have been read in correctly
  for (std::map<std::string, double>::iterator it = joints_.begin(); it != joints_.end(); ++it)
  {
    ROS_INFO("mutable joint %s has value %lf", it->first.c_str(), it->second);
  }

  return (true);
}

bool MutableJointStatePublisher::publishJointStates()
{
  sensor_msgs::JointState joint_states;
  joint_states.header.stamp = ros::Time::now();
  joint_states.name.clear();
  joint_states.position.clear();
  joint_states.velocity.clear();
  joint_states.effort.clear();
  for (std::map<string, double>::iterator it = joints_.begin(); it != joints_.end(); ++it)
  {
    joint_states.name.push_back(it->first);
    joint_states.position.push_back(it->second);
    joint_states.velocity.push_back(0.0);
    joint_states.effort.push_back(0.0);
  }
  joint_state_pub_.publish(joint_states);
}
}  // end namespace mutable_joint_state_publisher

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mutable_joint_state_publisher");
  ros::NodeHandle nh;
  industrial_extrinsic_cal::MutableJointStatePublisher MJSP(nh);
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    MJSP.publishJointStates();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
