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
#include <ros/ros.h>
#include <industrial_extrinsic_cal/transform_interface.hpp>
#include "yaml-cpp/yaml.h"
#include <industrial_extrinsic_cal/yaml_utils.h> /* for simple yaml functions */
#include <industrial_extrinsic_cal/pose_yaml_parser.h>

namespace industrial_extrinsic_cal
{
  bool TransformInterface::saveCurrentPose(int scene, std::string& filename)
  {
    std::string full_file_path_name;
    char scene_chars[9];
    sprintf(scene_chars,"_%03d.yaml",scene);
    if(filename == ""){ // build file name from data_directory_, 
      full_file_path_name  = data_directory_ + "/" +  transform_frame_ + std::string(scene_chars);
    }
    else{
      full_file_path_name  = data_directory_ + "/" +  filename;
    }
    writePoseYAML(full_file_path_name, pose_);
    return(true);
  }// end saveCurrentPose()

  bool TransformInterface::loadPose(int scene, std::string& filename)
  {
    std::string full_file_path_name;
    char scene_chars[8];
    sprintf(scene_chars,"_%03d.yaml",scene);
    if(filename == ""){ // build file name from data_directory_, 
      full_file_path_name  = data_directory_ + "/" +  transform_frame_ + std::string(scene_chars);
    }
    else{
      full_file_path_name  = data_directory_ + "/" +  filename;
    }
    
    return(loadPoseYAML(full_file_path_name));
  }// end loadPose()
  bool TransformInterface::loadPoseYAML(std::string &file)
  {
    YAML::Node n;
    if(yamlNodeFromFileName(file, n)){
      return(parsePose(n,pose_));
    }
    return(false);
  }
  Pose6d TransformInterface::getCurrentPose()
  {
    return (pose_);
  }
  void TransformInterface::setCurrentPose(const Pose6d P)
  {
    pose_ = P;
  }
  void TransformInterface::setDataDirectory(std::string dir_name)
  {
    data_directory_ = dir_name;
  }

  std::string TransformInterface::getDataDirectory()
  {
    return(data_directory_);
  }



}// end of namespace industrial_extrinsic_cal
