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

#include <industrial_extrinsic_cal/targets_yaml_parser.h>
#include <industrial_extrinsic_cal/yaml_utils.h>
#include <iostream>
#include <fstream>
#include <industrial_extrinsic_cal/ros_target_display.hpp>

namespace industrial_extrinsic_cal
{
using std::string;
using std::vector;

  class TargetRvizDisplay{
  public:
    // constructor
    TargetRvizDisplay();
    string target_yaml_file_name_;
    vector<boost::shared_ptr<Target> > targets_;
    void publishTargets();
  };
  
  TargetRvizDisplay::TargetRvizDisplay()
  {
    ros::NodeHandle pnh("~");
    
    // get the yaml file name for the target
    if (!pnh.getParam("target_yaml_file", target_yaml_file_name_))
      {
	ROS_ERROR("TargetRvizDisplay, target_yaml_file parameter for this node");
	target_yaml_file_name_ = "target.yaml";
      }
    else{
      ROS_INFO("target file = %s",target_yaml_file_name_.c_str());
    }
    YAML::Node target_doc;
    if (!yamlNodeFromFileName(target_yaml_file_name_, target_doc))
      {
	ROS_ERROR("Can't parse yaml file %s", target_yaml_file_name_.c_str());
      }
    const YAML::Node& target_parameters = parseNode(target_doc, "static_targets");
    if (target_parameters)
      {
	for (unsigned int i = 0; i < target_parameters.size(); i++)
	  {
	    boost::shared_ptr<Target> temp_target = parseSingleTarget(target_parameters[i]);
	    targets_.push_back(temp_target);
	  }
      }
  }// end of constructor TargetRvizDisplay()
  void  industrial_extrinsic_cal::TargetRvizDisplay::publishTargets()
  {
    for(unsigned int i=0; i< targets_.size(); i++){
      displayRvizTarget(targets_[i]);
    }
  }// end of member fuction TargetRvizDisplay::publishTargets()
}// end of namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_visualization_publisher");
  industrial_extrinsic_cal::TargetRvizDisplay TD;
  ros::Rate loop_rate(1);// no need to spin, just keep node running.
  TD.publishTargets();

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
