#include <stdlib.h>
#include <stdio.h>
#include <ostream>
#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <industrial_extrinsic_cal/pose_yaml_parser.h>

using std::ifstream;
using std::string;
using std::vector;
using YAML::Node;

namespace industrial_extrinsic_cal
{

bool parsePose(const YAML::Node& node, Pose6d& pose)
{
  bool rtn = true;
  std::vector<double> temp_values;
  if (parseVectorD(node, "xyz_quat_pose", temp_values))
  {
    if (temp_values.size() == 7)
    {
      pose.x = temp_values[0];
      pose.y = temp_values[1];
      pose.z = temp_values[2];
      pose.setQuaternion(temp_values[3], temp_values[4], temp_values[5], temp_values[6]);
    }  // got 7 values
    else
    {  // not 7 values
      ROS_ERROR("did not read xyz_quat_pose correctly, %d values, 7 required", (int)temp_values.size());
      rtn = false;
    }  // right number of values
  }    // "xyz_quat_pose" exists in yaml node
  else if (parseVectorD(node, "xyz_aaxis_pose", temp_values))
  {
    if (temp_values.size() == 6)
    {
      pose.x = temp_values[0];
      pose.y = temp_values[1];
      pose.z = temp_values[2];
      pose.setAngleAxis(temp_values[3], temp_values[4], temp_values[5]);
    }  // got 6 values
    else
    {  // can't find xyz_aaxis_pose
      ROS_ERROR("did not read xyz_aaxis_pose correctly, %d values, 6 required v[0] = %lf", (int)temp_values.size(),
                temp_values[0]);
      rtn = false;
    }
  }
  else
  {  // cant read xyz_aaxis either
    rtn = false;
  }
  return rtn;
}

 void writePoseYAML(std::string &file, Pose6d pose)
 {
   
   std::ofstream fout(file.c_str());
   if(!fout.is_open()){
     ROS_ERROR("writePoseYaml: Could not open %s", file.c_str());
     return;
   }
   YAML::Emitter yaml_emitter;
   yaml_emitter << YAML::BeginMap;
   yaml_emitter << YAML::Key << "xyz_aaxis_pose";
   yaml_emitter << YAML::BeginMap;
   yaml_emitter << YAML::Key << "x"  << YAML::Value << pose.x;
   yaml_emitter << YAML::Key << "y"  << YAML::Value << pose.y;
   yaml_emitter << YAML::Key << "z"  << YAML::Value << pose.z;
   yaml_emitter << YAML::Key << "ax" << YAML::Value << pose.ax;
   yaml_emitter << YAML::Key << "ay" << YAML::Value << pose.ay;
   yaml_emitter << YAML::Key << "az" << YAML::Value << pose.az;
   yaml_emitter << YAML::EndMap;
   yaml_emitter << YAML::EndMap;
   fout << yaml_emitter.c_str();
   fout.close();
 }



}  // end of namespace industrial_extrinsic_cal
