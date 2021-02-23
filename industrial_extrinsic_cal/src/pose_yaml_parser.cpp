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
  else  if (node["xyz_aaxis_pose"])
  {
    const YAML::Node& node2 = parseNode(node,"xyz_aaxis_pose");
    double x,y,z,ax,ay,az;
    rtn &= parseDouble(node2, "x", x);
    rtn &= parseDouble(node2, "y", y);
    rtn &= parseDouble(node2, "z", z);
    rtn &= parseDouble(node2, "ax", ax);
    rtn &= parseDouble(node2, "ay", ay);
    rtn &= parseDouble(node2, "az", az);
    if(rtn)
      {
	pose.setOrigin(x,y,z);
	pose.setAngleAxis(ax,ay,az);
      }
    else
    {  // can't find xyz_aaxis_pose
      ROS_ERROR("did not read xyz_aaxis_pose correctly");
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
