#ifndef POSE_YAML_PARSER_H
#define POSE_YAML_PARSER_H

#include <iostream>
#include <industrial_extrinsic_cal/basic_types.h>
#include "yaml-cpp/yaml.h"
#include <industrial_extrinsic_cal/yaml_utils.h>

namespace industrial_extrinsic_cal
{
  bool parsePose(const YAML::Node& node, Pose6d& pose);
  void writePoseYAML(std::string &file, Pose6d pose);
}  // end industrial_extrinsic_cal namespace

#endif
