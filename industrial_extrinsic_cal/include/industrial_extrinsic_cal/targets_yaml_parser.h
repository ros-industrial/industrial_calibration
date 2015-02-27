#ifndef TARGETS_YAML_PARSER_H
#define TARGETS_YAML_PARSER_H

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <boost/shared_ptr.hpp>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/target.h>

namespace industrial_extrinsic_cal {
  // prototypes
  void parse_targets(std::ifstream &targets_input_file, std::vector<boost::shared_ptr<Target> > & targets);
  boost::shared_ptr<Target> parse_single_target(const YAML::Node &node);
}// end industrial_extrinsic_cal namespace

#endif
