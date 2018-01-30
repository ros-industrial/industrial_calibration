#ifndef TARGETS_YAML_PARSER_H
#define TARGETS_YAML_PARSER_H

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <boost/shared_ptr.hpp>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/target.h>

namespace industrial_extrinsic_cal
{
// prototypes
/** @brief parse targets from a yaml file
 *    @param target_input_file the stream from which to parse the targets
 *    @param targets vector of shared pointers to targets parsed
 *    @return true if successful
 */
bool parseTargets(std::string& targets_input_file, std::vector<boost::shared_ptr<Target> >& targets);
/** @brief parse a single target
 *    @param node, the yaml node from which to parse the target
 */
boost::shared_ptr<Target> parseSingleTarget(const YAML::Node& node);
}  // end industrial_extrinsic_cal namespace

#endif
