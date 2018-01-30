#ifndef POINTS_YAML_PARSER_H
#define POINTS_YAML_PARSER_H

#include <iostream>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/yaml_utils.h>

namespace industrial_extrinsic_cal
{
/** @brief parse a list of points from a file
 *  @param points_input_file, the stream from which to parse the points
 *  @param points the returned vector of points
 *  @return true if success, false otherwise
 */
bool parsePoints(std::string& points_input_file, std::vector<industrial_extrinsic_cal::Point3d>& points);

}  // end industrial_extrinsic_cal namespace

#endif
