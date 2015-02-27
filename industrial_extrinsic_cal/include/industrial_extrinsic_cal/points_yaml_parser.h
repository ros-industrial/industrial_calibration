#ifndef POINTS_YAML_PARSER_H
#define POINTS_YAML_PARSER_H

#include <iostream>
#include <industrial_extrinsic_cal/basic_types.h>

namespace industrial_extrinsic_cal {

  void parsePoints(std::ifstream &points_input_file, std::vector<industrial_extrinsic_cal::Point3d> &original_points);
  
}// end industrial_extrinsic_cal namespace

#endif
