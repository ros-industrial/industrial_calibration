#include <stdlib.h>
#include <stdio.h>
#include <ostream>
#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <industrial_extrinsic_cal/points_yaml_parser.h>

using std::ifstream;
using std::string;
using std::vector;
using YAML::Node;

namespace industrial_extrinsic_cal
{
bool parsePoints(std::string& points_input_file, vector<Point3d>& points)
{
  // parse points
  bool return_value = true;
  try
  {
    YAML::Node points_doc;
    if (!yamlNodeFromFileName(points_input_file, points_doc))
    {
      ROS_ERROR("Can't parse yaml file %s", points_input_file.c_str());
    }
    // read in all points
    points.clear();
    const YAML::Node& points_node = parseNode(points_doc, "points");
    for (int j = 0; j < points_node.size(); j++)
    {
      vector<double> temp_pnt;
      parseVectorD(points_node[j], "pnt", temp_pnt);
      Point3d temp_pnt3d;
      temp_pnt3d.x = temp_pnt[0];
      temp_pnt3d.y = temp_pnt[1];
      temp_pnt3d.z = temp_pnt[2];
      points.push_back(temp_pnt3d);
    }
  }
  catch (YAML::ParserException& e)
  {
    ROS_INFO_STREAM("Failed to read points file ");
    return_value = false;
  }
  ROS_INFO_STREAM("Successfully read in " << (int)points.size() << " points");
  return (return_value);
}  // end of parse_points()

}  // end of namespace industrial_extrinsic_cal
