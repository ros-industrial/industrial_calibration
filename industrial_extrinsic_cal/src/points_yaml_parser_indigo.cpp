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
void parsePoints(ifstream& points_input_file, vector<Point3d>& points)
{
  // parse points
  try
  {
    YAML::Node points_doc = YAML::LoadFile(points_input_file.c_str());

    // read in all points
    points.clear();
    const YAML::Node points_node = points_doc("points");
    for (int j = 0; j < points_node.size(); j++)
    {
      const YAML::Node pnt_node = points_node[j]("pnt");
      vector<float> temp_pnt;
      temp_pnt = t_node.at<vector<float> >();
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
  }
  ROS_INFO_STREAM("Successfully read in " << (int)points.size() << " points");
}  // end of parse_points()

}  // end of namespace industrial_extrinsic_cal
