#include <stdlib.h>
#include <stdio.h>
#include <ostream>
#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <industrial_extrinsic_cal/ros_transform_interface.h>
#include <industrial_extrinsic_cal/targets_yaml_parser.h>
#include <industrial_extrinsic_cal/camera_yaml_parser.h>   // for parse_pose() and parse_transform_interface()
#include <industrial_extrinsic_cal/ros_camera_observer.h>  // for pattern options

using std::ifstream;
using std::string;
using std::vector;
using boost::shared_ptr;
using boost::make_shared;
using YAML::Node;

namespace industrial_extrinsic_cal
{
// prototypes
int parse_target_points(const Node& node, std::vector<Point3d> points);

void parse_targets(ifstream& targets_input_file, vector<boost::shared_ptr<Target> >& targets)
{
  try
  {
    YAML::Node targets_node = YAML::LoadFile(targets_input_file.c_str());

    // read in all static cameras
    targets.clear();
    const Node target_parameters = target_node.("static_targets");
    ROS_INFO_STREAM("Found " << target_parameters.size() << " static targets ");
    for (unsigned int i = 0; i < target_parameters.size(); i++)
    {
      shared_ptr<Target> temp_target = parse_single_target(target_parameters[i]);
      targets.push_back(temp_target);
    }

    // read in all moving targets
    target_parameters = target_doc.("moving_targets");
    ROS_INFO_STREAM("Found " << target_parameters.size() << " moving targets ");
    for (unsigned int i = 0; i < target_parameters.size(); i++)
    {
      shared_ptr<Target> temp_target = parse_single_target(target_parameters[i]);
      temp_target->is_moving_ = true;
      targets.push_back(temp_target);
    }
  }
  catch (YAML::ParserException& e)
  {
    ROS_INFO_STREAM("Failed to read points file ");
  }
  ROS_INFO_STREAM("Successfully read in " << (int)targets.size() << " targets");
}  // end of parse_targets

shared_ptr<Target> parse_single_target(const Node& node)
{
  shared_ptr<Target> temp_target = make_shared<Target>();
  shared_ptr<TransformInterface> temp_ti;
  try
  {
    temp_target->target_name_ = node["target_name"].at<std::string>();
    temp_target->target_frame_ = node["target_frame"].at<std::string>();
    temp_target->target_type_ = node["target_type"].at<std::string>();
    switch (temp_target->target_type_)
    {
      case pattern_options::Chessboard:
        temp_target->checker_board_parameters_.pattern_rows = node["target_rows"].at<int>();
        temp_target->checker_board_parameters_.pattern_cols = node["target_cols"].at<int>();
        ROS_DEBUG_STREAM("TargetRows: " << temp_target->checker_board_parameters_.pattern_rows);
        break;
      case pattern_options::CircleGrid:
        temp_target->circle_grid_parameters_.pattern_rows = node["target_rows"].at<int>();
        temp_target->circle_grid_parameters_.pattern_cols = node["target_cols"].at<int>();
        temp_target->circle_grid_parameters_.circle_diameter = ["circle_dia"].at<double>();
        temp_target->circle_grid_parameters_.is_symmetric = true;
        ROS_DEBUG_STREAM("TargetRows: " << temp_target->circle_grid_parameters_.pattern_rows);
        break;
      case pattern_options::ModifiedCircleGrid:
        temp_target->circle_grid_parameters_.pattern_rows = node["target_rows"].at<int>();
        temp_target->circle_grid_parameters_.pattern_cols = node["target_cols"].at<int>();
        temp_target->circle_grid_parameters_.circle_diameter = ["circle_dia"].at<double>();
        temp_target->circle_grid_parameters_.is_symmetric = true;
        ROS_DEBUG_STREAM("TargetRows: " << temp_target->circle_grid_parameters_.pattern_rows);
        break;
      case pattern_options::Balls:
        // no parameters yet
        break;
      default:
        ROS_ERROR_STREAM("unknown pattern option (Chessboard, CircleGrid, or ModifiedCircleGrid, Balls)");
        break;
    }  // end of target type
    temp_target->pose_ = parsePose(node);
    std::string transform_interface;
    transform_interface = node["transform_interface"].at<std::string>();
    shared_ptr<TransformInterface> temp_ti =
        parseTransformInterface(node, transform_interface, temp_target->target_frame_);
    temp_target->setTransformInterface(temp_ti);  // install the transform interface

    temp_target->num_points_ = node["num_points"].at<int>();
    const Node points_node = node("points");
    int num_points = parse_target_points(points_node, temp_target->pts_);
    if (num_points != temp_target->num_points_)
    {
      ROS_ERROR("Expecting %d points found %d", temp_target->num_points_, num_points);
    }
  }  // end try
  catch (YAML::ParserException& e)
  {
    ROS_INFO_STREAM("Failed to read target from yaml file ");
    ROS_INFO_STREAM("target name    = " << temp_target->target_name_.c_str());
    ROS_INFO_STREAM("target type    = " << temp_target->target_type_);
    ROS_INFO_STREAM("target frame    = " << temp_target->target_frame_.c_str());
    ROS_INFO_STREAM("num_points    = " << temp_target->num_points_);
    ROS_INFO_STREAM("angle_axis_ax  = " << temp_target->pose_.ax);
    ROS_INFO_STREAM("angle_axis_ay = " << temp_target->pose_.ay);
    ROS_INFO_STREAM("angle_axis_az  = " << temp_target->pose_.az);
    ROS_INFO_STREAM("position_x     = " << temp_target->pose_.x);
    ROS_INFO_STREAM("position_y     = " << temp_target->pose_.y);
    ROS_INFO_STREAM("position_z     = " << temp_target->pose_.z);
  }
  return (temp_target);
}  // end parse_single_target

int parse_target_points(const Node& node, std::vector<Point3d> points)
{
  ROS_DEBUG_STREAM("FoundPoints: " << node.size());
  points.clear();
  for (int i = 0; i < (int)node.size(); i++)
  {
    const YAML::Node pnt_node = node[i]("pnt");
    std::vector<float> temp_pnt;
    temp_pnt = pnt_node.at<std::vector<double> >();
    Point3d temp_pnt3d;
    temp_pnt3d.x = temp_pnt[0];
    temp_pnt3d.y = temp_pnt[1];
    temp_pnt3d.z = temp_pnt[2];
    points.push_back(temp_pnt3d);
  }
  return (node.size());
}

}  // end of industrial_extrinsic_cal namespace
