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
int parseTargetPoints(const Node& node, std::vector<Point3d>& points);

bool parseTargets(std::string& target_file, vector<boost::shared_ptr<Target> >& targets)
{
  bool rtn = true;
  Node target_doc;
  try
  {
    YAML::Node target_doc;
    if (!yamlNodeFromFileName(target_file, target_doc))
    {
      ROS_ERROR("Can't parse yaml file %s", target_file.c_str());
    }
    // read in all static cameras
    targets.clear();
    int n_static = 0;
    const YAML::Node& target_parameters = parseNode(target_doc, "static_targets");
    if (target_parameters)
    {
      for (unsigned int i = 0; i < target_parameters.size(); i++)
      {
        shared_ptr<Target> temp_target = parseSingleTarget(target_parameters[i]);
        temp_target->is_moving_ = false;
        targets.push_back(temp_target);
        n_static++;
      }
    }
    ROS_INFO("parsed = %d static targets",(int) targets.size());
    // read in all moving targets
    int n_moving = 0;
    const YAML::Node& target_parameters2 = parseNode(target_doc, "moving_targets");
    if (target_parameters2)
    {
      for (unsigned int i = 0; i < target_parameters2.size(); i++)
      {
        shared_ptr<Target> temp_target = parseSingleTarget(target_parameters2[i]);
        temp_target->is_moving_ = true;
        targets.push_back(temp_target);
        n_moving++;
      }
    }

    if (targets.size() == 0)
    {
      ROS_ERROR_STREAM("No valid targets found in file " << target_file.c_str());
      return false;
    }
    ROS_INFO_STREAM((int)targets.size() << " targets " << n_static << " static " << n_moving << " moving");
  }
  catch (YAML::ParserException& e)
  {
    ROS_ERROR_STREAM("Failed to parse targets with exception " << e.what());
    rtn = false;
  }
  return (rtn);
}

shared_ptr<Target> parseSingleTarget(const Node& node)
{
  shared_ptr<Target> temp_target = make_shared<Target>();
  shared_ptr<TransformInterface> temp_ti;
  try
  {
    bool success = true;
    success &= parseString(node, "target_name", temp_target->target_name_);
    success &= parseString(node, "target_frame", temp_target->target_frame_);
    success &= parseUInt(node, "target_type", temp_target->target_type_);
    if (!success)
    {
      ROS_ERROR("must set target name, frame and type %s %s %u", temp_target->target_name_.c_str(),
                temp_target->target_frame_.c_str(), temp_target->target_type_);
    }
    Pose6d pose;
    bool transform_available = parsePose(node, pose);
    if (transform_available)
    {
      temp_target->pose_ = pose;
    }

    switch (temp_target->target_type_)
    {
      case pattern_options::Chessboard:
        success &= parseInt(node, "target_rows", temp_target->checker_board_parameters_.pattern_rows);
        success &= parseInt(node, "target_cols", temp_target->checker_board_parameters_.pattern_cols);
	parseDouble(node, "square_size", temp_target->checker_board_parameters_.square_size);
        ROS_DEBUG_STREAM("TargetRows: " << temp_target->checker_board_parameters_.pattern_rows);
	parseBool(node,"publish_vis_marker", temp_target->pub_rviz_vis_);
        if (!success) ROS_ERROR("must set rows and cols");
        break;
      case pattern_options::CircleGrid:
        parseInt(node, "target_rows", temp_target->circle_grid_parameters_.pattern_rows);
        parseInt(node, "target_cols", temp_target->circle_grid_parameters_.pattern_cols);
        parseDouble(node, "circle_dia", temp_target->circle_grid_parameters_.circle_diameter);
	parseDouble(node, "spacing", temp_target->circle_grid_parameters_.spacing);
	parseBool(node,"publish_vis_marker", temp_target->pub_rviz_vis_);
        temp_target->circle_grid_parameters_.is_symmetric = true;
        ROS_DEBUG_STREAM("TargetRows: " << temp_target->circle_grid_parameters_.pattern_rows);
        break;
      case pattern_options::ModifiedCircleGrid:
        parseInt(node, "target_rows", temp_target->circle_grid_parameters_.pattern_rows);
        parseInt(node, "target_cols", temp_target->circle_grid_parameters_.pattern_cols);
        parseDouble(node, "circle_dia", temp_target->circle_grid_parameters_.circle_diameter);
	parseDouble(node, "spacing", temp_target->circle_grid_parameters_.spacing);
	parseBool(node,"publish_vis_marker", temp_target->pub_rviz_vis_);
        temp_target->circle_grid_parameters_.is_symmetric = true;
        ROS_DEBUG_STREAM("TargetRows: " << temp_target->circle_grid_parameters_.pattern_rows);
        break;
      case pattern_options::ARtag:
        // no parameters yet
        break;
      case pattern_options::Balls:
        // no parameters yet
        break;
      default:
        ROS_ERROR("unknown pattern option %d (Chessboard, CircleGrid, or ModifiedCircleGrid, Balls)",
                  (int)temp_target->target_type_);
        break;
    }  // end of target type
    std::string transform_interface;
    if (!parseString(node, "transform_interface", transform_interface))
    {
      ROS_ERROR("must set transform interface for target");
    }
    else
    {
      shared_ptr<TransformInterface> temp_ti =
	parseTransformInterface(node, transform_interface, temp_target->target_frame_, pose);
      temp_target->setTransformInterface(temp_ti);  // install the transform interface
      if (transform_available)
      {
        temp_target->pushTransform();
      }
    }
    temp_target->generatePoints();
    if (parseUInt(node, "num_points", temp_target->num_points_))
    {
      const YAML::Node& points_node = parseNode(node, "points");
      int num_points = parseTargetPoints(points_node, temp_target->pts_);
      if (num_points != temp_target->num_points_ || num_points != (int)temp_target->pts_.size())
	{
	  ROS_ERROR("Expecting %d points found %d", temp_target->num_points_, num_points);
	}
    }
  }  // end try
  catch (YAML::ParserException& e)
  {
    ROS_ERROR_STREAM("Failed to parse single target with exception " << e.what());
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

int parseTargetPoints(const Node& node, std::vector<Point3d>& points)
{
  points.clear();
  for (int i = 0; i < (int)node.size(); i++)
  {
    std::vector<double> temp_pnt;
    parseVectorD(node[i], "pnt", temp_pnt);
    Point3d temp_pnt3d;
    temp_pnt3d.x = temp_pnt[0];
    temp_pnt3d.y = temp_pnt[1];
    temp_pnt3d.z = temp_pnt[2];
    points.push_back(temp_pnt3d);
  }
  return (node.size());
}

}  // end of industrial_extrinsic_cal namespace
