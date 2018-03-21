/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <boost/foreach.hpp>

using std::string;

typedef struct point3d
{
  double x, y, z;
} Point3d;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_locator_service");
  ros::NodeHandle node_handle;
  ros::NodeHandle pnh("~");
  int rows, cols;
  double diameter, spacing;
  std::string target_name;
  std::string target_frame;
  std::string target_file_path;
  std::string transform_interface;
  int target_type;

  if (!pnh.getParam("target_rows", rows))
  {
    ROS_ERROR("Must set param:  target_rows");
  }
  if (!pnh.getParam("target_cols", cols))
  {
    ROS_ERROR("Must set param:  target_cols");
  }
  if (!pnh.getParam("target_circle_dia", diameter))
  {
    ROS_ERROR("Must set param:  target_circle_dia");
  }
  if (!pnh.getParam("target_spacing", spacing))
  {
    ROS_ERROR("Must set param:  target_spacing");
  }
  if (!pnh.getParam("target_type", target_type))
  {
    ROS_ERROR("Must set param:  target_type");
  }
  if (!pnh.getParam("target_name", target_name))
  {
    ROS_ERROR("Must set param:  target_name");
  }
  if (!pnh.getParam("target_frame", target_frame))
  {
    ROS_ERROR("Must set param:  target_frame");
  }
  if (!pnh.getParam("target_file_path", target_file_path))
  {
    ROS_ERROR("Must set param:  target_file_path");
  }
  if (!pnh.getParam("transform_interface", transform_interface))
  {
    ROS_ERROR("Must set param:  transform_interface");
  }
  // generate points
  std::vector<Point3d> pts;
  pts.clear();
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      Point3d point;
      point.x = j * spacing;
      point.y = (rows - 1 - i) * spacing;
      point.z = 0.0;
      pts.push_back(point);
    }
  }

  // write target
  FILE* fp;
  if ((fp = fopen(target_file_path.c_str(), "w")) != NULL)
  {
    fprintf(fp, "---\n\n");
    fprintf(fp, "static_targets:\n");
    fprintf(fp, "-\n");
    fprintf(fp, "     target_name: %s\n", target_name.c_str());
    fprintf(fp, "     target_type: %d\n", target_type);
    fprintf(fp, "     circle_dia: %f\n", diameter);
    fprintf(fp, "     target_frame: %s\n", target_frame.c_str());
    fprintf(fp, "     transform_interface: %s\n", transform_interface.c_str());
    fprintf(fp, "     xyz_aaxis_pose: [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n");
    fprintf(fp, "     target_rows: %d\n", rows);
    fprintf(fp, "     target_cols: %d\n", cols);
    fprintf(fp, "     num_points: %d\n", (int)pts.size());
    fprintf(fp, "     points: \n");
    BOOST_FOREACH (Point3d pt, pts)
    {
      fprintf(fp, "     - pnt: [ %7.4lf,  %7.4lf,  %7.4lf]\n", pt.x, pt.y, pt.z);
    }
    fclose(fp);
  }
  else
  {
    ROS_ERROR("Could not open target file %s", target_file_path.c_str());
  }
  return 1;
}
