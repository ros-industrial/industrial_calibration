/*
    Software License Agreement (Apache License)
    Copyright (c) 2014, Southwest Research Institute
    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#ifndef check_if_points_in_pic_h_
#define check_if_points_in_pic_h_

#include <iostream>
#include "tf/transform_datatypes.h"
#include <industrial_extrinsic_cal/ros_transform_interface.h>
#include <Eigen/Eigen>

/*!
 * \brief The check_if_point_in_pic class
 * this class has many functions that are all used
 * to create a bunch of transforms looking at a target, first.
 * Then it uses filters to check if the camera can see the function
 * and to see if the robot can actually reach those transforms we created.
 */

class check_if_point_in_pic
{
public:
  int image_width;
  int image_height;
  double cx_;
  double cy_;
  double fy;
  double fx;
  int numberOfStopsForPhotos; // controlls amount of stops for each ring
  double poseHeight;
  double angleOfCone;
  double spacing_in_z;
  double center_Of_TargetX;
  double center_Of_TargetY;
  Eigen::Vector2d center_point_of_target;
  tf::StampedTransform tf_transform;
  tf::TransformListener tf_listener;
  double xMax,yMax,xMin,yMin;
  std::string from_frame_param_;
  std::string to_frame_param_;
  Eigen::Vector3d corner_points_[4];

  void create_rviz_target(geometry_msgs::PoseArray& msg);
  void create_transform_listener(tf::StampedTransform& tf_transform, tf::TransformListener& tf_listen);
  geometry_msgs::PoseArray pose_filters(geometry_msgs::PoseArray msg2, tf::StampedTransform tf_transform, int image_width, int image_height, EigenSTL::vector_Affine3d AllcameraPoses,double fx, double fy, double cx, double cy );
  void imagePoint(Eigen::Vector3d TargetPoint, double fx, double fy, double cx, double cy, double &u, double &v,const Eigen::Affine3d &cameraPose);
  geometry_msgs::PoseArray create_all_poses(double poseHeight, double spacing_in_z, double angleOfCone, int numberOfStopsForPhotos, Eigen::Vector2d center_point_of_target );
  int addingFactorial(int lastAdded);
  Eigen::Vector2d finds_middle_of_target(double center_Of_TargetX, double center_Of_TargetY);
  check_if_point_in_pic(ros::NodeHandle pivnh);
private:
};
#endif
