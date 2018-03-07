#include <stdio.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <industrial_extrinsic_cal/ros_target_display.hpp>

ros::Publisher vis_pub;
visualization_msgs::Marker dots;
visualization_msgs::Marker board;
visualization_msgs::Marker zero_dot;
visualization_msgs::MarkerArray target_geom;

void displayRvizTarget(boost::shared_ptr<industrial_extrinsic_cal::Target> target)
{
  ros::NodeHandle pnh("~");
  vis_pub = pnh.advertise<visualization_msgs::MarkerArray>(target->target_name_.c_str(),0,true);
  double diameter       = target->circle_grid_parameters_.circle_diameter;
  double target_length  = (target->circle_grid_parameters_.pattern_cols+1) *  target->circle_grid_parameters_.spacing;
  double target_width   = (target->circle_grid_parameters_.pattern_rows+1) *  target->circle_grid_parameters_.spacing;
  double target_height  = 0.001;
  board.header.frame_id = target->target_frame_.c_str();
  board.header.stamp = ros::Time();
  board.ns = "my_namespace";
  board.id = 0;
  board.type = visualization_msgs::Marker::CUBE;
  board.action = visualization_msgs::Marker::ADD;
  board.pose.position.x = target_length/2.0 -  target->circle_grid_parameters_.spacing;
  board.pose.position.y = target_width/2.0 -  target->circle_grid_parameters_.spacing;
  board.pose.position.z = 0.0;
  board.pose.orientation.x = 0.0;
  board.pose.orientation.y = 0.0;
  board.pose.orientation.z = 0.0;
  board.pose.orientation.w = 1.0;
  board.scale.x = target_length;
  board.scale.y = target_width;
  board.scale.z = target_height;
  board.color.a = 1.0; // Don't forget to set the alpha!
  board.color.r = 1.0;
  board.color.g = 1.0;
  board.color.b = 1.0;

  dots.header.frame_id = target->target_frame_.c_str();
  dots.header.stamp = ros::Time();
  dots.ns = "my_namespace";
  dots.id = 1;
  dots.type = visualization_msgs::Marker::SPHERE_LIST;
  dots.action = visualization_msgs::Marker::ADD;
  dots.pose.position.x = 0.0;
  dots.pose.position.y = 0.0;
  dots.pose.position.z = 0.002;
  dots.pose.orientation.x = 0.0;
  dots.pose.orientation.y = 0.0;
  dots.pose.orientation.z = 0.0;
  dots.pose.orientation.w = 1.0;
  dots.scale.x = diameter;
  dots.scale.y = diameter;
  dots.scale.z = 0.001;

  std_msgs::ColorRGBA c;
  c.r = 0.01;
  c.g = 0.01;
  c.b = 0.01;
  c.a = 1.0;
  for(int i=0;i<target->num_points_;i++){
    geometry_msgs::Point pt;
    pt.x = target->pts_[i].x;
    pt.y = target->pts_[i].y;
    pt.z = target->pts_[i].z;
    dots.points.push_back(pt);
    dots.colors.push_back(c);
  }

  zero_dot.header.frame_id = target->target_frame_.c_str();
  zero_dot.header.stamp = ros::Time();
  zero_dot.ns = "my_namespace";
  zero_dot.id = 2;
  zero_dot.type = visualization_msgs::Marker::SPHERE;
  zero_dot.action = visualization_msgs::Marker::ADD;
  zero_dot.pose.position.x = 0.00;
  zero_dot.pose.position.y = 0.00;
  zero_dot.pose.position.z = 0.002;
  zero_dot.pose.orientation.x = 0.0;
  zero_dot.pose.orientation.y = 0.0;
  zero_dot.pose.orientation.z = 0.0;
  zero_dot.pose.orientation.w = 1.0;
  zero_dot.scale.x = diameter*1.4;
  zero_dot.scale.y = diameter*1.4;
  zero_dot.scale.z = 0.001;
  zero_dot.color.a = 1.0; // Don't forget to set the alpha!
  zero_dot.color.r = 0.01;
  zero_dot.color.g = 0.01;
  zero_dot.color.b = 0.01;

  target_geom.markers.push_back(board);
  target_geom.markers.push_back(zero_dot);
  target_geom.markers.push_back(dots);

  vis_pub.publish(target_geom);
}
