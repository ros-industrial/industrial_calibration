/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <target_finder/target_locator.h>
#include <tf/transform_broadcaster.h>

using std::string;
using std::vector;
// This node locates two cameras relative to a single target and publishes their transforms
class callService
{
public:
  callService(ros::NodeHandle nh) : nh_(nh)
  {
    ros::NodeHandle pnh("~");
    std::string c1_cs, c2_cs;  // the call service names for each camera
    if (!pnh.getParam("camera1_target_locate_service", c1_cs))
    {
      ROS_ERROR("must define camera1_target_locate_service parameter");
      exit(1);
    }
    if (!pnh.getParam("camera2_target_locate_service", c2_cs))
    {
      ROS_ERROR("must define camera1_target_locate_service parameter");
      exit(1);
    }
    ROS_INFO("C1 service = %s", c1_cs.c_str());
    ROS_INFO("C2 service = %s", c2_cs.c_str());
    c1_client_ = nh_.serviceClient<target_finder::target_locator>(c1_cs.c_str());
    c2_client_ = nh_.serviceClient<target_finder::target_locator>(c2_cs.c_str());

    if (!pnh.getParam("c1_roi_width", c1_roi_width_))
    {
      c1_roi_width_ = 1280;
    }
    if (!pnh.getParam("c2_roi_width", c2_roi_width_))
    {
      c2_roi_width_ = 1280;
    }
    if (!pnh.getParam("c1_roi_height", c1_roi_height_))
    {
      c1_roi_height_ = 1024;
    }
    if (!pnh.getParam("c2_roi_height", c2_roi_height_))
    {
      c2_roi_height_ = 1024;
    }
    if (!pnh.getParam("c1_optical_frame", c1_optical_frame_))
    {
      c1_optical_frame_ = "basler1_optical_frame";
    }
    if (!pnh.getParam("c2_optical_frame", c2_optical_frame_))
    {
      c2_optical_frame_ = "basler2_optical_frame";
    }
    if (!pnh.getParam("c1_target_frame", c1_target_frame_))
    {
      c1_target_frame_ = "target_frame";
    }
    if (!pnh.getParam("c2_target_frame", c2_target_frame_))
    {
      c2_target_frame_ = "c2_target_frame";
    }
    setRequest();
  }
  bool callTheService();
  void copyResponseToRequest();
  void setRequest();
  bool resetC2();

private:
  ros::NodeHandle nh_;
  ros::ServiceClient c1_client_, c2_client_;
  target_finder::target_locator c1_srv_, c2_srv_;
  tf::TransformBroadcaster tf_broadcaster_;
  int c1_roi_width_, c2_roi_width_;
  int c1_roi_height_, c2_roi_height_;
  std::string c1_optical_frame_, c2_optical_frame_;
  std::string c1_target_frame_, c2_target_frame_;
  tf::Transform camera1_to_camera2_;
};

void callService::copyResponseToRequest()
{
  setRequest();
  c1_srv_.request.initial_pose.position.x = c1_srv_.response.final_pose.position.x;
  c1_srv_.request.initial_pose.position.y = c1_srv_.response.final_pose.position.y;
  c1_srv_.request.initial_pose.position.z = c1_srv_.response.final_pose.position.z;
  c1_srv_.request.initial_pose.orientation.x = c1_srv_.response.final_pose.orientation.x;
  c1_srv_.request.initial_pose.orientation.y = c1_srv_.response.final_pose.orientation.y;
  c1_srv_.request.initial_pose.orientation.z = c1_srv_.response.final_pose.orientation.z;
  c1_srv_.request.initial_pose.orientation.w = c1_srv_.response.final_pose.orientation.w;

  c2_srv_.request.initial_pose.position.x = c2_srv_.response.final_pose.position.x;
  c2_srv_.request.initial_pose.position.y = c2_srv_.response.final_pose.position.y;
  c2_srv_.request.initial_pose.position.z = c2_srv_.response.final_pose.position.z;
  c2_srv_.request.initial_pose.orientation.x = c2_srv_.response.final_pose.orientation.x;
  c2_srv_.request.initial_pose.orientation.y = c2_srv_.response.final_pose.orientation.y;
  c2_srv_.request.initial_pose.orientation.z = c2_srv_.response.final_pose.orientation.z;
  c2_srv_.request.initial_pose.orientation.w = c2_srv_.response.final_pose.orientation.w;
}
bool callService::callTheService()
{
  bool c1_target_found = false;
  bool c2_target_found = false;

  if (c1_client_.call(c1_srv_))
  {
    double x, y, z, qx, qy, qz, qw;
    x = c1_srv_.response.final_pose.position.x;
    y = c1_srv_.response.final_pose.position.y;
    z = c1_srv_.response.final_pose.position.z;
    qx = c1_srv_.response.final_pose.orientation.x;
    qy = c1_srv_.response.final_pose.orientation.y;
    qz = c1_srv_.response.final_pose.orientation.z;
    qw = c1_srv_.response.final_pose.orientation.w;
    ROS_INFO("Camera1 Pose: tx= %5.4lf  %5.4lf  %5.4lf quat= %5.3lf  %5.3lf  %5.3lf %5.3lf, cost= %5.3lf",
             c1_srv_.response.final_pose.position.x, c1_srv_.response.final_pose.position.y,
             c1_srv_.response.final_pose.position.z, c1_srv_.response.final_pose.orientation.x,
             c1_srv_.response.final_pose.orientation.y, c1_srv_.response.final_pose.orientation.z,
             c1_srv_.response.final_pose.orientation.w, c1_srv_.response.final_cost_per_observation);
    tf::Transform camera_to_target;
    tf::Quaternion quat(qx, qy, qz, qw);
    camera_to_target.setOrigin(tf::Vector3(x, y, z));
    camera_to_target.setRotation(quat);
    tf::StampedTransform stf(camera_to_target.inverse(), ros::Time::now(), c1_target_frame_.c_str(),
                             c1_optical_frame_.c_str());
    tf_broadcaster_.sendTransform(stf);
    c1_target_found = 1;
  }
  if (c2_client_.call(c2_srv_))
  {
    double x, y, z, qx, qy, qz, qw;
    x = c2_srv_.response.final_pose.position.x;
    y = c2_srv_.response.final_pose.position.y;
    z = c2_srv_.response.final_pose.position.z;
    qx = c2_srv_.response.final_pose.orientation.x;
    qy = c2_srv_.response.final_pose.orientation.y;
    qz = c2_srv_.response.final_pose.orientation.z;
    qw = c2_srv_.response.final_pose.orientation.w;
    ROS_INFO("Camera1 Pose: tx= %5.4lf  %5.4lf  %5.4lf quat= %5.3lf  %5.3lf  %5.3lf %5.3lf, cost= %5.3lf",
             c2_srv_.response.final_pose.position.x, c2_srv_.response.final_pose.position.y,
             c2_srv_.response.final_pose.position.z, c2_srv_.response.final_pose.orientation.x,
             c2_srv_.response.final_pose.orientation.y, c2_srv_.response.final_pose.orientation.z,
             c2_srv_.response.final_pose.orientation.w, c2_srv_.response.final_cost_per_observation);
    tf::Transform camera_to_target;
    tf::Quaternion quat(qx, qy, qz, qw);
    camera_to_target.setOrigin(tf::Vector3(x, y, z));
    camera_to_target.setRotation(quat);
    tf::StampedTransform stf(camera_to_target, ros::Time::now(), c2_optical_frame_.c_str(), c2_target_frame_.c_str());
    tf_broadcaster_.sendTransform(stf);
    c2_target_found = true;
  }

  // resend camera1 to camera2 tf
  ROS_INFO("publishing %s to %s %f %f %f", c1_optical_frame_.c_str(), c2_optical_frame_.c_str(),
           camera1_to_camera2_.getOrigin().x(), camera1_to_camera2_.getOrigin().y(),
           camera1_to_camera2_.getOrigin().z());

  tf::StampedTransform stf(camera1_to_camera2_, ros::Time::now(), c1_optical_frame_.c_str(), c2_optical_frame_.c_str());
  tf_broadcaster_.sendTransform(stf);

  if (!c1_target_found) ROS_ERROR("Target Location Failure camera1");
  if (!c2_target_found) ROS_ERROR("Target Location Failure camera2");
  if (c1_target_found && c2_target_found) return (true);
  return (false);
}

bool callService::resetC2()
{
  bool c1_target_found = false;
  bool c2_target_found = false;
  tf::Transform c1_to_target;
  tf::Transform c2_to_target;

  while (!c1_client_.call(c1_srv_))
  {
    ROS_ERROR("C1 call failed");
    sleep(1);
  }
  double x, y, z, qx, qy, qz, qw;
  x = c1_srv_.response.final_pose.position.x;
  y = c1_srv_.response.final_pose.position.y;
  z = c1_srv_.response.final_pose.position.z;
  qx = c1_srv_.response.final_pose.orientation.x;
  qy = c1_srv_.response.final_pose.orientation.y;
  qz = c1_srv_.response.final_pose.orientation.z;
  qw = c1_srv_.response.final_pose.orientation.w;
  tf::Quaternion quat(qx, qy, qz, qw);
  c1_to_target.setOrigin(tf::Vector3(x, y, z));
  c1_to_target.setRotation(quat);
  c1_target_found = 1;

  while (!c2_client_.call(c2_srv_))
  {
    ROS_ERROR("C2 call failed");
    sleep(1);
  }
  x = c2_srv_.response.final_pose.position.x;
  y = c2_srv_.response.final_pose.position.y;
  z = c2_srv_.response.final_pose.position.z;
  qx = c2_srv_.response.final_pose.orientation.x;
  qy = c2_srv_.response.final_pose.orientation.y;
  qz = c2_srv_.response.final_pose.orientation.z;
  qw = c2_srv_.response.final_pose.orientation.w;
  tf::Quaternion quat2(qx, qy, qz, qw);
  c2_to_target.setOrigin(tf::Vector3(x, y, z));
  c2_to_target.setRotation(quat2);
  c2_target_found = 1;

  if (c1_target_found && c2_target_found)
  {
    camera1_to_camera2_ = c1_to_target * c2_to_target.inverse();
    return (true);
  }
  else
  {
    ROS_ERROR("Target Location Failure Did not reset C1-C2 Transform");
  }
  return (false);
}
void callService::setRequest()
{
  c1_srv_.request.roi.x_offset = 0;
  c1_srv_.request.roi.y_offset = 0;
  c1_srv_.request.roi.width = c1_roi_width_;
  c1_srv_.request.roi.height = c1_roi_height_;
  c1_srv_.request.initial_pose.position.x = 0.0;
  c1_srv_.request.initial_pose.position.y = 0.1;
  c1_srv_.request.initial_pose.position.z = 1.0;
  c1_srv_.request.initial_pose.orientation.x = 1.0;
  c1_srv_.request.initial_pose.orientation.y = 0.0;
  c1_srv_.request.initial_pose.orientation.z = 0.0;
  c1_srv_.request.initial_pose.orientation.w = 0.00000000001;
  c1_srv_.request.allowable_cost_per_observation = 7.0;

  c2_srv_.request.roi.x_offset = 0;
  c2_srv_.request.roi.y_offset = 0;
  c2_srv_.request.roi.width = c2_roi_width_;
  c2_srv_.request.roi.height = c2_roi_height_;
  c2_srv_.request.initial_pose.position.x = 0.0;
  c2_srv_.request.initial_pose.position.y = 0.1;
  c2_srv_.request.initial_pose.position.z = 1.0;
  c2_srv_.request.initial_pose.orientation.x = 1.0;
  c2_srv_.request.initial_pose.orientation.y = 0.0;
  c2_srv_.request.initial_pose.orientation.z = 0.0;
  c2_srv_.request.initial_pose.orientation.w = 0.00000000001;
  c2_srv_.request.allowable_cost_per_observation = 7.0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "call_service");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  callService call_service(nh);
  ros::Rate loop_rate(10);
  while (!call_service.resetC2())
  {
    sleep(3);
  }  // initialized camera1 to camera2 transform
  while (ros::ok())
  {
    if (call_service.callTheService())
    {
      call_service.copyResponseToRequest();
    }
    else
    {
      call_service.setRequest();
    }

    // recompute camera1 to camera2 transform on request
    bool reset_camera2_tf;
    nh.getParam("reset_camera2_tf", reset_camera2_tf);
    if (reset_camera2_tf)
    {
      call_service.resetC2();
      nh.setParam("reset_camera2_tf", false);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
