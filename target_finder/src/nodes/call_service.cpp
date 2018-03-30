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

class callService
{
public:
  callService(ros::NodeHandle nh) : nh_(nh)
  {
    client_ = nh_.serviceClient<target_finder::target_locator>("TargetLocateService");
    ros::NodeHandle pnh("~");
    if (!pnh.getParam("roi_width", roi_width_))
    {
      roi_width_ = 1280;
    }
    if (!pnh.getParam("roi_height", roi_height_))
    {
      roi_height_ = 1024;
    }
    if (!pnh.getParam("optical_frame", optical_frame_))
    {
      optical_frame_ = "basler1_optical_frame";
    }
    setRequest();
  }
  bool callTheService();
  void copyResponseToRequest();
  void setRequest();

private:
  ros::NodeHandle nh_;
  ros::ServiceClient client_;
  target_finder::target_locator srv_;
  tf::TransformBroadcaster tf_broadcaster_;
  int roi_width_;
  int roi_height_;
  std::string optical_frame_;
};

void callService::copyResponseToRequest()
{
  setRequest();
  srv_.request.initial_pose.position.x = srv_.response.final_pose.position.x;
  srv_.request.initial_pose.position.y = srv_.response.final_pose.position.y;
  srv_.request.initial_pose.position.z = srv_.response.final_pose.position.z;
  srv_.request.initial_pose.orientation.x = srv_.response.final_pose.orientation.x;
  srv_.request.initial_pose.orientation.y = srv_.response.final_pose.orientation.y;
  srv_.request.initial_pose.orientation.z = srv_.response.final_pose.orientation.z;
  srv_.request.initial_pose.orientation.w = srv_.response.final_pose.orientation.w;
}
bool callService::callTheService()
{
  if (client_.call(srv_))
  {
    double x, y, z, qx, qy, qz, qw;
    x = srv_.response.final_pose.position.x;
    y = srv_.response.final_pose.position.y;
    z = srv_.response.final_pose.position.z;
    qx = srv_.response.final_pose.orientation.x;
    qy = srv_.response.final_pose.orientation.y;
    qz = srv_.response.final_pose.orientation.z;
    qw = srv_.response.final_pose.orientation.w;
    ROS_INFO("Pose: tx= %5.4lf  %5.4lf  %5.4lf quat= %5.3lf  %5.3lf  %5.3lf %5.3lf, cost= %5.3lf",
             srv_.response.final_pose.position.x, srv_.response.final_pose.position.y,
             srv_.response.final_pose.position.z, srv_.response.final_pose.orientation.x,
             srv_.response.final_pose.orientation.y, srv_.response.final_pose.orientation.z,
             srv_.response.final_pose.orientation.w, srv_.response.final_cost_per_observation);
    tf::Transform camera_to_target;
    tf::Quaternion quat(qx, qy, qz, qw);
    camera_to_target.setOrigin(tf::Vector3(x, y, z));
    camera_to_target.setRotation(quat);
    tf::StampedTransform stf(camera_to_target, ros::Time::now(), optical_frame_.c_str(), "target_frame");
    tf_broadcaster_.sendTransform(stf);
    return (true);
  }
  ROS_ERROR("Target Location Failure");
  return (false);
}
void callService::setRequest()
{
  srv_.request.roi.x_offset = 0;
  srv_.request.roi.y_offset = 0;
  srv_.request.roi.width = roi_width_;
  srv_.request.roi.height = roi_height_;
  srv_.request.initial_pose.position.x = 0.0;
  srv_.request.initial_pose.position.y = 0.1;
  srv_.request.initial_pose.position.z = 1.0;
  srv_.request.initial_pose.orientation.x = 1.0;
  srv_.request.initial_pose.orientation.y = 0.0;
  srv_.request.initial_pose.orientation.z = 0.0;
  srv_.request.initial_pose.orientation.w = 0.00000000001;
  srv_.request.allowable_cost_per_observation = 7.0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "call_service");
  ros::NodeHandle nh;
  callService call_service(nh);
  ros::Rate loop_rate(10);
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
    ros::spinOnce();
    loop_rate.sleep();
  }
}
