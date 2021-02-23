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
#include <target_finder/target_verify.h>
#include <target_finder/target_save_location.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>

using std::string;
using std::vector;

class callService
{
public:
  callService(ros::NodeHandle nh) : nh_(nh)
  {
    save_service_ = nh_.advertiseService("save_pose", &callService::saveCallBack, this);
    verify_service_ = nh_.advertiseService("verify_pose", &callService::verifyCallBack, this);
    client_ = nh_.serviceClient<target_finder::target_locator>("/target_locator_srv/target_locator/target_locate_srv");
    save_client_ = nh_.serviceClient<target_finder::target_save_location>("/target_locator_srv/target_locator/"
                                                                          "target_save_location_srv");
    verify_client_ = nh_.serviceClient<target_finder::target_verify>("/target_locator_srv/target_locator/"
                                                                     "target_verify_srv");
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
    if (!pnh.getParam("pose_file_name", pose_file_name_))
    {
      pose_file_name_ = "verify_pose.yaml";
    }
    setRequest();
  }
  bool callTheService();
  void copyResponseToRequest();
  void setRequest();
  bool saveCallBack(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
  bool verifyCallBack(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer save_service_;
  ros::ServiceServer verify_service_;
  ros::ServiceClient client_;
  ros::ServiceClient save_client_;
  ros::ServiceClient verify_client_;
  target_finder::target_locator srv_;
  target_finder::target_save_location save_srv_;
  target_finder::target_verify verify_srv_;
  tf::TransformBroadcaster tf_broadcaster_;
  int roi_width_;
  int roi_height_;
  std::string optical_frame_;
  std::string pose_file_name_;
};

bool callService::saveCallBack(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  setRequest();  // copies default initial conditions
  save_srv_.request.initial_pose.position.x = srv_.request.initial_pose.position.x;
  save_srv_.request.initial_pose.position.y = srv_.request.initial_pose.position.y;
  save_srv_.request.initial_pose.position.z = srv_.request.initial_pose.position.z;
  save_srv_.request.initial_pose.orientation.x = srv_.request.initial_pose.orientation.x;
  save_srv_.request.initial_pose.orientation.y = srv_.request.initial_pose.orientation.y;
  save_srv_.request.initial_pose.orientation.z = srv_.request.initial_pose.orientation.z;
  save_srv_.request.initial_pose.orientation.w = srv_.request.initial_pose.orientation.w;
  save_srv_.request.allowable_cost_per_observation = 7.0;
  save_srv_.request.file_name.data = pose_file_name_;
  if (save_client_.call(save_srv_))
  {
    if (save_srv_.response.success)
    {
      res.success = true;
      res.message = std::string("Pose computed and saved");
    }
    else
    {
      res.success = false;
      res.message = std::string("Pose not successfuly computed");
    }
  }
  else
  {
    res.success = false;
    res.message = std::string("Couldn't make the call to the save client");
  }
  return (true);
}

bool callService::verifyCallBack(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  setRequest();  // copies default initial conditions
  verify_srv_.request.initial_pose.position.x = srv_.request.initial_pose.position.x;
  verify_srv_.request.initial_pose.position.y = srv_.request.initial_pose.position.y;
  verify_srv_.request.initial_pose.position.z = srv_.request.initial_pose.position.z;
  verify_srv_.request.initial_pose.orientation.x = srv_.request.initial_pose.orientation.x;
  verify_srv_.request.initial_pose.orientation.y = srv_.request.initial_pose.orientation.y;
  verify_srv_.request.initial_pose.orientation.z = srv_.request.initial_pose.orientation.z;
  verify_srv_.request.initial_pose.orientation.w = srv_.request.initial_pose.orientation.w;
  verify_srv_.request.file_name.data = pose_file_name_;
  verify_srv_.request.allowable_cost_per_observation = 7.0;
  verify_srv_.request.max_error = 0.005;
  if (verify_client_.call(verify_srv_))
  {
    if (verify_srv_.response.success)
    {
      res.success = true;
      res.message = std::string("Pose verified");
    }
    else
    {
      res.success = false;
      res.message = std::string("Pose verification failed");
    }
  }
  else
  {
    res.success = false;
    res.message = std::string("Couldn't make the call to the verify client");
  }
  return (true);
}
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
    if (std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(qx) || std::isnan(qy) || std::isnan(qz) ||
        std::isnan(qw))
    {
      ROS_ERROR("One or more values of returned transform are NAN");
      setRequest();  // resetting the initial conditions of the request
      return (false);
    }
    else if (srv_.response.cost_per_observation == 0)
    {
      ROS_ERROR("could not find target");
      setRequest();  // resetting the initial conditions of the request
      return (false);
    }
    else
    {
      ROS_INFO("Pose: tx= %5.4lf  %5.4lf  %5.4lf quat= %5.3lf  %5.3lf  %5.3lf %5.3lf, cost= %5.3lf",
               srv_.response.final_pose.position.x, srv_.response.final_pose.position.y,
               srv_.response.final_pose.position.z, srv_.response.final_pose.orientation.x,
               srv_.response.final_pose.orientation.y, srv_.response.final_pose.orientation.z,
               srv_.response.final_pose.orientation.w, srv_.response.cost_per_observation);
      tf::Transform camera_to_target;
      tf::Quaternion quat(qx, qy, qz, qw);
      camera_to_target.setOrigin(tf::Vector3(x, y, z));
      camera_to_target.setRotation(quat);
      tf::StampedTransform stf(camera_to_target, ros::Time::now(), optical_frame_.c_str(), "target_frame");
      tf_broadcaster_.sendTransform(stf);
    }
    return (true);
  }
  ROS_ERROR("Couldn't call service");
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
