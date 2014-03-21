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

#include <industrial_extrinsic_cal/runtime_utils.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <ros/package.h>

bool calibrated=false;
bool callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
std::vector<tf::Transform> b_transforms;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration_service_node");

  ros::NodeHandle nh;
  ros::ServiceServer service=nh.advertiseService("calibration_service", callback);
  industrial_extrinsic_cal::ROSRuntimeUtils utils;
  ros::NodeHandle priv_nh_("~");

  priv_nh_.getParam("camera_file", utils.camera_file_);
  priv_nh_.getParam("target_file", utils.target_file_);
  priv_nh_.getParam("cal_job_file", utils.caljob_file_);
  std::string path = ros::package::getPath("industrial_extrinsic_cal");
  std::string file_path=path+"/yaml/";
  industrial_extrinsic_cal::CalibrationJob cal_job(file_path+utils.camera_file_, file_path+utils.target_file_, file_path+utils.caljob_file_);

  if (cal_job.load())
  {
    ROS_INFO_STREAM("Calibration job (cal_job, target and camera) yaml parameters loaded.");
  }

  utils.world_frame_=cal_job.getReferenceFrame();
  utils.camera_optical_frame_=cal_job.getCameraOpticalFrame();
  utils.camera_intermediate_frame_=cal_job.getCameraIntermediateFrame();
  utils.initial_extrinsics_ = cal_job.getOriginalExtrinsics();
  utils.target_frame_=cal_job.getTargetFrames();
  industrial_extrinsic_cal::P_BLOCK orig_extrinsics;
  tf::Transform tf_camera_orig;
  for (int k=0; k<utils.initial_extrinsics_.size(); k++ )
  {
    orig_extrinsics=utils.initial_extrinsics_[k];
    ROS_INFO_STREAM("Original Camera "<<k);
    tf_camera_orig= utils.pblockToPose(orig_extrinsics);
    utils.initial_transforms_.push_back(tf_camera_orig);
  }
  tf::StampedTransform temp_tf;
  try
  {
    utils.listener_.waitForTransform( utils.world_frame_,utils.target_frame_[0],
                                      ros::Time(0), ros::Duration(3.0));
    utils.listener_.lookupTransform(utils.world_frame_,utils.target_frame_[0], ros::Time(0), temp_tf);
    utils.points_to_world_transforms_.push_back(temp_tf);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  for (int k=0; k<utils.initial_transforms_.size(); k++ )
  {
    utils.initial_transforms_[k]=utils.points_to_world_transforms_[0]*utils.initial_transforms_[k];
  }


  utils.broadcasters_.resize(utils.initial_extrinsics_.size());

  ros::Rate r(5); // 5 hz
  while (ros::ok())
  {
    if(!calibrated)
    {
      b_transforms=utils.initial_transforms_;
      for (int k=0; k<b_transforms.size(); k++ )
      {
        utils.broadcasters_[k].sendTransform(tf::StampedTransform(b_transforms[k], ros::Time::now(),
                                                                  utils.world_frame_, utils.camera_intermediate_frame_[k]));
      }
    }
    else if(calibrated)
    {
      for (int k=0; k<b_transforms.size(); k++ )
      {
        utils.broadcasters_[k].sendTransform(tf::StampedTransform(b_transforms[k], ros::Time::now(),
                                                                  utils.world_frame_, utils.camera_intermediate_frame_[k]));
      }
    }
    ros::spinOnce();
    r.sleep();
  }


  ros::spin();
  return 0;
}

bool callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  industrial_extrinsic_cal::ROSRuntimeUtils utils;
  ros::NodeHandle priv_nh_("~");

  std::string ros_package_name;
  std::string launch_file_name;

  priv_nh_.getParam("camera_file", utils.camera_file_);
  priv_nh_.getParam("target_file", utils.target_file_);
  priv_nh_.getParam("cal_job_file", utils.caljob_file_);
  priv_nh_.getParam("store_results_package_name", ros_package_name);
  priv_nh_.getParam("store_results_file_name", launch_file_name);

  std::string path = ros::package::getPath("industrial_extrinsic_cal");
  std::string file_path=path+"/yaml/";
  ROS_INFO("path: %s",file_path.c_str());
  ROS_INFO("camera_file: %s",utils.camera_file_.c_str());
  ROS_INFO("target_file: %s",utils.target_file_.c_str());
  ROS_INFO("cal_job_file: %s",utils.caljob_file_.c_str());
  ROS_INFO("store results: %s",ros_package_name.c_str());
  ROS_INFO("launch_file_name: %s",launch_file_name.c_str());

  industrial_extrinsic_cal::CalibrationJob cal_job(file_path+utils.camera_file_, file_path+utils.target_file_, file_path+utils.caljob_file_);

  cal_job.load();
  utils.world_frame_=cal_job.getReferenceFrame();
  utils.camera_optical_frame_=cal_job.getCameraOpticalFrame();
  utils.camera_intermediate_frame_=cal_job.getCameraIntermediateFrame();
  utils.target_frame_=cal_job.getTargetFrames();

  ROS_INFO("world_fame: %s",utils.world_frame_.c_str());
  for(int i=0;i<(int)utils.camera_optical_frame_.size();i++){
    ROS_INFO("optical_fame %d: %s",i,utils.camera_optical_frame_[i].c_str());
  }
  for(int i=0;i<(int)utils.camera_intermediate_frame_.size();i++){
    ROS_INFO("intermediate_fame %d: %s",i,utils.camera_intermediate_frame_[i].c_str());
  }
  for(int i=0;i<(int)utils.target_frame_.size();i++){
    ROS_INFO("target_fame %d: %s",i,utils.target_frame_[i].c_str());
  }

  ROS_INFO("State prior to optimization");
  cal_job.show();

  ROS_INFO("RUNNING");
  if (cal_job.run())
  {
    ROS_INFO_STREAM("Calibration job observations and optimization complete");
    calibrated=true;
  }
  else
    {
      ROS_INFO_STREAM("Calibration job failed");
      return(false);
    }
  
  // PRINT RESULTS TO THE SCREEN
  
  if (!cal_job.store())
  {
    ROS_INFO_STREAM(" Trouble storing calibration job optimization results ");
  }
  cal_job.show();
  return true;
}
