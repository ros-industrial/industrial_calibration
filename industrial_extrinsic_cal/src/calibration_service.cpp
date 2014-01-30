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

bool calibrated=false;
bool callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
//industrial_extrinsic_cal::ROSRuntimeUtils utils;
std::vector<tf::Transform> b_transforms;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration_service_node");

  ros::NodeHandle nh;
  ros::ServiceServer service=nh.advertiseService("calibration_service", callback);
  industrial_extrinsic_cal::ROSRuntimeUtils utils;

  utils.camera_file_="/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration"
      "/industrial_extrinsic_cal/yaml/test2_camera_def.yaml";
  utils.target_file_="/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration"
      "/industrial_extrinsic_cal/yaml/test2_target_def.yaml";
  utils.caljob_file_="/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration"
      "/industrial_extrinsic_cal/yaml/test2_caljob_def.yaml";
  industrial_extrinsic_cal::CalibrationJob cal_job(utils.camera_file_, utils.target_file_, utils.caljob_file_);

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

/*  ROS_INFO_STREAM("Target frame1: "<<utils.target_frame_[0]);
  ROS_INFO_STREAM("World frame: "<<utils.world_frame_);
  ROS_INFO_STREAM("Init tf size: "<<utils.initial_transforms_.size());
  tf::StampedTransform temp_tf;
  try
  {
    utils.listener_.lookupTransform(utils.world_frame_,utils.target_frame_[0], ros::Time(0), temp_tf);
    utils.points_to_world_transforms_.push_back(temp_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  for (int k=0; k<utils.initial_transforms_.size(); k++ )
  {
    utils.initial_transforms_[k]=utils.points_to_world_transforms_[0]*utils.initial_transforms_[k];
  }*/


  utils.broadcasters_.resize(utils.initial_extrinsics_.size());

  ros::Rate r(5); // 5 hz
  while (ros::ok())
  {
    /*if(!calibrated)
    {
      b_transforms=utils.initial_transforms_;
    }*/
    if(calibrated)
      for (int k=0; k<b_transforms.size(); k++ )
      {
        utils.broadcasters_[k].sendTransform(tf::StampedTransform(b_transforms[k], ros::Time::now(),
                                                                  utils.world_frame_, utils.camera_intermediate_frame_[k]));
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
  utils.camera_file_="/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration/"
      "industrial_extrinsic_cal/yaml/test1_camera_def.yaml";
  utils.target_file_="/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration"
      "/industrial_extrinsic_cal/yaml/test1_target_def.yaml";
  utils.caljob_file_="/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration"
      "/industrial_extrinsic_cal/yaml/test1_caljob_def.yaml";
  industrial_extrinsic_cal::CalibrationJob cal_job(utils.camera_file_, utils.target_file_, utils.caljob_file_);
  //ROS_INFO_STREAM("hello world ");
  cal_job.load();
  utils.world_frame_=cal_job.getReferenceFrame();
  utils.camera_optical_frame_=cal_job.getCameraOpticalFrame();
  utils.camera_intermediate_frame_=cal_job.getCameraIntermediateFrame();
  utils.target_frame_=cal_job.getTargetFrames();
  if (cal_job.run())
  {
    ROS_INFO_STREAM("Calibration job observations and optimization complete");
  }
  utils.calibrated_extrinsics_ = cal_job.getExtrinsics();
  utils.target_poses_ = cal_job.getTargetPose();
  ROS_DEBUG_STREAM("Size of optimized_extrinsics_: "<<utils.calibrated_extrinsics_.size());
  ROS_DEBUG_STREAM("Size of targets_: "<<utils.target_poses_.size());

  industrial_extrinsic_cal::P_BLOCK optimized_extrinsics, target;
  tf::Transform tf_camera, tf_target;
  for (int k=0; k<utils.calibrated_extrinsics_.size(); k++ )
  {
    optimized_extrinsics=utils.calibrated_extrinsics_[k];
    ROS_INFO_STREAM("Optimized Camera "<<k);
     tf_camera= utils.pblockToPose(optimized_extrinsics);
    utils.calibrated_transforms_.push_back(tf_camera);
  }
  for (int k=0; k<utils.target_poses_.size(); k++ )
  {
    target=utils.target_poses_[k];
    ROS_INFO_STREAM("Optimized Target "<<k);
    tf_target = utils.pblockToPose(target);
    utils.target_transforms_.push_back(tf_target);
  }
  tf::StampedTransform temp_tf;
  for (int i=0; i<utils.calibrated_extrinsics_.size(); i++ )
  {
    try
    {
      utils.listener_.lookupTransform( utils.camera_optical_frame_[i],utils.camera_intermediate_frame_[i],
                                       ros::Time(0), temp_tf);
      utils.camera_internal_transforms_.push_back(temp_tf);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }
  }
  ROS_INFO_STREAM("Size of internal_transforms: "<<utils.camera_internal_transforms_.size());
  for (int k=0; k<utils.calibrated_transforms_.size(); k++ )
  {
    utils.calibrated_transforms_[k]=utils.calibrated_transforms_[k]*utils.camera_internal_transforms_[k];
  }
  ROS_INFO_STREAM("Target frame1: "<<utils.target_frame_[0]);
  ROS_INFO_STREAM("World frame: "<<utils.world_frame_);
  try
  {
    utils.listener_.lookupTransform(utils.world_frame_,utils.target_frame_[0], ros::Time(0), temp_tf);
    utils.points_to_world_transforms_.push_back(temp_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  for (int k=0; k<utils.calibrated_transforms_.size(); k++ )
  {
    utils.calibrated_transforms_[k]=utils.points_to_world_transforms_[0]*utils.calibrated_transforms_[k];
  }

  b_transforms=utils.calibrated_transforms_;
  calibrated=true;

  if (cal_job.store())
  {
    ROS_INFO_STREAM("Calibration job optimization saved to file");
  }

  ROS_INFO_STREAM("Camera pose(s) published");

  return true;
}
