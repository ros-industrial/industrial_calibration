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

#include <industrial_extrinsic_cal/calibration_job_definition.h>

using industrial_extrinsic_cal::CalibrationJob;
using std::string;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_node_name");
  string camera_file_name(
      "/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration/industrial_extrinsic_cal/yaml/test1_camera_def.yaml");
  string target_file_name(
      "/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration/industrial_extrinsic_cal/yaml/test1_target_def.yaml");
  string caljob_file_name(
      "/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration/industrial_extrinsic_cal/yaml/test1_caljob_def.yaml");
  CalibrationJob Cal_job(camera_file_name, target_file_name, caljob_file_name);
  //ROS_INFO_STREAM("hello world ");
  if (Cal_job.load())
  {
    ROS_INFO_STREAM("Calibration job (cal_job, target and camera) yaml parameters loaded.");
  }
  if (Cal_job.run())
  {
    ROS_INFO_STREAM("Calibration job observations run");
  }
}
