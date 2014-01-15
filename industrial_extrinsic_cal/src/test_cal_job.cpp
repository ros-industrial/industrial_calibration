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
#include <industrial_extrinsic_cal/observation_data_point.h>

using industrial_extrinsic_cal::CalibrationJob;
using std::string;

void print_AAtoH(double x, double y, double z, double tx, double ty, double tz);
int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_node_name");
  string camera_file_name(
      "/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration/industrial_extrinsic_cal/yaml/test1a_camera_def.yaml");
  string target_file_name(
      "/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration/industrial_extrinsic_cal/yaml/test1_target_def.yaml");
  string caljob_file_name(
      "/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration/industrial_extrinsic_cal/yaml/test1a_caljob_def.yaml");
  CalibrationJob Cal_job(camera_file_name, target_file_name, caljob_file_name);
  //ROS_INFO_STREAM("hello world ");
  if (Cal_job.load())
  {
    ROS_INFO_STREAM("Calibration job (cal_job, target and camera) yaml parameters loaded.");
  }

  industrial_extrinsic_cal::CeresBlocks c_blocks = Cal_job.getCeresBlocks();
  //c_blocks.static_cameras_.at(0)->camera_parameters_.pb_extrinsics;
  industrial_extrinsic_cal::P_BLOCK original_extrinsics= c_blocks.static_cameras_.at(0)->camera_parameters_.pb_extrinsics;//c_blocks.getStaticCameraParameterBlockExtrinsics("Asus1");

  if (Cal_job.run())
  {
    ROS_INFO_STREAM("Calibration job observations and optimization complete");
  }

  //industrial_extrinsic_cal::P_BLOCK original_extrinsics= c_blocks.getStaticCameraParameterBlockExtrinsics("Asus1");
  industrial_extrinsic_cal::P_BLOCK optimized_extrinsics = Cal_job.getExtrinsics();

  printf("World to Camera\n");
  printf("Original Camera\n");
  print_AAtoH(original_extrinsics[0], original_extrinsics[1], original_extrinsics[2],
               original_extrinsics[3], original_extrinsics[4], original_extrinsics[5]);
  printf("Optimized Camera\n");
  print_AAtoH(optimized_extrinsics[0], optimized_extrinsics[1], optimized_extrinsics[2],
               optimized_extrinsics[3], optimized_extrinsics[4], optimized_extrinsics[5]);
}

// angle axis to homogeneous transform inverted
void print_AAtoH(double x, double y, double z, double tx, double ty, double tz)
{
  double R[9];
  double aa[3];
  aa[0] = x;
  aa[1] = y;
  aa[2] = z;
  ceres::AngleAxisToRotationMatrix(aa, R);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[0], R[3], R[6], tx);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[1], R[4], R[7], ty);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[2], R[5], R[8], tz);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", 0.0, 0.0, 0.0, 1.0);
}

