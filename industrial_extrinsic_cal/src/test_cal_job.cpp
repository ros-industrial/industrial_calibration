/*
 * test_cal_job.cpp
 *
 *  Created on: Dec 27, 2013
 *      Author: cgomez
 */

#include <industrial_extrinsic_cal/calibration_job_definition.h>

using industrial_extrinsic_cal::CalibrationJob;
using std::string;
int main()
{
  string camera_file_name(
      "/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration/industrial_extrinsic_cal/yaml/camera_def.yaml");
  string target_file_name(
      "/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration/industrial_extrinsic_cal/yaml/target_def.yaml");
  string caljob_file_name(
      "/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration/industrial_extrinsic_cal/yaml/caljob_def.yaml");
  CalibrationJob Cal_job(camera_file_name, target_file_name, caljob_file_name);
  ROS_INFO_STREAM("hello world ");
  if (Cal_job.load())
  {
    ROS_INFO_STREAM("Calibration job (cal_job, target and camera) yaml parameters loaded.");
  }
  if (Cal_job.runObservations())
  {
    ROS_INFO_STREAM("Calibration job observations run");
  }
}
