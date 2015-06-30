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

#include <industrial_extrinsic_cal/calibration_job_definition.h>
#include <industrial_extrinsic_cal/ceres_blocks.h>
#include <industrial_extrinsic_cal/observation_scene.h>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

TEST(IndustrialExtrinsicCalSuite, loadCamera)
{
  
  ros::NodeHandle nh("~");
  std::string yaml_path;
  // set test/yaml_file_path to the test/yaml directory
  EXPECT_TRUE(nh.getParam("yaml_file_path", yaml_path));

  std::string camera_file("/test_cameras.yaml");
  std::string target_file("/test_targets.yaml");
  std::string caljob_file("/test_caljob.yaml");
  industrial_extrinsic_cal::CalibrationJob cal_job(yaml_path+camera_file, yaml_path+target_file, yaml_path+caljob_file);
  EXPECT_TRUE(cal_job.load()); // read in cameras, targets, and scenes
  industrial_extrinsic_cal::CeresBlocks * cblocks = cal_job.getBlocks();
  std::vector<industrial_extrinsic_cal::ObservationScene> * scenes = cal_job.getScenes();
  // now, compare the data in these to that expected

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  testing::InitGoogleTest(&argc, argv);
  ros::AsyncSpinner spinner(4);
  spinner.start();
  bool rtn = RUN_ALL_TESTS();
  ros::waitForShutdown();
  return(rtn);
}
