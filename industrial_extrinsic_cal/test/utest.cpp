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
#include <industrial_extrinsic_cal/target.h>
#include <industrial_extrinsic_cal/camera_definition.h>
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
  industrial_extrinsic_cal::CalibrationJob cal_job(yaml_path + camera_file, yaml_path + target_file,
                                                   yaml_path + caljob_file);
  EXPECT_TRUE(cal_job.load());  // read in cameras, targets, and scenes
  industrial_extrinsic_cal::CeresBlocks* cblocks = cal_job.getBlocks();
  std::vector<industrial_extrinsic_cal::ObservationScene>* scenes = cal_job.getScenes();
  // now, compare the data in these to that expected
  EXPECT_EQ((int)scenes->size(), 2);
  EXPECT_EQ((*scenes)[0].get_id(), 0);
  EXPECT_EQ((*scenes)[1].get_id(), 1);

  // cursury check to see if the targets were loaded by names
  boost::shared_ptr<industrial_extrinsic_cal::Target> T1 = boost::make_shared<industrial_extrinsic_cal::Target>();
  boost::shared_ptr<industrial_extrinsic_cal::Target> T2 = boost::make_shared<industrial_extrinsic_cal::Target>();
  boost::shared_ptr<industrial_extrinsic_cal::Target> T3 = boost::make_shared<industrial_extrinsic_cal::Target>();
  boost::shared_ptr<industrial_extrinsic_cal::Target> T4 = boost::make_shared<industrial_extrinsic_cal::Target>();
  T1 = cblocks->getTargetByName("Chessboard");
  EXPECT_EQ(T1->target_name_, "Chessboard");
  T2 = cblocks->getTargetByName("CircleGrid");
  EXPECT_EQ(T2->target_name_, "CircleGrid");
  T3 = cblocks->getTargetByName("MCircleGrid");
  EXPECT_EQ(T3->target_name_, "MCircleGrid");
  T4 = cblocks->getTargetByName("Balls");
  EXPECT_EQ(T4->target_name_, "Balls");

  // cursury check to see if the cameras were loaded by names
  boost::shared_ptr<industrial_extrinsic_cal::Camera> C1 = boost::make_shared<industrial_extrinsic_cal::Camera>();
  boost::shared_ptr<industrial_extrinsic_cal::Camera> C2 = boost::make_shared<industrial_extrinsic_cal::Camera>();
  boost::shared_ptr<industrial_extrinsic_cal::Camera> C3 = boost::make_shared<industrial_extrinsic_cal::Camera>();
  boost::shared_ptr<industrial_extrinsic_cal::Camera> C4 = boost::make_shared<industrial_extrinsic_cal::Camera>();
  C1 = cblocks->getCameraByName("asus1");
  EXPECT_EQ(C1->camera_name_, "asus1");
  C2 = cblocks->getCameraByName("asus2");
  EXPECT_EQ(C2->camera_name_, "asus2");
  C3 = cblocks->getCameraByName("asus6");
  EXPECT_EQ(C3->camera_name_, "asus6");
  C4 = cblocks->getCameraByName("asus7");
  EXPECT_EQ(C4->camera_name_, "asus7");
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  testing::InitGoogleTest(&argc, argv);
  ros::AsyncSpinner spinner(4);
  spinner.start();
  bool rtn = RUN_ALL_TESTS();
  ros::waitForShutdown();
  return (rtn);
}
