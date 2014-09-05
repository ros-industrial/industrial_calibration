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

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace industrial_extrinsic_cal;
using industrial_extrinsic_cal::CalibrationJob;

void print_AAtoH(double x, double y, double z, double tx, double ty, double tz);

std::string camera_file_name("/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration/industrial_extrinsic_cal/yaml/test1_camera_def.yaml");
std::string target_file_name("/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration/industrial_extrinsic_cal/yaml/test1_target_def.yaml");
std::string caljob_file_name("/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration/industrial_extrinsic_cal/yaml/test1_caljob_def.yaml");

//CalibrationJob Cal_job(camera_file_name, target_file_name, caljob_file_name);
CalibrationJob Cal_job(camera_file_name, target_file_name, caljob_file_name);

//need: Camera intrinstics, camera extrinsics, observations (x,y),
//camera name, target name, target pose, and point position
std::vector<int> point_ids;
std::vector<float> obs_x;
std::vector<float> obs_y;

P_BLOCK intrinsics;
P_BLOCK extrinsics;
P_BLOCK target_pose;
P_BLOCK pnt_pos;


TEST(DISABLED_IndustrialExtrinsicCalSuite, load_check)
{
  //need: Camera intrinstics, camera extrinsics, observations (x,y),
  //camera name, target name, scene id, pnt id, target pose, and point position

  EXPECT_TRUE(Cal_job.load());//should import camera instrinsics

  //read in observation file which contains Points with pnt_id, x and y
  std::ifstream obs_input_file("/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration/industrial_extrinsic_cal/observations.txt");
  YAML::Parser obs_parser(obs_input_file);

  YAML::Node obs_doc;
  obs_parser.GetNextDocument(obs_doc);

  // read in all observations
  if (const YAML::Node *observations = obs_doc.FindValue("Points"))
  {
    ROS_DEBUG_STREAM("Found "<<observations->size() <<" observations ");
    point_ids.resize(observations->size());
    obs_x.resize(observations->size());
    obs_y.resize(observations->size());
    for (unsigned int i = 0; i < observations->size(); i++)
    {
      (*observations)[i]["Point_id"] >> point_ids.at(i);

      const YAML::Node *points_node = (*observations)[i].FindValue("Pnts");
      std::vector<float> temp_pnt;
      (*points_node) >> temp_pnt;
      obs_x.at(i) = temp_pnt[0];
      obs_y.at(i) = temp_pnt[1];
    }
  }

}

TEST(DISABLED_IndustrialExtrinsicCalSuite, param_check)
{
  //need: Camera intrinstics, camera extrinsics, observations (x,y),
  //camera name, target name, scene id, pnt id, target pose, and point position
  std::vector<ObservationScene> obs_scene_list_ = Cal_job.getSceneList();

  EXPECT_STRCASEEQ("Asus1", obs_scene_list_.at(0).cameras_in_scene_.at(0)->camera_name_.c_str());

  EXPECT_STRCASEEQ("Checkerboard",
                   obs_scene_list_.at(0).observation_command_list_.at(0).target->target_name.c_str());

  CeresBlocks c_blocks_ = Cal_job.getCeresBlocks();
  ASSERT_GT(c_blocks_.static_cameras_.size(), 0);

  intrinsics = c_blocks_.getStaticCameraParameterBlockIntrinsics("Asus1");
  extrinsics = c_blocks_.getStaticCameraParameterBlockExtrinsics("Asus1");

  target_pose = c_blocks_.getStaticTargetPoseParameterBlock("Checkerboard");
  pnt_pos = c_blocks_.getStaticTargetPointParameterBlock("Checkerboard", 0);

  //Cal_job.current_scene_=1;

  EXPECT_EQ(525, intrinsics[0]);//focal_length_x from camera yaml file
  EXPECT_EQ(525, intrinsics[1]);//focal_length_y from camera yaml file
  EXPECT_EQ(320, intrinsics[2]);//center_x from camera yaml file
  EXPECT_EQ(240, intrinsics[3]);//center_y from camera yaml file
  EXPECT_EQ(0.01, intrinsics[4]);//distortion_k1 from camera yaml file
  EXPECT_EQ(0.02, intrinsics[5]);//distortion_k2 from camera yaml file
  EXPECT_EQ(0.03, intrinsics[6]);//distortion_k3 from camera yaml file
  EXPECT_EQ(0.01, intrinsics[7]);//distortion_p1 from camera yaml file
  EXPECT_EQ(0.01, intrinsics[8]);//distortion_p2 from camera yaml file
/*  ROS_INFO_STREAM("instrinsics 0: "<<intrinsics[0]);
  ROS_INFO_STREAM("instrinsics 1: "<<intrinsics[1]);
  ROS_INFO_STREAM("instrinsics 2: "<<intrinsics[2]);
  ROS_INFO_STREAM("instrinsics 3: "<<intrinsics[3]);
  ROS_INFO_STREAM("instrinsics 4: "<<intrinsics[4]);
  ROS_INFO_STREAM("instrinsics 5: "<<intrinsics[5]);
  ROS_INFO_STREAM("instrinsics 6: "<<intrinsics[6]);
  ROS_INFO_STREAM("instrinsics 7: "<<intrinsics[7]);
  ROS_INFO_STREAM("instrinsics 8: "<<intrinsics[8]);
  ROS_INFO_STREAM("extrinsics 0: "<<extrinsics[0]);
  ROS_INFO_STREAM("extrinsics 1: "<<extrinsics[1]);
  ROS_INFO_STREAM("extrinsics 2: "<<extrinsics[2]);
  ROS_INFO_STREAM("extrinsics 3: "<<extrinsics[3]);
  ROS_INFO_STREAM("extrinsics 4: "<<extrinsics[4]);
  ROS_INFO_STREAM("extrinsics 5: "<<extrinsics[5]);*/
  EXPECT_EQ(1.0, extrinsics[0]);//angle_axis_ax in camera yaml file
  EXPECT_EQ(1.1, extrinsics[1]);//angle_axis_ay in camera yaml file
  EXPECT_EQ(1.2, extrinsics[2]);//angle_axis_az in camera yaml file
  EXPECT_EQ(1.3, extrinsics[3]);//position_x in camera yaml file
  EXPECT_EQ(1.4, extrinsics[4]);//position_y in camera yaml file
  EXPECT_EQ(1.5, extrinsics[5]);//position_z in camera yaml file
/*  ROS_INFO_STREAM("target_pose 0: "<<target_pose[0]);
  ROS_INFO_STREAM("target_pose 1: "<<target_pose[1]);
  ROS_INFO_STREAM("target_pose 2: "<<target_pose[2]);
  ROS_INFO_STREAM("target_pose 3: "<<target_pose[3]);
  ROS_INFO_STREAM("target_pose 4: "<<target_pose[4]);
  ROS_INFO_STREAM("target_pose 5: "<<target_pose[5]);*/
  EXPECT_EQ(2.2, target_pose[0]);//position_x from target yaml
  EXPECT_EQ(2.3, target_pose[1]);//position_y from target yaml
  EXPECT_EQ(2.4, target_pose[2]);//position_z from target yaml
  EXPECT_EQ(1.1, target_pose[3]);//angle_axis_ax from target yaml
  EXPECT_EQ(2.0, target_pose[4]);//angle_axis_ay from target yaml
  EXPECT_EQ(2.1, target_pose[5]);//angle_axis_ay from target yaml

  //check first and last points (from target yaml)
  EXPECT_EQ(0.0, pnt_pos[0]);
  EXPECT_EQ(0.0, pnt_pos[1]);
  EXPECT_EQ(0.0, pnt_pos[2]);
  pnt_pos = c_blocks_.getStaticTargetPointParameterBlock("Checkerboard", 120);
  EXPECT_FLOAT_EQ(0.40, pnt_pos[0]);
  EXPECT_FLOAT_EQ(0.40, pnt_pos[1]);
  EXPECT_EQ(0.0, pnt_pos[2]);

  ASSERT_GT(point_ids.size(), 0);
  //check first and last points (from observation.txt)
  EXPECT_EQ(0,point_ids.at(0));
  EXPECT_EQ(120,point_ids.at(120));
  EXPECT_FLOAT_EQ(64.8908386230469,obs_x.at(0));
  EXPECT_FLOAT_EQ(58.9467353820801,obs_y.at(0));
  EXPECT_FLOAT_EQ(282.849487304688,obs_x.at(120));
  EXPECT_FLOAT_EQ(280.590637207031,obs_y.at(120));
}

TEST(DISABLED_IndustrialExtrinsicCalSuite, ceres_check)
{

  CeresBlocks c_blocks_ = Cal_job.getCeresBlocks();
  ceres::Problem problem;//=Cal_job.getProblem();
  for (int i=0; i<point_ids.size(); i++)
  {

    pnt_pos = c_blocks_.getStaticTargetPointParameterBlock("Checkerboard", i);
  // create the cost function
  ceres::CostFunction* cost_function = TargetCameraReprjErrorNoDistortion::Create(obs_x.at(i), obs_y.at(i),
                                                                           intrinsics[0],
                                                                           intrinsics[1],
                                                                           intrinsics[2],
                                                                           intrinsics[3],
                                                                           pnt_pos[0],
                                                                           pnt_pos[1],
                                                                           pnt_pos[2]);
  problem.AddResidualBlock(cost_function, NULL , extrinsics, target_pose);
  }
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 1000;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  //std::cout << summary.FullReport() << "\n";


  printf("World to Camera\n");
  printf("Optimized Camera\n");
  print_AAtoH(extrinsics[0], extrinsics[1], extrinsics[2],
               extrinsics[3], extrinsics[4], extrinsics[5]);

  /*
  //printf("%s\n", words.c_str());
  printf("Camera to World Transform:\n");
  //print_AATasHI(C.aa[0], C.aa[1], C.aa[2], C.pos[0], C.pos[1], C.pos[2]);
  double R[9];
  double aa[3];
  aa[0] = extrinsics[0];
  aa[1] = extrinsics[1];
  aa[2] = extrinsics[2];
  ceres::AngleAxisToRotationMatrix(aa, R);
  double ix = -(extrinsics[3] * R[0] + extrinsics[4] * R[1] + extrinsics[5] * R[2]);
  double iy = -(extrinsics[3] * R[3] + extrinsics[4] * R[4] + extrinsics[5] * R[5]);
  double iz = -(extrinsics[3] * R[6] + extrinsics[4] * R[7] + extrinsics[5] * R[8]);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[0], R[1], R[2], ix);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[3], R[4], R[5], iy);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[6], R[7], R[8], iz);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", 0.0, 0.0, 0.0, 1.0);*/
}
// angle axis to homogeneous transform
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


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{

  ros::init(argc, argv, "test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
