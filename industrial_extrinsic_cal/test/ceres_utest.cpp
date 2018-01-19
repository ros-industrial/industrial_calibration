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

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <industrial_extrinsic_cal/ceres_costs_utils.hpp>

using namespace industrial_extrinsic_cal;

Point3d xformPoint(Point3d& original_point, double& ax, double& ay, double& az, double& x, double& y, double& z);

class Observation
{
public:
  Observation()
  {
    point_id = 0;
    image_loc_x = 0.0;
    image_loc_y = 0.0;
  };
  ~Observation(){};
  int point_id;
  double image_loc_x;
  double image_loc_y;
};

Observation projectPoint(CameraParameters C, Point3d P)
{
  double p[3];
  double pt[3];
  pt[0] = P.x;
  pt[1] = P.y;
  pt[2] = P.z;

  /* transform point into camera frame */
  /* note, camera transform takes points from camera frame into world frame */
  ceres::AngleAxisRotatePoint(C.angle_axis, pt, p);

  p[0] += C.position[0];
  p[1] += C.position[1];
  p[2] += C.position[2];

  double xp = p[0] / p[2];
  double yp = p[1] / p[2];

  double r2 = xp * xp + yp * yp;
  double r4 = r2 * r2;
  double r6 = r2 * r4;

  double xp2 = xp * xp; /* temporary variables square of others */
  double yp2 = yp * yp;

  /* apply the distortion coefficients to refine pixel location */
  double xpp = xp + C.distortion_k1 * r2 * xp + C.distortion_k2 * r4 * xp + C.distortion_k3 * r6 * xp +
               C.distortion_p2 * (r2 + 2 * xp2) + C.distortion_p1 * xp * yp * 2.0;
  double ypp = yp + C.distortion_k1 * r2 * yp + C.distortion_k2 * r4 * yp + C.distortion_k3 * r6 * yp +
               C.distortion_p1 * (r2 + 2 * yp2) + C.distortion_p2 * xp * yp * 2.0;

  /* perform projection using focal length and camera center into image plane */
  Observation O;
  O.point_id = 0;
  O.image_loc_x = C.focal_length_x * xpp + C.center_x;
  O.image_loc_y = C.focal_length_y * ypp + C.center_y;

  return (O);
}

// GLOBAL VARIABLES FOR TESTING
std::vector<Point3d> created_points;
double aa[3];  // angle axis known/set
double p[3];   // point rotated known/set
std::vector<Point3d> transformed_points;
std::vector<Observation> observations;
CameraParameters C;

TEST(IndustrialExtrinsicCalCeresSuite, rotationProduct)
{
  double R1[9];
  double R2[9];
  double R3[9];
  double R1p[9];
  double R2p[9];

  // create 2 poses, one the inverse of the other
  // multiply them and see if we get identity
  Pose6d P1, P2;
  P1.setAngleAxis(.1, .2, 3.0);
  P2 = P1.getInverse();
  ceres::AngleAxisToRotationMatrix(P1.pb_aa, R1);
  ceres::AngleAxisToRotationMatrix(P2.pb_aa, R2);
  rotationProduct(R1, R2, R3);
  // NOTE, this should work regardless of row/column major snaffos
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (i == j)
      {
        ASSERT_NEAR(1.0, R3[i + 3 * j], .001);
      }
      else
      {
        ASSERT_NEAR(0.0, R3[i + 3 * j], .001);
      }
    }
  }
}
TEST(IndustrialExtrinsicCalCeresSuite, extractCameraIntrinsics)
{
  C.angle_axis[0] = 0.0;
  C.angle_axis[1] = 0.0;
  C.angle_axis[2] = 0.0;
  C.position[0] = 0.0;
  C.position[1] = 0.0;
  C.position[2] = 0.0;
  C.focal_length_x = 525;
  C.focal_length_y = 525;
  C.center_x = 320;
  C.center_y = 240;
  C.distortion_k1 = 0.01;
  C.distortion_k2 = 0.02;
  C.distortion_k3 = 0.03;
  C.distortion_p1 = 0.01;
  C.distortion_p2 = 0.01;

  double intrinsics[9];
  intrinsics[0] = C.focal_length_x;
  intrinsics[1] = C.focal_length_y;
  intrinsics[2] = C.center_x;
  intrinsics[3] = C.center_y;
  intrinsics[4] = C.distortion_k1;
  intrinsics[5] = C.distortion_k2;
  intrinsics[6] = C.distortion_k3;
  intrinsics[7] = C.distortion_p1;
  intrinsics[8] = C.distortion_p2;

  double fx, fy, cx, cy, k1, k2, k3, p1, p2;
  extractCameraIntrinsics(C.pb_intrinsics, fx, fy, cx, cy, k1, k2, k3, p1, p2);

  ASSERT_NEAR(C.focal_length_x, fx, .00001);
  ASSERT_NEAR(C.focal_length_y, fy, .00001);
  ASSERT_NEAR(C.center_x, cx, .00001);
  ASSERT_NEAR(C.center_y, cy, .00001);
  ASSERT_NEAR(C.distortion_k1, k1, .00001);
  ASSERT_NEAR(C.distortion_k2, k2, .00001);
  ASSERT_NEAR(C.distortion_k3, k3, .00001);
  ASSERT_NEAR(C.distortion_p1, p1, .00001);
  ASSERT_NEAR(C.distortion_p2, p2, .00001);
}
TEST(IndustrialExtrinsicCalCeresSuite, rotationInverse)
{
  double R1[9], R2[9];
  double angle_axis[3];
  angle_axis[0] = .1;
  angle_axis[1] = -3.0;
  angle_axis[2] = .5;
  ceres::AngleAxisToRotationMatrix(angle_axis, R1);
  rotationInverse(R1, R2);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      int index1 = i * 3 + j;
      int index2 = i + j * 3;
      ASSERT_NEAR(R1[index1], R2[index2], .00001);
    }
  }
}
TEST(IndustrialExtrinsicCalCeresSuite, transformPoint)
{
  double point[3], tx[3];
  tx[0] = 15.0;
  tx[1] = 30.0;
  tx[2] = 23;
  point[0] = 10.0;
  point[1] = -35;
  point[2] = -100;
  double angle_axis[3];
  angle_axis[0] = .1;
  angle_axis[1] = -3.0;
  angle_axis[2] = .5;
  double t_point[3];
  transformPoint(angle_axis, tx, point, t_point);

  Pose6d pose;
  pose.setAngleAxis(angle_axis[0], angle_axis[1], angle_axis[2]);
  pose.setOrigin(tx[0], tx[1], tx[2]);

  // invert transformation and apply
  Pose6d posei = pose.getInverse();
  angle_axis[0] = posei.ax;
  angle_axis[1] = posei.ay;
  angle_axis[2] = posei.az;
  tx[0] = posei.x;
  tx[1] = posei.y;
  tx[2] = posei.z;
  double tt_point[3];
  transformPoint(angle_axis, tx, t_point, tt_point);
  ASSERT_NEAR(tt_point[0], point[0], .00001);
  ASSERT_NEAR(tt_point[1], point[1], .00001);
  ASSERT_NEAR(tt_point[2], point[2], .00001);
}

TEST(IndustrialExtrinsicCalCeresSuite, poseTransformPoint)
{
  Pose6d pose;
  Pose6d posei;
  double point[3];
  point[0] = 10.0;
  point[1] = -35;
  point[2] = -100;
  pose.setAngleAxis(1.2, 2.2, 3.3);
  pose.setOrigin(1.2, 2.2, 3.3);
  posei = pose.getInverse();

  double t_point[3];
  double tt_point[3];
  poseTransformPoint(pose, point, t_point);
  poseTransformPoint(posei, t_point, tt_point);
  ASSERT_NEAR(tt_point[0], point[0], .00001);
  ASSERT_NEAR(tt_point[1], point[1], .00001);
  ASSERT_NEAR(tt_point[2], point[2], .00001);
}

TEST(IndustrialExtrinsicCalCeresSuite, transformPoint3d)
{
  double tx[3];
  tx[0] = 15.0;
  tx[1] = 30.0;
  tx[2] = 23;
  Point3d point;
  point.x = 10.0;
  point.y = -35;
  point.z = -100;
  double angle_axis[3];
  angle_axis[0] = .1;
  angle_axis[1] = -3.0;
  angle_axis[2] = .5;
  double t_point[3];
  transformPoint3d(angle_axis, tx, point, t_point);
  Point3d t_point3d;
  t_point3d.x = t_point[0];
  t_point3d.y = t_point[1];
  t_point3d.z = t_point[2];

  Pose6d pose;
  pose.setAngleAxis(angle_axis[0], angle_axis[1], angle_axis[2]);
  pose.setOrigin(tx[0], tx[1], tx[2]);

  // invert transformation and apply
  Pose6d posei = pose.getInverse();
  angle_axis[0] = posei.ax;
  angle_axis[1] = posei.ay;
  angle_axis[2] = posei.az;
  tx[0] = posei.x;
  tx[1] = posei.y;
  tx[2] = posei.z;

  double tt_point[3];
  transformPoint3d(angle_axis, tx, t_point3d, tt_point);
  ASSERT_NEAR(tt_point[0], point.x, .00001);
  ASSERT_NEAR(tt_point[1], point.y, .00001);
  ASSERT_NEAR(tt_point[2], point.z, .00001);
}
TEST(IndustrialExtrinsicCalCeresSuite, poseRotationMatrix)
{
  Pose6d pose;
  double ax = .5;  // must use small values to avoid alternate solutions
  double ay = .23;
  double az = .45;
  pose.setAngleAxis(ax, ay, az);
  pose.setOrigin(11.5, 21.5, 31.5);
  double R[9];
  poseRotationMatrix(pose, R);
  double aa[3];
  ceres::RotationMatrixToAngleAxis(R, aa);
  ASSERT_NEAR(aa[0], ax, .00001);
  ASSERT_NEAR(aa[1], ay, .00001);
  ASSERT_NEAR(aa[2], az, .00001);
}

TEST(IndustrialExtrinsicCalCeresSuite, cameraPntResidualDist)
{
}

TEST(IndustrialExtrinsicCalCeresSuite, cameraCircResidualDist)
{
}
TEST(IndustrialExtrinsicCalCeresSuite, cameraCircResidual)
{
}
// used for intrinsic cal
// Camera may move to several locations (multiple extrinsics)
// target has no transform, origin is origin of target, single static target
// points are known, no calibraition of the target itself
TEST(IndustrialExtrinsicCalCeresSuite, CircleCameraReprjErrorWithDistortion)
{
}
// used for extrinsic cal of camera on robot with target on ground
// finds transform from link to camera, and transform from robot to target
// should have multiple images from different robot locations
TEST(IndustrialExtrinsicCalCeresSuite, LinkCameraCircleTargetReprjError)
{
}

TEST(IndustrialExtrinsicCalCeresSuite, create_points)
{
  Point3d x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15, x16, x17, x18, x19, x20;
  x1.pb[0] = 1.0;
  x1.pb[1] = 1.0;
  x1.pb[2] = 1;
  created_points.push_back(x1);
  x2.pb[0] = 0;
  x2.pb[1] = 0;
  x2.pb[2] = 0.01;
  created_points.push_back(x2);
  x3.pb[0] = 0.65;
  x3.pb[1] = 0.94;
  x3.pb[2] = 0.01;
  created_points.push_back(x3);
  x4.pb[0] = 0.2;
  x4.pb[1] = 0.3;
  x4.pb[2] = 0.01;
  created_points.push_back(x4);
  x5.pb[0] = 0.372;
  x5.pb[1] = 0.4;
  x5.pb[2] = 0.4;
  created_points.push_back(x5);
  x6.pb[0] = 0.3762;
  x6.pb[1] = 0.8;
  x6.pb[2] = 0.3;
  created_points.push_back(x6);
  x7.pb[0] = 0.4;
  x7.pb[1] = 0.389;
  x7.pb[2] = 0.4;
  created_points.push_back(x7);
  x8.pb[0] = 0.431;
  x8.pb[1] = 0.7;
  x8.pb[2] = 0.7;
  created_points.push_back(x8);
  x9.pb[0] = 0.535;
  x9.pb[1] = 0.9;
  x9.pb[2] = 0.01;
  created_points.push_back(x9);
  x10.pb[0] = 0.596;
  x10.pb[1] = 1;
  x10.pb[2] = 1.0;
  created_points.push_back(x10);
  x11.pb[0] = 0.24;
  x11.pb[1] = 1.0;
  x11.pb[2] = 0.01;
  created_points.push_back(x11);
  x12.pb[0] = 0.673;
  x12.pb[1] = 1.7;
  x12.pb[2] = 0.8;
  created_points.push_back(x12);
  x13.pb[0] = 0.552;
  x13.pb[1] = 1.15;
  x13.pb[2] = 0.01;
  created_points.push_back(x13);
  x14.pb[0] = 0.56;
  x14.pb[1] = 0.81;
  x14.pb[2] = 0.01;
  created_points.push_back(x14);
  x15.pb[0] = .70;
  x15.pb[1] = 1.10;
  x15.pb[2] = 0.01;
  created_points.push_back(x15);
  x16.pb[0] = 0.3762;
  x16.pb[1] = 0.02435;
  x16.pb[2] = 0.3;
  created_points.push_back(x16);
  x17.pb[0] = 0.0234;
  x17.pb[1] = 0.389;
  x17.pb[2] = 0.132;
  created_points.push_back(x17);
  x18.pb[0] = 0.431;
  x18.pb[1] = 0.245;
  x18.pb[2] = 0.0235;
  created_points.push_back(x18);
  x19.pb[0] = 0.535;
  x19.pb[1] = 0.673;
  x19.pb[2] = 0.01;
  created_points.push_back(x19);
  x20.pb[0] = 0.76;
  x20.pb[1] = 0.453;
  x20.pb[2] = 1.0;
  created_points.push_back(x20);
  std::cout << "Original Point 1: " << x1.pb[0] << " " << x1.pb[1] << " " << x1.pb[2] << std::endl;

  // create known transform
  aa[0] = 2.7;
  aa[1] = 0.3;
  aa[2] = 0.1;
  p[0] = 0.2;
  p[1] = 0.4;
  p[2] = 0.5;

  // initialize the camera parameters
  C.angle_axis[0] = 0.0;
  C.angle_axis[1] = 0.0;
  C.angle_axis[2] = 0.0;
  C.position[0] = 0.0;
  C.position[1] = 0.0;
  C.position[2] = 0.0;
  C.focal_length_x = 525;
  C.focal_length_y = 525;
  C.center_x = 320;
  C.center_y = 240;
  C.distortion_k1 = 0.01;
  C.distortion_k2 = 0.02;
  C.distortion_k3 = 0.03;
  C.distortion_p1 = 0.01;
  C.distortion_p2 = 0.01;

  // transform points, and then project into image plane
  Point3d t_point;
  Observation o_point;
  for (int i = 0; i < created_points.size(); i++)
  {
    t_point = xformPoint(created_points.at(i), aa[0], aa[1], aa[2], p[0], p[1], p[2]);
    o_point = projectPoint(C, t_point);
    transformed_points.push_back(t_point);
    observations.push_back(o_point);
  }
}

TEST(IndustrialExtrinsicCalCeresSuite, points_costfunction)
{
  // create known transform to move created points to transformed points
  aa[0] = 2.7;
  aa[1] = 0.3;
  aa[2] = 0.1;
  p[0] = 0.2;
  p[1] = 0.4;
  p[2] = 0.5;

  // initialize the camera parameters
  C.angle_axis[0] = 0.0;
  C.angle_axis[1] = 0.0;
  C.angle_axis[2] = 0.0;
  C.position[0] = 0.0;
  C.position[1] = 0.0;
  C.position[2] = 0.0;
  C.focal_length_x = 525;
  C.focal_length_y = 525;
  C.center_x = 320;
  C.center_y = 240;
  C.distortion_k1 = 0.01;
  C.distortion_k2 = 0.02;
  C.distortion_k3 = 0.03;
  C.distortion_p1 = 0.01;
  C.distortion_p2 = 0.01;

  // transform points, and then project into image plane
  Point3d t_point;
  Observation o_point;
  for (int i = 0; i < created_points.size(); i++)
  {
    t_point = xformPoint(created_points.at(i), aa[0], aa[1], aa[2], p[0], p[1], p[2]);
    o_point = projectPoint(C, t_point);
    transformed_points.push_back(t_point);
    observations.push_back(o_point);
  }

  double extrinsics[6];
  extrinsics[0] = C.angle_axis[0];
  extrinsics[1] = C.angle_axis[1];
  extrinsics[2] = C.angle_axis[2];
  extrinsics[3] = C.position[0];
  extrinsics[4] = C.position[1];
  extrinsics[5] = C.position[2];

  double intrinsics[9];
  intrinsics[0] = C.focal_length_x;
  intrinsics[1] = C.focal_length_y;
  intrinsics[2] = C.center_x;
  intrinsics[3] = C.center_y;
  intrinsics[4] = C.distortion_k1;
  intrinsics[5] = C.distortion_k2;
  intrinsics[6] = C.distortion_k3;
  intrinsics[7] = C.distortion_p1;
  intrinsics[8] = C.distortion_p2;
  ceres::Problem problem;
  // when points, extrinsics, and intrinsics are not perturbed, there should be very little re-projection error
  for (int j = 0; j < transformed_points.size(); ++j)
  {
    double ox = observations[j].image_loc_x;
    double oy = observations[j].image_loc_y;
    ceres::CostFunction* cost_function = CameraReprjErrorWithDistortion::Create(ox, oy);
    problem.AddResidualBlock(cost_function, NULL, extrinsics, intrinsics, transformed_points[j].pb);
    double residual[2];
    CameraReprjErrorWithDistortion CFC(ox, oy);
    CFC(extrinsics, intrinsics, transformed_points[j].pb, residual);
    // no reprojection error should be observed
    ASSERT_NEAR(0.0, residual[0], .1);
    ASSERT_NEAR(0.0, residual[1], .1);
  }

  // optimization should therefore not do anything either
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 1000;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // no changes due to optimization
  ASSERT_NEAR(C.angle_axis[0], extrinsics[0], .1);
  ASSERT_NEAR(C.angle_axis[1], extrinsics[1], .1);
  ASSERT_NEAR(C.angle_axis[2], extrinsics[2], .1);
  ASSERT_NEAR(C.position[0], extrinsics[3], .1);
  ASSERT_NEAR(C.position[1], extrinsics[4], .1);
  ASSERT_NEAR(C.position[2], extrinsics[5], .1);

  ASSERT_NEAR(C.focal_length_x, intrinsics[0], .1);
  ASSERT_NEAR(C.focal_length_y, intrinsics[1], .1);
  ASSERT_NEAR(C.center_x, intrinsics[2], .1);
  ASSERT_NEAR(C.center_y, intrinsics[3], .1);
  ASSERT_NEAR(C.distortion_k1, intrinsics[4], .1);
  ASSERT_NEAR(C.distortion_k2, intrinsics[5], .1);
  ASSERT_NEAR(C.distortion_k3, intrinsics[6], .1);
  ASSERT_NEAR(C.distortion_p1, intrinsics[7], .1);
  ASSERT_NEAR(C.distortion_p2, intrinsics[8], .1);
}

TEST(IndustrialExtrinsicCalCeresSuite, circle_cost)
{
  // create known transform to move created points to transformed points
  aa[0] = 2.7;
  aa[1] = 0.3;
  aa[2] = 0.1;
  p[0] = 0.2;
  p[1] = 0.4;
  p[2] = 0.5;

  // initialize the camera parameters
  C.angle_axis[0] = 0.0;
  C.angle_axis[1] = 0.0;
  C.angle_axis[2] = 0.0;
  C.position[0] = 0.0;
  C.position[1] = 0.0;
  C.position[2] = 0.0;
  C.focal_length_x = 525;
  C.focal_length_y = 525;
  C.center_x = 320;
  C.center_y = 240;
  C.distortion_k1 = 0.01;
  C.distortion_k2 = 0.02;
  C.distortion_k3 = 0.03;
  C.distortion_p1 = 0.01;
  C.distortion_p2 = 0.01;

  // transform points, and then project into image plane
  Point3d t_point;
  Observation o_point;
  for (int i = 0; i < created_points.size(); i++)
  {
    t_point = xformPoint(created_points.at(i), aa[0], aa[1], aa[2], p[0], p[1], p[2]);
    o_point = projectPoint(C, t_point);
    transformed_points.push_back(t_point);
    observations.push_back(o_point);
  }

  double extrinsics[6];
  extrinsics[0] = C.angle_axis[0];
  extrinsics[1] = C.angle_axis[1];
  extrinsics[2] = C.angle_axis[2];
  extrinsics[3] = C.position[0];
  extrinsics[4] = C.position[1];
  extrinsics[5] = C.position[2];

  double intrinsics[9];
  intrinsics[0] = C.focal_length_x;
  intrinsics[1] = C.focal_length_y;
  intrinsics[2] = C.center_x;
  intrinsics[3] = C.center_y;
  intrinsics[4] = C.distortion_k1;
  intrinsics[5] = C.distortion_k2;
  intrinsics[6] = C.distortion_k3;
  intrinsics[7] = C.distortion_p1;
  intrinsics[8] = C.distortion_p2;
  ceres::Problem problem;
  // when points, extrinsics, and intrinsics are not perturbed, there should be very little re-projection error
  for (int j = 0; j < transformed_points.size(); ++j)
  {
    double ox = observations[j].image_loc_x;
    double oy = observations[j].image_loc_y;
    ceres::CostFunction* cost_function = CameraReprjErrorWithDistortion::Create(ox, oy);
    problem.AddResidualBlock(cost_function, NULL, extrinsics, intrinsics, transformed_points[j].pb);
    double residual[2];
    CameraReprjErrorWithDistortion CFC(ox, oy);
    CFC(extrinsics, intrinsics, transformed_points[j].pb, residual);
    // no reprojection error should be observed
    ASSERT_NEAR(0.0, residual[0], .1);
    ASSERT_NEAR(0.0, residual[1], .1);
  }

  // optimization should therefore not do anything either
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 1000;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // no changes due to optimization
  ASSERT_NEAR(C.angle_axis[0], extrinsics[0], .1);
  ASSERT_NEAR(C.angle_axis[1], extrinsics[1], .1);
  ASSERT_NEAR(C.angle_axis[2], extrinsics[2], .1);
  ASSERT_NEAR(C.position[0], extrinsics[3], .1);
  ASSERT_NEAR(C.position[1], extrinsics[4], .1);
  ASSERT_NEAR(C.position[2], extrinsics[5], .1);

  ASSERT_NEAR(C.focal_length_x, intrinsics[0], .1);
  ASSERT_NEAR(C.focal_length_y, intrinsics[1], .1);
  ASSERT_NEAR(C.center_x, intrinsics[2], .1);
  ASSERT_NEAR(C.center_y, intrinsics[3], .1);
  ASSERT_NEAR(C.distortion_k1, intrinsics[4], .1);
  ASSERT_NEAR(C.distortion_k2, intrinsics[5], .1);
  ASSERT_NEAR(C.distortion_k3, intrinsics[6], .1);
  ASSERT_NEAR(C.distortion_p1, intrinsics[7], .1);
  ASSERT_NEAR(C.distortion_p2, intrinsics[8], .1);
}

Point3d xformPoint(Point3d& original_point, double& ax, double& ay, double& az, double& x, double& y, double& z)
{
  // std::cout<<"ange axis inputs ax, ay, az: "<<ax<<", "<<ay<<", "<<az<<std::endl;
  Eigen::Matrix3f m_ceres;
  Eigen::Matrix3f m_eigen;
  m_eigen = Eigen::AngleAxisf(ax, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(ay, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(az, Eigen::Vector3f::UnitZ());
  // std::cout<<"m_eigen : "<<std::endl<< m_eigen <<std::endl;
  double aa[3];  // angle axis
  double p[3];   // point rotated
  aa[0] = ax;
  aa[1] = ay;
  aa[2] = az;
  Eigen::Vector3f orig_point(original_point.pb[0], original_point.pb[1], original_point.pb[2]);
  // std::cout<<"within transformPoint, original point Vector3f x, y, z: "<<orig_point.x()<<", "<<orig_point.y()<<",
  // "<<orig_point.z()<<std::endl;

  double R[9];
  ceres::AngleAxisToRotationMatrix(aa, R);
  m_ceres << R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8];
  // std::cout<<"m_ceres : "<<std::endl<< m_ceres <<std::endl;
  Eigen::Vector3f rot_point = m_ceres * orig_point;
  // std::cout<<"within transformPoint, rotated point Vector3f x, y, z: "<<rot_point.x()<<", "<<rot_point.y()<<",
  // "<<rot_point.z()<<std::endl;
  double xp1 = rot_point.x() + x;  // point rotated and translated
  double yp1 = rot_point.y() + y;
  double zp1 = rot_point.z() + z;
  // std::cout<<"within transformPoint, original point double x, y, z: "<<xp1<<", "<<yp1<<", "<<zp1<<std::endl;
  Point3d t_point;
  t_point.pb[0] = xp1;
  t_point.pb[1] = yp1;
  t_point.pb[2] = zp1;
  // std::cout<<"within transformPoint, t_point x, y, z: "<<t_point.pb[0]<<", "<<t_point.pb[1]<<",
  // "<<t_point.pb[2]<<std::endl;
  return t_point;
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  // ros::init(argc, argv, "test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
