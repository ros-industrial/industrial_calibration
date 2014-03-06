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


#include <industrial_extrinsic_cal/ceres_costs_utils.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

#include <Eigen/Geometry>
#include <Eigen/Core>

using namespace industrial_extrinsic_cal;

Point3d transformPoint(Point3d &original_point, double &ax, double &ay, double &az, double &x, double&y, double &z);

std::vector<Point3d> created_points;
double aa[3]; // angle axis known/set
double p[3]; // point rotated known/set
std::vector<Point3d> transformed_points;

struct PointReprjErrorNoDistortion
{
  PointReprjErrorNoDistortion(double ob_x, double ob_y, double fx) :
      px_(ob_x), py_(ob_y), pz_(fx)
  {
  }

  template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    //const T* c_p2, /** intrinsic parameters */
                    const T* point, /** point being projected, yes this is has 3 parameters */
                    T* resid) const
    {
      /** extract the variables from the camera parameters */
      int q = 0; /** extrinsic block of parameters */
      const T& x = c_p1[0]; /**  angle_axis x for rotation of camera           */
      const T& y = c_p1[1]; /**  angle_axis y for rotation of camera */
      const T& z = c_p1[2]; /**  angle_axis z for rotation of camera */
      const T& tx = c_p1[3]; /**  translation of camera x */
      const T& ty = c_p1[4]; /**  translation of camera y */
      const T& tz = c_p1[5]; /**  translation of camera z */

      //std::cout<<"x, y, z: "<<c_p1[3]<<", "<<c_p1[4]<<", "<<c_p1[5]<<std::endl;

      //q = 0; /** intrinsic block of parameters */
      //const T& fx = c_p2[q++]; /**  focal length x */
      //const T& fy = c_p2[q++]; /**  focal length x */
      //const T& cx = c_p2[q++]; /**  center point x */
      //const T& cy = c_p2[q++]; /**  center point y */

      /** rotate and translate points into camera frame*/
      T aa[3]; /** angle axis*/
      T p[3]; /** point rotated*/
      aa[0] = x;
      aa[1] = y;
      aa[2] = z;
      ceres::AngleAxisRotatePoint(aa, point, p);

      /** apply camera translation*/
      T xp1 = p[0] + tx; /** point rotated and translated*/
      T yp1 = p[1] + ty;
      T zp1 = p[2] + tz;

      /** scale into the image plane by distance away from camera
      T xp = xp1 / zp1;
      T yp = yp1 / zp1;*/
      T xp=xp1;
      T yp=yp1;
      T zp=zp1;

      /** perform projection using focal length and camera center into image plane
      resid[0] = T(fx_) * xp + T(cx_) - T(ox_);
      resid[1] = T(fy_) * yp + T(cy_) - T(oy_);*/
      resid[0] = T(xp)-T(px_);
      resid[1] = T(yp)-T(py_);
      resid[2] = T(zp)-T(pz_);

      return true;
    } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double p_x, const double p_y, const double p_z)
  {
    return (new ceres::AutoDiffCostFunction<PointReprjErrorNoDistortion, 3, 6, 3>(new PointReprjErrorNoDistortion(p_x, p_y, p_z)));
  }
  double px_; /** observed x location of object in image */
  double py_; /** observed y location of object in image */
  double pz_; /*!< known focal length of camera in x */
};

TEST(IndustrialExtrinsicCalCeresSuite, create_points)
{
  Point3d x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15, x16, x17, x18, x19, x20;
  x1.pb[0]=1.0;x1.pb[1]=1.0;x1.pb[2]=1;
  created_points.push_back(x1);
  x2.pb[0]=0;x2.pb[1]=0;x2.pb[2]=0.01;
  created_points.push_back(x2);
  x3.pb[0]=0.65;x3.pb[1]=0.94;x3.pb[2]=0.01;
  created_points.push_back(x3);
  x4.pb[0]=0.2;x4.pb[1]=0.3;x4.pb[2]=0.01;
  created_points.push_back(x4);
  x5.pb[0]=0.372;x5.pb[1]=0.4;x5.pb[2]=0.4;
  created_points.push_back(x5);
  x6.pb[0]=0.3762;x6.pb[1]=0.8;x6.pb[2]=0.3;
  created_points.push_back(x6);
  x7.pb[0]=0.4;x7.pb[1]=0.389;x7.pb[2]=0.4;
  created_points.push_back(x7);
  x8.pb[0]=0.431;x8.pb[1]=0.7;x8.pb[2]=0.7;
  created_points.push_back(x8);
  x9.pb[0]=0.535;x9.pb[1]=0.9;x9.pb[2]=0.01;
  created_points.push_back(x9);
  x10.pb[0]=0.596;x10.pb[1]=1;x10.pb[2]=1.0;
  created_points.push_back(x10);
  x11.pb[0]=0.24;x11.pb[1]=1.0;x11.pb[2]=0.01;
  created_points.push_back(x11);
  x12.pb[0]=0.673;x12.pb[1]=1.7;x12.pb[2]=0.8;
  created_points.push_back(x12);
  x13.pb[0]=0.552;x13.pb[1]=1.15;x13.pb[2]=0.01;
  created_points.push_back(x13);
  x14.pb[0]=0.56;x14.pb[1]=0.81;x14.pb[2]=0.01;
  created_points.push_back(x14);
  x15.pb[0]=.70;x15.pb[1]=1.10;x15.pb[2]=0.01;
  created_points.push_back(x15);
  x16.pb[0]=0.3762;x16.pb[1]=0.02435;x16.pb[2]=0.3;
  created_points.push_back(x16);
  x17.pb[0]=0.0234;x17.pb[1]=0.389;x17.pb[2]=0.132;
  created_points.push_back(x17);
  x18.pb[0]=0.431;x18.pb[1]=0.245;x18.pb[2]=0.0235;
  created_points.push_back(x18);
  x19.pb[0]=0.535;x19.pb[1]=0.673;x19.pb[2]=0.01;
  created_points.push_back(x19);
  x20.pb[0]=0.76;x20.pb[1]=0.453;x20.pb[2]=1.0;
  created_points.push_back(x20);
  std::cout<<"Original Point 1: "<<x1.pb[0]<<" "<<x1.pb[1]<<" "<<x1.pb[2]<<std::endl;

  //create known transform
  aa[0] = 2.7;
  aa[1] = 0.3;
  aa[2] = 0.1;
  p[0]=0.2;
  p[1]=0.4;
  p[2]=0.5;
  Point3d t_point;
  for (int i=0; i<created_points.size();i++)
  {
    t_point=transformPoint(created_points.at(i), aa[0], aa[1], aa[2], p[0], p[1], p[2]);
    transformed_points.push_back(t_point);
  }

}

TEST(IndustrialExtrinsicCalCeresSuite, points_costfunction)
{
double extrinsics[6];
ceres::Problem problem;
  for (int j = 0; j < transformed_points.size(); ++j)
  {
    ceres::CostFunction* cost_function = PointReprjErrorNoDistortion::Create(transformed_points.at(j).pb[0], transformed_points.at(j).pb[1], transformed_points.at(j).pb[2]);

    problem.AddResidualBlock(cost_function, NULL, extrinsics, created_points[j].pb);
    //problem.SetParameterBlockConstant(C.PB_intrinsics);
    problem.SetParameterBlockConstant(created_points[j].pb);
  }
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 1000;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  //std::cout << summary.FullReport() << "\n";
  std::cout << summary.BriefReport() << "\n";

  EXPECT_FLOAT_EQ(-2.7,extrinsics[0]);
  EXPECT_FLOAT_EQ(-0.3,extrinsics[1]);
  EXPECT_FLOAT_EQ(-0.1,extrinsics[2]);
  EXPECT_FLOAT_EQ(0.2,extrinsics[3]);
  EXPECT_FLOAT_EQ(0.4,extrinsics[4]);
  EXPECT_FLOAT_EQ(0.5,extrinsics[5]);
  std::cout<<"Original aa rot and translation: "<<aa[0]<<" "<<aa[1]<<" "<<aa[2]
                                     <<" "<<p[0]<<" "<<p[1]<<" "<<p[2]<<std::endl;

  std::cout<<"Optimized Extrinsics rot/transl: "<<extrinsics[0]<<" "<<extrinsics[1]<<" "<<extrinsics[2]<<" "
      <<extrinsics[3]<<" "<<extrinsics[4]<<" "<<extrinsics[5]<<std::endl;
}

TEST(DISABLED_IndustrialExtrinsicCalCeresSuite, camera_costfunction)
//void test()//
{
  CameraParameters c_parameters;
  c_parameters.angle_axis[0]=0.0;//aa[0];//0.3;//
  c_parameters.angle_axis[1]=0.0;//aa[1];//0.7;//
  c_parameters.angle_axis[2]=0.0;//aa[2];//2.9;//
  c_parameters.position[0]=0.0;//p[0];//0.9;//
  c_parameters.position[1]=0.0;//p[1];//0.3;//
  c_parameters.position[2]=0.0;//p[2];//1.8;//
  c_parameters.focal_length_x=525;
  c_parameters.focal_length_y=525;
  c_parameters.center_x=320;
  c_parameters.center_y=240;
  c_parameters.distortion_k1=0.01;
  c_parameters.distortion_k2=0.02;
  c_parameters.distortion_k3=0.03;
  c_parameters.distortion_p1=0.01;
  c_parameters.distortion_p2=0.01;

  std::vector<Observation> projected_observations;
  Observation proj_point;
  for (int m=0; m<transformed_points.size(); m++)
  {
    proj_point=industrial_extrinsic_cal::projectPointWithDistortion(c_parameters, transformed_points.at(m));
    projected_observations.push_back(proj_point);
  }

  double extrinsics[6];
  extrinsics[0]=c_parameters.pb_extrinsics[0];//0.2;//
  extrinsics[1]=c_parameters.pb_extrinsics[1];//0.1;//
  extrinsics[2]=c_parameters.pb_extrinsics[2];//0.9;//
  extrinsics[3]=c_parameters.pb_extrinsics[3];//1.3;//
  extrinsics[4]=c_parameters.pb_extrinsics[4];//0.1;//
  extrinsics[5]=c_parameters.pb_extrinsics[5];//2.0;//
  ceres::Problem problem;
  double original_points[3];
    for (int j = 0; j < projected_observations.size(); ++j)
    {
      ceres::CostFunction* cost_function = CameraReprjErrorWithDistortion::Create(projected_observations.at(j).image_loc_x,
             projected_observations.at(j).image_loc_y);


      problem.AddResidualBlock(cost_function, NULL, extrinsics, c_parameters.pb_intrinsics, created_points[j].pb);
      problem.SetParameterBlockConstant(c_parameters.pb_intrinsics);
      problem.SetParameterBlockConstant(created_points[j].pb);
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 1000;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.FullReport() << "\n";
    std::cout << summary.BriefReport() << "\n";

    std::cout<<"Expected aa rot and translation: "<<aa[0]<<" "<<aa[1]<<" "<<aa[2]
                                       <<" "<<p[0]<<" "<<p[1]<<" "<<p[2]<<std::endl;

    std::cout<<"Optimized Extrinsics rot/transl: "<<extrinsics[0]<<" "<<extrinsics[1]<<" "
        <<extrinsics[2]<<" "<<extrinsics[3]<<" "<<extrinsics[4]<<" "
        <<extrinsics[5]<<std::endl;
}

//void test()
TEST(DISABLED_IndustrialExtrinsicCalCeresSuite, camera_no_dist_costfunction)
{
  CameraParameters c_parameters;
  c_parameters.angle_axis[0]=0.0;//aa[0];//0.3;//
  c_parameters.angle_axis[1]=0.0;//aa[1];//0.7;//
  c_parameters.angle_axis[2]=0.0;//aa[2];//2.9;//
  c_parameters.position[0]=0.0;//p[0];//0.9;//
  c_parameters.position[1]=0.0;//p[1];//0.3;//
  c_parameters.position[2]=0.0;//p[2];//1.8;//
  c_parameters.focal_length_x=525;
  c_parameters.focal_length_y=525;
  c_parameters.center_x=320;
  c_parameters.center_y=240;
  c_parameters.distortion_k1=0.01;
  c_parameters.distortion_k2=0.02;
  c_parameters.distortion_k3=0.03;
  c_parameters.distortion_p1=0.01;
  c_parameters.distortion_p2=0.01;

  std::vector<Observation> projected_observations;
  Observation proj_point;
  for (int m=0; m<transformed_points.size(); m++)
  {

    proj_point=industrial_extrinsic_cal::projectPointNoDistortion(c_parameters, transformed_points.at(m));
    projected_observations.push_back(proj_point);
  }

  double extrinsics[6];
  extrinsics[0]=c_parameters.pb_extrinsics[0];//0.2;//
  extrinsics[1]=c_parameters.pb_extrinsics[1];//0.1;//
  extrinsics[2]=c_parameters.pb_extrinsics[2];//0.9;//
  extrinsics[3]=c_parameters.pb_extrinsics[3];//1.3;//
  extrinsics[4]=c_parameters.pb_extrinsics[4];//0.1;//
  extrinsics[5]=c_parameters.pb_extrinsics[5];//2.0;//
  ceres::Problem problem;
    for (int j = 0; j < projected_observations.size(); ++j)
    {
      ceres::CostFunction* cost_function = CameraReprjErrorNoDistortion::Create(projected_observations.at(j).image_loc_x,
             projected_observations.at(j).image_loc_y, c_parameters.focal_length_x, c_parameters.focal_length_y,
             c_parameters.center_x, c_parameters.center_y);

      problem.AddResidualBlock(cost_function, NULL, extrinsics, c_parameters.pb_intrinsics, created_points.at(j).pb);
      problem.SetParameterBlockConstant(c_parameters.pb_intrinsics);
      problem.SetParameterBlockConstant(created_points.at(j).pb);
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 1000;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.FullReport() << "\n";
    std::cout << summary.BriefReport() << "\n";

    std::cout<<"Expected aa rot and translation: "<<aa[0]<<" "<<aa[1]<<" "<<aa[2]
                                       <<" "<<p[0]<<" "<<p[1]<<" "<<p[2]<<std::endl;

    std::cout<<"Optimized Extrinsics rot/transl: "<<extrinsics[0]<<" "<<extrinsics[1]<<" "
        <<extrinsics[2]<<" "<<extrinsics[3]<<" "<<extrinsics[4]<<" "
        <<extrinsics[5]<<std::endl;
}

Point3d transformPoint(Point3d &original_point, double &ax, double &ay, double &az, double &x, double&y, double &z)
{
  //std::cout<<"ange axis inputs ax, ay, az: "<<ax<<", "<<ay<<", "<<az<<std::endl;
  Eigen::Matrix3f m_ceres;
  Eigen::Matrix3f m_eigen;
  m_eigen = Eigen::AngleAxisf(ax, Eigen::Vector3f::UnitX())
  * Eigen::AngleAxisf(ay, Eigen::Vector3f::UnitY())
  * Eigen::AngleAxisf(az, Eigen::Vector3f::UnitZ());
  //std::cout<<"m_eigen : "<<std::endl<< m_eigen <<std::endl;
  double aa[3]; // angle axis
  double p[3]; // point rotated
  aa[0] = ax;
  aa[1] = ay;
  aa[2] = az;
  Eigen::Vector3f orig_point(original_point.pb[0], original_point.pb[1], original_point.pb[2]);
  //std::cout<<"within transformPoint, original point Vector3f x, y, z: "<<orig_point.x()<<", "<<orig_point.y()<<", "<<orig_point.z()<<std::endl;

  double R[9];
  ceres::AngleAxisToRotationMatrix(aa, R);
  m_ceres << R[0], R[1],R[2],
      R[3], R[4], R[5],
      R[6], R[7], R[8];
  //std::cout<<"m_ceres : "<<std::endl<< m_ceres <<std::endl;
  Eigen::Vector3f rot_point=m_ceres*orig_point;
  //std::cout<<"within transformPoint, rotated point Vector3f x, y, z: "<<rot_point.x()<<", "<<rot_point.y()<<", "<<rot_point.z()<<std::endl;
  double xp1 = rot_point.x() + x; // point rotated and translated
  double yp1 = rot_point.y() + y;
  double zp1 = rot_point.z() + z;
  //std::cout<<"within transformPoint, original point double x, y, z: "<<xp1<<", "<<yp1<<", "<<zp1<<std::endl;
  Point3d t_point;
  t_point.pb[0]=xp1;t_point.pb[1]=yp1;t_point.pb[2]=zp1;
  //std::cout<<"within transformPoint, t_point x, y, z: "<<t_point.pb[0]<<", "<<t_point.pb[1]<<", "<<t_point.pb[2]<<std::endl;
  return t_point;
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  //ros::init(argc, argv, "test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();

}
