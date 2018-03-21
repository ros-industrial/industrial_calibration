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

#ifndef CERES_COSTS_UTILS_TEST_HPP_
#define CERES_COSTS_UTILS_TEST_HPP_

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <industrial_extrinsic_cal/basic_types.h>

namespace industrial_extrinsic_cal
{
/* local prototypes of helper functions */

/*! \brief print a quaternion plus position as a homogeneous transform
 *  \param qx quaternion x value
 *  \param qy quaternion y value
 *  \param qz quaternion z value
 *  \param qw quaternion w value
 *  \param tx position x value
 *  \param ty position y value
 *  \param tz position z value
 */
void printQTasH(double qx, double qy, double qz, double qw, double tx, double ty, double tz);

/*! \brief print an angle axis transform as a homogeneous transform
 *  \param x angle axis x value
 *  \param y angle axis y value
 *  \param z angle axis z value
 *  \param tx position x value
 *  \param ty position y value
 *  \param tz position z value
 */
void printAATasH(double ax, double ay, double az, double tx, double ty, double tz);

/*! \brief print angle axis to homogeneous transform inverse
 *  \param ax angle axis x value
 *  \param ay angle axis y value
 *  \param az angle axis z value
 *  \param tx position x value
 *  \param ty position y value
 *  \param tz position z value
 */
void printAATasHI(double ax, double ay, double az, double tx, double ty, double tz);

/*! \brief print angle axis as euler angles
 *  \param ax angle axis x value
 *  \param ay angle axis y value
 *  \param az angle axis z value
 */
void printAAasEuler(double ax, double ay, double az);

/*! \brief print Camera Parameters
 *  \param CameraParameters include intrinsic and extrinsic
 *  \param words to provide as a header
 */
void printCameraParameters(CameraParameters C, std::string words);

/*! \brief  computes image of point in cameras image plane
 *  \param  C both intrinsic and extrinsic camera parameters
 *  \param  P the point to be projected into image
 */
Observation projectPointWithDistortion(CameraParameters camera_parameters, Point3d point);
Observation projectPointNoDistortion(CameraParameters camera_params, Point3d point_to_project);

Observation projectPointWithDistortion(CameraParameters C, Point3d P)
{
  double p[3];
  double pt[3];
  pt[0] = P.x;
  pt[1] = P.y;
  pt[2] = P.z;

  /* transform point into camera frame */
  /* note, camera transform takes points from camera frame into world frame */
  double aa[3];
  aa[0] = C.pb_extrinsics[0];
  aa[1] = C.pb_extrinsics[1];
  aa[2] = C.pb_extrinsics[2];
  ceres::AngleAxisRotatePoint(aa, pt, p);

  // apply camera translation
  double xp1 = p[0] + C.pb_extrinsics[3];
  double yp1 = p[1] + C.pb_extrinsics[4];
  double zp1 = p[2] + C.pb_extrinsics[5];
  // p[0] +=C.pb_extrinsics[3];
  // p[1] +=C.pb_extrinsics[4];
  // p[2] +=C.pb_extrinsics[5];

  double xp = xp1 / zp1;
  double yp = yp1 / zp1;

  // calculate terms for polynomial distortion
  double r2 = xp * xp + yp * yp;
  double r4 = r2 * r2;
  double r6 = r2 * r4;

  double xp2 = xp * xp; /* temporary variables square of others */
  double yp2 = yp * yp;

  /* apply the distortion coefficients to refine pixel location */
  double xpp = xp + C.distortion_k1 * r2 * xp + C.distortion_k2 * r4 * xp + C.distortion_k3 * r6 * xp +
               C.distortion_p2 * (r2 + 2 * xp2) + 2 * C.distortion_p1 * xp * yp;
  double ypp = yp + C.distortion_k1 * r2 * yp + C.distortion_k2 * r4 * yp + C.distortion_k3 * r6 * yp +
               C.distortion_p1 * (r2 + 2 * yp2) + 2 * C.distortion_p2 * xp * yp;

  /* perform projection using focal length and camera center into image plane */
  Observation O;
  O.point_id = 0;
  O.image_loc_x = C.focal_length_x * xpp + C.center_x;
  O.image_loc_y = C.focal_length_y * ypp + C.center_y;
  return (O);
}

Observation projectPointNoDistortion(CameraParameters C, Point3d P)
{
  double p[3];                   // rotated into camera frame
  double point[3];               // world location of point
  double aa[3];                  // angle axis representation of camera transform
  double tx = C.position[0];     // location of origin in camera frame x
  double ty = C.position[1];     // location of origin in camera frame y
  double tz = C.position[2];     // location of origin in camera frame z
  double fx = C.focal_length_x;  // focal length x
  double fy = C.focal_length_y;  // focal length y
  double cx = C.center_x;        // optical center x
  double cy = C.center_y;        // optical center y

  aa[0] = C.angle_axis[0];
  aa[1] = C.angle_axis[1];
  aa[2] = C.angle_axis[2];
  point[0] = P.x;
  point[1] = P.y;
  point[2] = P.z;

  /** rotate and translate points into camera frame */
  ceres::AngleAxisRotatePoint(aa, point, p);

  // apply camera translation
  double xp1 = p[0] + tx;
  double yp1 = p[1] + ty;
  double zp1 = p[2] + tz;

  // scale into the image plane by distance away from camera
  double xp = xp1 / zp1;
  double yp = yp1 / zp1;

  // perform projection using focal length and camera center into image plane
  Observation O;
  O.image_loc_x = fx * xp + cx;
  O.image_loc_y = fy * yp + cy;
  return (O);
}

}  // end of namespace
#endif
