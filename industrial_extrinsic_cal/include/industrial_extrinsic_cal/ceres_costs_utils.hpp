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

#ifndef CERES_COSTS_UTILS_HPP_
#define CERES_COSTS_UTILS_HPP_

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/ceres_costs_utils.h>

namespace industrial_extrinsic_cal
{
// HELPER TEMPLATES

/*! \brief ceres compliant template to compute product of two rotations
 *   @param R1 first rotation matrix in column major order
 *   @param R1 second rotation matrix in column major order
 *   @param R3 matrix product of R1*R2
 */
template <typename T>
void rotationProduct(const T R1[9], const T R2[9], T R3[9]);
template <typename T>
inline void rotationProduct(const T R1[9], const T R2[9], T R3[9])
{
  // We assume that the rotation matrices are in column major order
  // x column
  R3[0] = R1[0] * R2[0] + R1[3] * R2[1] + R1[6] * R2[2];
  R3[1] = R1[1] * R2[0] + R1[4] * R2[1] + R1[7] * R2[2];
  R3[2] = R1[2] * R2[0] + R1[5] * R2[1] + R1[8] * R2[2];
  // y column
  R3[3] = R1[0] * R2[3] + R1[3] * R2[4] + R1[6] * R2[5];
  R3[4] = R1[1] * R2[3] + R1[4] * R2[4] + R1[7] * R2[5];
  R3[5] = R1[2] * R2[3] + R1[5] * R2[4] + R1[8] * R2[5];
  // z column
  R3[6] = R1[0] * R2[6] + R1[3] * R2[7] + R1[6] * R2[8];
  R3[7] = R1[1] * R2[6] + R1[4] * R2[7] + R1[7] * R2[8];
  R3[8] = R1[2] * R2[6] + R1[5] * R2[7] + R1[8] * R2[8];
}

/*! \brief ceres compliant template to extract the camera parameters from an ambigious vector of parameters
 *   @param intrinsics[9] vector of parameters
 *   @param fx focal length in x
 *   @param fy focal length in y
 *   @param cx optical center in x
 *   @param cy optical center in y
 *   @param k1 radial distortion coefficient k1
 *   @param k2 radial distortion coefficient k2
 *   @param k3 radial distortion coefficient k3
 *   @param p1 tangential distortion coefficient p1
 *   @param p2 tangential distortion coefficient p2
 */

template <typename T>
void extractCameraIntrinsics(const T intrinsics[9], T& fx, T& fy, T& cx, T& cy, T& k1, T& k2, T& k3, T& p1, T& p2);
template <typename T>
inline void extractCameraIntrinsics(const T intrinsics[9], T& fx, T& fy, T& cx, T& cy, T& k1, T& k2, T& k3, T& p1,
                                    T& p2)
{
  fx = intrinsics[0]; /** focal length x */
  fy = intrinsics[1]; /** focal length y */
  cx = intrinsics[2]; /** central point x */
  cy = intrinsics[3]; /** central point y */
  k1 = intrinsics[4]; /** distortion k1  */
  k2 = intrinsics[5]; /** distortion k2  */
  k3 = intrinsics[6]; /** distortion k3  */
  p1 = intrinsics[7]; /** distortion p1  */
  p2 = intrinsics[8]; /** distortion p2  */
}

/*! \brief ceres compliant template to extract the camera parameters from an ambigious vector of parameters without
 * distortion
 *   @param intrinsics[9] vector of parameters
 *   @param fx focal length in x
 *   @param fy focal length in y
 *   @param cx optical center in x
 *   @param cy optical center in y
 */

template <typename T>
void extractCameraIntrinsics(const T intrinsics[4], T& fx, T& fy, T& cx, T& cy);
template <typename T>
inline void extractCameraIntrinsics(const T intrinsics[4], T& fx, T& fy, T& cx, T& cy)
{
  fx = intrinsics[0]; /** focal length x */
  fy = intrinsics[1]; /** focal length y */
  cx = intrinsics[2]; /** central point x */
  cy = intrinsics[3]; /** central point y */
}

/*! \brief ceres compliant template to extract the camera pose from an ambigious vector of parameters
 *   @param extrinsics[9] vector of parameters
 *   @param tx, position x
 *   @param ty, position y
 *   @param tz, position z
 *   @param ax, angle axis x
 *   @param ay, angle axis y
 *   @param az, angle axis z
 */
template <typename T>
void extractCameraExtrinsics(const T extrinsics[6], T& x, T& y, T& z, T& ax, T& ay, T& az);
template <typename T>
inline void extractCameraExtrinsics(const T extrinsics[6], T& x, T& y, T& z, T& ax, T& ay, T& az)
{
  ax = extrinsics[0]; /** angle axis x */
  ay = extrinsics[1]; /** angle axis y */
  az = extrinsics[2]; /** angle axis z */
  x = extrinsics[3];  /** position x */
  y = extrinsics[4];  /** position y */
  z = extrinsics[5];  /** position z */
}

/*! \brief ceres compliant to compute inverse of a rotation matrix
 *  @param R the input rotation
 *  @param RI the output inverse rotation
 */

template <typename T>
void rotationInverse(const T R[9], const T RI[9]);
template <typename T>
inline void rotationInverse(const T R[9], T RI[9])
{
  RI[0] = R[0];
  RI[3] = R[1];
  RI[6] = R[2];
  RI[1] = R[3];
  RI[4] = R[4];
  RI[7] = R[5];
  RI[2] = R[6];
  RI[5] = R[7];
  RI[8] = R[8];
}

/*! \brief ceres compliant function to apply an angle-axis and translation to transform a point
 *  @param angle_axis, ax, ay, and az
 *  @param tx translation tx, ty and tz
 *  @param point the original point
 *  @param t_point the transformed point
 */

template <typename T>
inline void transformPoint(const T angle_axis[3], const T tx[3], const T point[3], T t_point[3]);
template <typename T>
inline void transformPoint(const T angle_axis[3], const T tx[3], const T point[3], T t_point[3])
{
  ceres::AngleAxisRotatePoint(angle_axis, point, t_point);
  t_point[0] = t_point[0] + tx[0];
  t_point[1] = t_point[1] + tx[1];
  t_point[2] = t_point[2] + tx[2];
}

/*! \brief ceres compliant function to apply a pose to transform a point
 *  @param pose, contains both rotation and translation in a structure
 *  @param point the original point
 *  @param t_point the transformed point
 */

template <typename T>
inline void poseTransformPoint(const Pose6d& pose, const T point[3], T t_point[3]);
template <typename T>
inline void poseTransformPoint(const Pose6d& pose, const T point[3], T t_point[3])
{
  T angle_axis[3];
  angle_axis[0] = T(pose.ax);
  angle_axis[1] = T(pose.ay);
  angle_axis[2] = T(pose.az);
  ceres::AngleAxisRotatePoint(angle_axis, point, t_point);
  t_point[0] = t_point[0] + T(pose.x);
  t_point[1] = t_point[1] + T(pose.y);
  t_point[2] = t_point[2] + T(pose.z);
}

/*! \brief ceres compliant function to apply an angle-axis and translation to transform a point in Point3d form
 *  @param angle_axis, ax, ay, and az
 *  @param tx translation tx, ty and tz
 *  @param point the original point in a Point3d form
 *  @param t_point the transformed point
 */
template <typename T>
void transformPoint3d(const T angle_axis[3], const T tx[3], const Point3d& point, T t_point[3]);
template <typename T>
inline void transformPoint3d(const T angle_axis[3], const T tx[3], const Point3d& point, T t_point[3])
{
  T point_[3];
  point_[0] = T(point.x);
  point_[1] = T(point.y);
  point_[2] = T(point.z);
  ceres::AngleAxisRotatePoint(angle_axis, point_, t_point);
  t_point[0] = t_point[0] + tx[0];
  t_point[1] = t_point[1] + tx[1];
  t_point[2] = t_point[2] + tx[2];
}

/*! \brief ceres compliant function get a templated rotation from a Pose6d structure
 *  @param pose the input pose
 *  @param pose the output rotatation matrix
 */
template <typename T>
void poseRotationMatrix(const Pose6d& pose, T R[9]);
template <typename T>
inline void poseRotationMatrix(const Pose6d& pose, T R[9])
{
  T angle_axis[3];
  angle_axis[0] = T(pose.ax);
  angle_axis[1] = T(pose.ay);
  angle_axis[2] = T(pose.az);
  ceres::AngleAxisToRotationMatrix(angle_axis, R);
}

/*! \brief ceres compliant function to compute the residual from a distorted pinhole camera model
 *  @param point[3] the input point
 *  @param k1 radial distortion parameter k1
 *  @param k2 radial distortion parameter k2
 *  @param k3 radial distortion parameter k3
 *  @param p1 tangential distortion parameter p1
 *  @param p2 tangential distortion parameter p2
 *  @param fx focal length in x
 *  @param fy focal length in y
 *  @param cx optical center in x
 *  @param cy optical center in y
 *  @param ox observation in x
 *  @param oy observation in y
 *  @param residual the output or difference between where the point should appear given the parameters, and where it
 * was observed
 */
template <typename T>
void cameraPntResidualDist(T point[3], T& k1, T& k2, T& k3, T& p1, T& p2, T& fx, T& fy, T& cx, T& cy, T& ox, T& oy,
                           T residual[2]);
template <typename T>
inline void cameraPntResidualDist(T point[3], T& k1, T& k2, T& k3, T& p1, T& p2, T& fx, T& fy, T& cx, T& cy, T& ox,
                                  T& oy, T residual[2])
{
  T xp1 = point[0];
  T yp1 = point[1];
  T zp1 = point[2];

  /** scale into the image plane by distance away from camera */
  T xp;
  T yp;
  if (zp1 == T(0))
  {  // avoid divide by zero
    xp = xp1;
    yp = yp1;
  }
  else
  {
    xp = xp1 / zp1;
    yp = yp1 / zp1;
  }

  /* temporary variables for distortion model */
  T xp2 = xp * xp;  /* x^2 */
  T yp2 = yp * yp;  /* y^2 */
  T r2 = xp2 + yp2; /* r^2 radius squared */
  T r4 = r2 * r2;   /* r^4 */
  T r6 = r2 * r4;   /* r^6 */

  /* apply the distortion coefficients to refine pixel location */
  T xpp = xp + k1 * r2 * xp           // 2nd order term
          + k2 * r4 * xp              // 4th order term
          + k3 * r6 * xp              // 6th order term
          + p2 * (r2 + T(2.0) * xp2)  // tangential
          + p1 * xp * yp * T(2.0);    // other tangential term
  T ypp = yp + k1 * r2 * yp           // 2nd order term
          + k2 * r4 * yp              // 4th order term
          + k3 * r6 * yp              // 6th order term
          + p1 * (r2 + T(2.0) * yp2)  // tangential term
          + p2 * xp * yp * T(2.0);    // other tangential term

  /** perform projection using focal length and camera center into image plane */
  residual[0] = fx * xpp + cx - ox;
  residual[1] = fy * ypp + cy - oy;
}

/*! \brief ceres compliant function to compute projection of a point into the image plane of a pinhole camera without
 * distortion
 *  @param point[3] the input point
 *  @param fx focal length in x
 *  @param fy focal length in y
 *  @param cx optical center in x
 *  @param cy optical center in y
 *  @param ox observation in x
 *  @param oy observation in y
 */
template <typename T>
void projectPntNoDistortion(T point[3], T& fx, T& fy, T& cx, T& cy, T& ox, T& oy);
template <typename T>
inline void projectPntNoDistortion(T point[3], T& fx, T& fy, T& cx, T& cy, T& ox, T& oy)
{
  T xp1 = point[0];
  T yp1 = point[1];
  T zp1 = point[2];

  /** scale into the image plane by distance away from camera */
  T xp;
  T yp;
  if (zp1 == T(0))
  {  // avoid divide by zero
    xp = xp1;
    yp = yp1;
  }
  else
  {
    xp = xp1 / zp1;
    yp = yp1 / zp1;
  }

  /** perform projection using focal length and camera center into image plane */
  ox = fx * xp + cx;
  oy = fy * yp + cy;
}

/*! \brief ceres compliant function to compute the residual from a distorted pinhole camera model, in this case,
 *   the point is actually a planar circle with some diameter. There observation is not the projected center,
 *   but is offset due to the angle between the plane and the imaging plane
 *  @param point[3] the input point
 *  @param circle_diameter the diameter of the circle being observed
 *  @param k1 radial distortion parameter k1
 *  @param k2 radial distortion parameter k2
 *  @param k3 radial distortion parameter k3
 *  @param p1 tangential distortion parameter p1
 *  @param p2 tangential distortion parameter p2
 *  @param fx focal length in x
 *  @param fy focal length in y
 *  @param cx optical center in x
 *  @param cy optical center in y
 *  @param ox observation in x
 *  @param oy observation in y
 *  @param residual the output or difference between where the point should appear given the parameters, and where it
 * was observed
 */
template <typename T>
void cameraCircResidualDist(T point[3], T& circle_diameter, T R_TtoC[9], T& k1, T& k2, T& k3, T& p1, T& p2, T& fx,
                            T& fy, T& cx, T& cy, T& ox, T& oy, T residual[2]);
template <typename T>
inline void cameraCircResidualDist(T point[3], T& circle_diameter, T R_TtoC[9], T& k1, T& k2, T& k3, T& p1, T& p2,
                                   T& fx, T& fy, T& cx, T& cy, T& ox, T& oy, T residual[2])
{
  T xp1 = point[0];
  T yp1 = point[1];
  T zp1 = point[2];

  // Circle Delta is the difference between the projection of the center of the circle
  // and the center of the projected ellipse

  // The 3 columns of R_TtoC represent:
  // 1. the x-axis of the target in camera coordinates
  // 2. the y-axis of the target in camera coordinates
  // 3. the z-axis (normal of the target plane) in camera coordinates
  // NOTE: we assume target is an XY planar target (all points on target have nominal z=0)

  // Find projection of distance vector D = [xp1 yp1 zp1] on to plane of target
  T D_targetx = xp1 * R_TtoC[0] + yp1 * R_TtoC[1] + zp1 * R_TtoC[2];
  T D_targety = xp1 * R_TtoC[3] + yp1 * R_TtoC[4] + zp1 * R_TtoC[5];

  // projection of D onto target xy plane expressed in camera frame is given by
  // D_targetx * R_TtoC(1stcol) + D_targety * R_TtoC(2ndcol)

  // The vector of interest "Vperp" is orthogonal to this in the target xy plane
  // Vperp = -D_targety * R_TtoC(1stcol) + D_targetx * R_TtoC(2ndcol)
  // However we want Vperp to be in the direction with a negative z component

  T Vperp[3];
  Vperp[0] = -D_targety * R_TtoC[0] + D_targetx * R_TtoC[3];
  Vperp[1] = -D_targety * R_TtoC[1] + D_targetx * R_TtoC[4];
  Vperp[2] = -D_targety * R_TtoC[2] + D_targetx * R_TtoC[5];

  // Vector direction of Vperp is arbitrary, but need to specify direction closer to camera
  T mysign;
  if (Vperp[2] * Vperp[2] > T(0.0))
  {
    mysign = -abs(Vperp[2]) / Vperp[2];
  }
  else
  {
    mysign = T(1);
  }
  Vperp[0] = mysign * Vperp[0];
  Vperp[1] = mysign * Vperp[1];
  Vperp[2] = mysign * Vperp[2];

  /** scale into the image plane by distance away from camera */
  T xp = xp1 / zp1;
  T yp = yp1 / zp1;

  if (zp1 + Vperp[2] != 0.0)
  {  // adjust only focal plan not parallel to target's xy plane
    T Vpx = (xp1 + Vperp[0]) / (zp1 + Vperp[2]);
    T Vpy = (yp1 + Vperp[1]) / (zp1 + Vperp[2]);
    T Vnorm = sqrt(Vpx * Vpx + Vpy * Vpy);
    if (Vnorm != 0.0)
    {
      // find scale of motion
      // Delta = (r*sin(theta)/(D-rcos(theta)) - r*sin(theta)/(D+rcos(theta)))/2
      // where r is the radius of the circle being projected
      //       D is the distance between camera and circle center
      //       theta is the angle between D vector an target xy plane
      Vpx = Vpx / Vnorm;
      Vpy = Vpy / Vnorm;
      T D = sqrt(xp1 * xp1 + yp1 * yp1 + zp1 * zp1);
      T s_theta = (R_TtoC[6] * xp1 + R_TtoC[7] * yp1 + R_TtoC[8] * zp1) / D;
      T c_theta = sqrt(T(1.0) - s_theta * s_theta);
      T r = T(circle_diameter / 2.0);
      T Delta = r * s_theta * (T(1.0) / (D - r * c_theta) - T(1.0) / (D + r * c_theta)) / T(2.0);
      xp = xp + Delta * Vpx;
      yp = yp + Delta * Vpy;
    }
  }

  /* temporary variables for distortion model */
  T xp2 = xp * xp;  /* x^2 */
  T yp2 = yp * yp;  /* y^2 */
  T r2 = xp2 + yp2; /* r^2 radius squared */
  T r4 = r2 * r2;   /* r^4 */
  T r6 = r2 * r4;   /* r^6 */

  /* apply the distortion coefficients to refine pixel location */
  T xpp = xp + k1 * r2 * xp           // 2nd order term
          + k2 * r4 * xp              // 4th order term
          + k3 * r6 * xp              // 6th order term
          + p2 * (r2 + T(2.0) * xp2)  // tangential
          + p1 * xp * yp * T(2.0);    // other tangential term
  T ypp = yp + k1 * r2 * yp           // 2nd order term
          + k2 * r4 * yp              // 4th order term
          + k3 * r6 * yp              // 6th order term
          + p1 * (r2 + T(2.0) * yp2)  // tangential term
          + p2 * xp * yp * T(2.0);    // other tangential term

  /** perform projection using focal length and camera center into image plane */
  residual[0] = fx * xpp + cx - ox;
  residual[1] = fy * ypp + cy - oy;
}

/*! \brief ceres compliant function to compute the residual from a pinhole camera model without distortion, in this
 * case,
 *   the point is actually a planar circle with some diameter. There observation is not the projected center,
 *   but is offset due to the angle between the plane and the imaging plane
 *  @param point[3] the input point
 *  @param circle_diameter the diameter of the circle being observed
 *  @param fx focal length in x
 *  @param fy focal length in y
 *  @param cx optical center in x
 *  @param cy optical center in y
 *  @param ox observation in x
 *  @param oy observation in y
 *  @param residual the output or difference between where the point should appear given the parameters, and where it
 * was observed
 */
template <typename T>
void cameraCircResidual(T point[3], T& circle_diameter, T R_TtoC[9], T& fx, T& fy, T& cx, T& cy, T& ox, T& oy,
                        T residual[2]);
template <typename T>
inline void cameraCircResidual(T point[3], T& circle_diameter, T R_TtoC[9], T& fx, T& fy, T& cx, T& cy, T& ox, T& oy,
                               T residual[2])
{
  T xp1 = point[0];
  T yp1 = point[1];
  T zp1 = point[2];

  // Circle Delta is the difference between the projection of the center of the circle
  // and the center of the projected ellipse

  // find rotation from target coordinates into camera coordinates
  // The 3 columns represent:
  // 1. the x-axis of the target in camera coordinates
  // 2. the y-axis of the target in camera coordinates
  // 3. the z-axis (normal of the target plane) in camera coordinates
  // NOTE: we assume target is an XY planar target (all points on target have nominal z=0)

  // Find projection of distance vector D = [xp1 yp1 zp1] on to plane of target
  T D_targetx = xp1 * R_TtoC[0] + yp1 * R_TtoC[1] + zp1 * R_TtoC[2];
  T D_targety = xp1 * R_TtoC[3] + yp1 * R_TtoC[4] + zp1 * R_TtoC[5];

  // projection of D onto target xy plane expressed in camera frame is given by
  // D_targetx * R_TtoC(1stcol) + D_targety * R_TtoC(2ndcol)

  // The vector of interest "Vperp" is orthogonal to this in the target xy plane
  // Vperp = -D_targety * R_TtoC(1stcol) + D_targetx * R_TtoC(2ndcol)
  // However we want Vperp to be in the direction with a negative z component

  T Vperp[3];
  Vperp[0] = -D_targety * R_TtoC[0] + D_targetx * R_TtoC[3];
  Vperp[1] = -D_targety * R_TtoC[1] + D_targetx * R_TtoC[4];
  Vperp[2] = -D_targety * R_TtoC[2] + D_targetx * R_TtoC[5];

  // Vector direction of Vperp is arbitrary, but need to specify direction closer to camera
  T mysign;
  if (Vperp[2] * Vperp[2] > T(0.0))
  {
    mysign = -abs(Vperp[2]) / Vperp[2];
  }
  else
  {
    mysign = T(1);
  }
  Vperp[0] = mysign * Vperp[0];
  Vperp[1] = mysign * Vperp[1];
  Vperp[2] = mysign * Vperp[2];

  /** scale into the image plane by distance away from camera */
  T xp = xp1 / zp1;
  T yp = yp1 / zp1;

  if (zp1 + Vperp[2] != 0.0)
  {  // adjust only focal plan not parallel to target's xy plane
    T Vpx = (xp1 + Vperp[0]) / (zp1 + Vperp[2]);
    T Vpy = (yp1 + Vperp[1]) / (zp1 + Vperp[2]);
    T Vnorm = sqrt(Vpx * Vpx + Vpy * Vpy);
    if (Vnorm != 0.0)
    {
      // find scale of motion
      // Delta = (r*sin(theta)/(D-rcos(theta)) - r*sin(theta)/(D+rcos(theta)))/2
      // where r is the radius of the circle being projected
      //       D is the distance between camera and circle center
      //       theta is the angle between D vector an target xy plane
      Vpx = Vpx / Vnorm;
      Vpy = Vpy / Vnorm;
      T D = sqrt(xp1 * xp1 + yp1 * yp1 + zp1 * zp1);
      T s_theta = (R_TtoC[6] * xp1 + R_TtoC[7] * yp1 + R_TtoC[8] * zp1) / D;
      T c_theta = sqrt(T(1.0) - s_theta * s_theta);
      T r = T(circle_diameter / 2.0);
      T Delta = r * s_theta * (T(1.0) / (D - r * c_theta) - T(1.0) / (D + r * c_theta)) / T(2.0);
      xp = xp + Delta * Vpx;
      yp = yp + Delta * Vpy;
    }
  }

  /** perform projection using focal length and camera center into image plane */
  residual[0] = fx * xp + cx - ox;
  residual[1] = fy * yp + cy - oy;
}

/*! \brief ceres compliant function to compute the residual from a pinhole camera model without distortion
 *  @param point[3] the input point
 *  @param fx focal length in x
 *  @param fy focal length in y
 *  @param cx optical center in x
 *  @param cy optical center in y
 *  @param ox observation in x
 *  @param oy observation in y
 *  @param residual the output or difference between where the point should appear given the parameters, and where it
 * was observed
 */

template <typename T>
void cameraPntResidual(T point[3], T& fx, T& fy, T& cx, T& cy, T& ox, T& oy, T residual[2]);
template <typename T>
inline void cameraPntResidual(T point[3], T& fx, T& fy, T& cx, T& cy, T& ox, T& oy, T residual[2])
{
  T xp1 = point[0];
  T yp1 = point[1];
  T zp1 = point[2];

  /** scale into the image plane by distance away from camera */
  T xp, yp;
  if (zp1 == T(0))
  {  // avoid divide by zero
    xp = xp1;
    yp = yp1;
  }
  else
  {
    xp = xp1 / zp1;
    yp = yp1 / zp1;
  }
  /** perform projection using focal length and camera center into image plane */
  residual[0] = fx * xp + cx - ox;
  residual[1] = fy * yp + cy - oy;
}

class CameraReprjErrorWithDistortion
{
public:
  CameraReprjErrorWithDistortion(double ob_x, double ob_y) : ox_(ob_x), oy_(ob_y)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters */
                  const T* c_p2,       /** intrinsic parameters */
                  const T* point,      /** point being projected, has 3 parameters */
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;
    extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    T camera_point[3]; /** point in camera coordinates*/

    /** transform point into camera coordinates */
    transformPoint(camera_aa, camera_tx, point, camera_point);

    /** compute project point into image plane and compute residual */
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidualDist(camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y)
  {
    return (new ceres::AutoDiffCostFunction<CameraReprjErrorWithDistortion, 2, 6, 9, 3>(
        new CameraReprjErrorWithDistortion(o_x, o_y)));
  }
  double ox_; /** observed x location of object in image */
  double oy_; /** observed y location of object in image */
};

// reprojection error of a single simple point observed by a camera with lens distortion
// typically used for intrinsic calibration
// location of target is known, and fixed, point's location within target is also known
// both extrinsic and intrinsic parameters of camera are being computed
class CameraReprjErrorWithDistortionPK
{
public:
  CameraReprjErrorWithDistortionPK(double ob_x, double ob_y, Point3d point) : ox_(ob_x), oy_(ob_y), point_(point)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters */
                  const T* c_p2,       /** intrinsic parameters */
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;
    extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    T camera_point[3];

    /** transform point into camera coordinates */
    transformPoint3d(camera_aa, camera_tx, point_, camera_point);

    /** compute project point into image plane and compute residual */
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidualDist(camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, Point3d point)
  {
    return (new ceres::AutoDiffCostFunction<CameraReprjErrorWithDistortionPK, 2, 6, 9>(
        new CameraReprjErrorWithDistortionPK(o_x, o_y, point)));
  }
  double ox_;     /** observed x location of object in image */
  double oy_;     /** observed y location of object in image */
  Point3d point_; /*! location of point in target coordinates */
};

// reprojection error of a single simple point observed by a camera with NO lens distortion
// should subscribe to a rectified image when using the error function
class CameraReprjError
{
public:
  CameraReprjError(double ob_x, double ob_y, double fx, double fy, double cx, double cy)
    : ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters */
                  const T* point,      /** point being projected, yes this is has 3 parameters */
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    transformPoint(camera_aa, camera_tx, point, camera_point);

    /** compute project point into image plane and compute residual */
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double fx, const double fy,
                                     const double cx, const double cy)
  {
    return (new ceres::AutoDiffCostFunction<CameraReprjError, 2, 6, 3>(new CameraReprjError(o_x, o_y, fx, fy, cx, cy)));
  }
  double ox_; /** observed x location of object in image */
  double oy_; /** observed y location of object in image */
  double fx_; /*!< known focal length of camera in x */
  double fy_; /*!< known focal length of camera in y */
  double cx_; /*!< known optical center of camera in x */
  double cy_; /*!< known optical center of camera in y */
};

class TriangulationError
{
public:
  TriangulationError(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Pose6d camera_pose)
    : ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), camera_pose_(camera_pose)
  {
  }

  template <typename T>
  bool operator()(const T* point, /** point being projected, yes this is has 3 parameters */
                  T* residual) const
  {
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    poseTransformPoint(camera_pose_, point, camera_point);

    /** compute project point into image plane and compute residual */
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double fx, const double fy,
                                     const double cx, const double cy, const Pose6d camera_pose)
  {
    return (new ceres::AutoDiffCostFunction<TriangulationError, 2, 3>(
        new TriangulationError(o_x, o_y, fx, fy, cx, cy, camera_pose)));
  }
  double ox_;          /** observed x location of object in image */
  double oy_;          /** observed y location of object in image */
  double fx_;          /*!< known focal length of camera in x */
  double fy_;          /*!< known focal length of camera in y */
  double cx_;          /*!< known optical center of camera in x */
  double cy_;          /*!< known optical center of camera in y */
  Pose6d camera_pose_; /*!< known camera pose */
};

// reprojection error of a single simple point observed by a camera with NO lens distortion
// should subscribe to a rectified image when using the error function
class CameraReprjErrorPK
{
public:
  CameraReprjErrorPK(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Point3d point)
    : ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), point_(point)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters */
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    transformPoint3d(camera_aa, camera_tx, point_, camera_point);

    /** compute project point into image plane and compute residual */
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double fx, const double fy,
                                     const double cx, const double cy, const Point3d point)
  {
    return (new ceres::AutoDiffCostFunction<CameraReprjErrorPK, 2, 6>(
        new CameraReprjErrorPK(o_x, o_y, fx, fy, cx, cy, point)));
  }
  double ox_;     /** observed x location of object in image */
  double oy_;     /** observed y location of object in image */
  double fx_;     /*!< known focal length of camera in x */
  double fy_;     /*!< known focal length of camera in y */
  double cx_;     /*!< known optical center of camera in x */
  double cy_;     /*!< known optical center of camera in y */
  Point3d point_; /*! location of point in target coordinates */
};

// reprojection error of a single point attatched to a target observed by a camera with NO lens distortion
// should subscribe to a rectified image when using the error function
//
class TargetCameraReprjError
{
public:
  TargetCameraReprjError(double ob_x, double ob_y, double fx, double fy, double cx, double cy)
    : ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1,  /** extrinsic parameters */
                  const T* const c_p2,  /** 6Dof transform of target points into world frame */
                  const T* const point, /** point described in target frame */
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    const T* target_aa(&c_p2[0]);
    const T* target_tx(&c_p2[3]);

    T world_point[3];  /** point in world coordinates */
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    transformPoint(target_aa, target_tx, point, world_point);
    transformPoint(camera_aa, camera_tx, world_point, camera_point);

    /** compute project point into image plane and compute residual */
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double fx, const double fy,
                                     const double cx, const double cy)

  {
    return (new ceres::AutoDiffCostFunction<TargetCameraReprjError, 2, 6, 6, 3>(
        new TargetCameraReprjError(o_x, o_y, fx, fy, cx, cy)));
  }
  double ox_; /** observed x location of object in image */
  double oy_; /** observed y location of object in image */
  double fx_; /*!< known focal length of camera in x */
  double fy_; /*!< known focal length of camera in y */
  double cx_; /*!< known optical center of camera in x */
  double cy_; /*!< known optical center of camera in y */
};

// reprojection error of a single point attatched to a target observed by a camera with NO lens distortion
// should subscribe to a rectified image when using the error function
//
class TargetCameraReprjErrorPK
{
public:
  TargetCameraReprjErrorPK(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Point3d point)
    : ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), point_(point)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters */
                  const T* const c_p2, /** 6Dof transform of target points into world frame */
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    const T* target_aa(&c_p2[0]);
    const T* target_tx(&c_p2[3]);
    T world_point[3];  /** point in world coordinates */
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    transformPoint3d(target_aa, target_tx, point_, world_point);
    transformPoint(camera_aa, camera_tx, world_point, camera_point);

    /** compute project point into image plane and compute residual */
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double fx, const double fy,
                                     const double cx, const double cy, const Point3d point)

  {
    return (new ceres::AutoDiffCostFunction<TargetCameraReprjErrorPK, 2, 6, 6>(
        new TargetCameraReprjErrorPK(o_x, o_y, fx, fy, cx, cy, point)));
  }
  double ox_;     /** observed x location of object in image */
  double oy_;     /** observed y location of object in image */
  double fx_;     /*!< known focal length of camera in x */
  double fy_;     /*!< known focal length of camera in y */
  double cx_;     /*!< known optical center of camera in x */
  double cy_;     /*!< known optical center of camera in y */
  Point3d point_; /*! location of point in target coordinates */
};

// reprojection error of a single point attatched to a target observed by a camera with NO lens distortion
// should subscribe to a rectified image when using the error function
//
class LinkTargetCameraReprjError
{
public:
  LinkTargetCameraReprjError(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Pose6d link_pose)
    : ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), link_pose_(link_pose)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1,  /** extrinsic parameters */
                  const T* const c_p2,  /** 6Dof transform of target points into world frame */
                  const T* const point, /** point described in target frame that is being seen */
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    const T* target_aa(&c_p2[0]);
    const T* target_tx(&c_p2[3]);
    T link_point[3];   /** point in link coordinates */
    T world_point[3];  /** point in world coordinates */
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    transformPoint(target_aa, target_tx, point, link_point);
    poseTransformPoint(link_pose_, link_point, world_point);
    transformPoint(camera_aa, camera_tx, world_point, camera_point);

    /** compute project point into image plane and compute residual */
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double ox, const double oy, const double fx, const double fy,
                                     const double cx, const double cy, Pose6d pose)

  {
    return (new ceres::AutoDiffCostFunction<LinkTargetCameraReprjError, 2, 6, 6, 3>(
        new LinkTargetCameraReprjError(ox, oy, fx, fy, cx, cy, pose)));
  }
  double ox_;        /** observed x location of object in image */
  double oy_;        /** observed y location of object in image */
  double fx_;        /*!< known focal length of camera in x */
  double fy_;        /*!< known focal length of camera in y */
  double cx_;        /*!< known optical center of camera in x */
  double cy_;        /*!< known optical center of camera in y */
  Pose6d link_pose_; /*!< transform from world to link coordinates */
};

// reprojection error of a single point attatched to a target observed by a camera with NO lens distortion
// should subscribe to a rectified image when using the error function
//
class LinkTargetCameraReprjErrorPK
{
public:
  LinkTargetCameraReprjErrorPK(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Pose6d link_pose,
                               Point3d point)
    : ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), link_pose_(link_pose), point_(point)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters */
                  const T* const c_p2, /** 6Dof transform of target points into world frame */
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    const T* target_aa(&c_p2[0]);
    const T* target_tx(&c_p2[3]);
    T link_point[3];   /** point in link coordinates */
    T world_point[3];  /** point in worls coordinates */
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    transformPoint3d(target_aa, target_tx, point_, link_point);
    poseTransformPoint(link_pose_, link_point, world_point);
    transformPoint(camera_aa, camera_tx, world_point, camera_point);

    /** compute project point into image plane and compute residual */
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double fx, const double fy,
                                     const double cx, const double cy, Pose6d pose, Point3d point)

  {
    return (new ceres::AutoDiffCostFunction<LinkTargetCameraReprjErrorPK, 2, 6, 6>(
        new LinkTargetCameraReprjErrorPK(o_x, o_y, fx, fy, cx, cy, pose, point)));
  }
  double ox_;        /** observed x location of object in image */
  double oy_;        /** observed y location of object in image */
  double fx_;        /*!< known focal length of camera in x */
  double fy_;        /*!< known focal length of camera in y */
  double cx_;        /*!< known optical center of camera in x */
  double cy_;        /*!< known optical center of camera in y */
  Pose6d link_pose_; /*!< transform from world to link coordinates */
  Point3d point_;    /*! location of point in target coordinates */
};

// reprojection error of a single point attatched to a target observed by a camera with NO lens distortion
// should subscribe to a rectified image when using the error function
//
class PosedTargetCameraReprjErrorPK
{
public:
  PosedTargetCameraReprjErrorPK(double ob_x, double ob_y, double fx, double fy, double cx, double cy,
                                Pose6d target_pose, Point3d point)
    : ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), target_pose_(target_pose), point_(point)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters */
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    T world_point[3];  /** point in worls coordinates */
    T camera_point[3]; /** point in camera coordinates */
    T point[3];
    point[0] = T(point_.x);
    point[1] = T(point_.y);
    point[2] = T(point_.z);
    /** transform point into camera coordinates */
    poseTransformPoint(target_pose_, point, world_point);
    transformPoint(camera_aa, camera_tx, world_point, camera_point);

    /** compute project point into image plane and compute residual */
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double fx, const double fy,
                                     const double cx, const double cy, Pose6d target_pose, Point3d point)

  {
    return (new ceres::AutoDiffCostFunction<PosedTargetCameraReprjErrorPK, 2, 6>(
        new PosedTargetCameraReprjErrorPK(o_x, o_y, fx, fy, cx, cy, target_pose, point)));
  }
  double ox_;          /** observed x location of object in image */
  double oy_;          /** observed y location of object in image */
  double fx_;          /*!< known focal length of camera in x */
  double fy_;          /*!< known focal length of camera in y */
  double cx_;          /*!< known optical center of camera in x */
  double cy_;          /*!< known optical center of camera in y */
  Pose6d target_pose_; /*!< transform from world to target coordinates */
  Point3d point_;      /*! location of point in target coordinates */
};

// reprojection error of a single point attatched to a target observed by a camera with NO lens distortion
// should subscribe to a rectified image when using the error function
//
class LinkCameraTargetReprjError
{
public:
  LinkCameraTargetReprjError(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Pose6d link_pose)
    : ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), link_pose_(link_pose)
  {
    link_posei_ = link_pose_.getInverse();
  }

  template <typename T>
  bool operator()(const T* const c_p1,  /** extrinsic parameters */
                  const T* const c_p2,  /** 6Dof transform of target points into world frame */
                  const T* const point, /** point described in target frame that is being seen */
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    const T* target_aa(&c_p2[0]);
    const T* target_tx(&c_p2[3]);
    T world_point[3];  /** point in world coordinates */
    T link_point[3];   /** point in link coordinates */
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    transformPoint(target_aa, target_tx, point, world_point);
    poseTransformPoint(link_posei_, world_point, link_point);
    transformPoint(camera_aa, camera_tx, link_point, camera_point);

    /** compute project point into image plane and compute residual */
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double ox, const double oy, const double fx, const double fy,
                                     const double cx, const double cy, Pose6d pose)

  {
    return (new ceres::AutoDiffCostFunction<LinkCameraTargetReprjError, 2, 6, 6, 3>(
        new LinkCameraTargetReprjError(ox, oy, fx, fy, cx, cy, pose)));
  }
  double ox_;         /*!< observed x location of object in image */
  double oy_;         /*!< observed y location of object in image */
  double fx_;         /*!< known focal length of camera in x */
  double fy_;         /*!< known focal length of camera in y */
  double cx_;         /*!< known optical center of camera in x */
  double cy_;         /*!< known optical center of camera in y */
  Pose6d link_pose_;  /*!< transform from world to link coordinates */
  Pose6d link_posei_; /*!< transform from link to world coordinates */
};

// reprojection error of a single point attatched to a target observed by a camera with NO lens distortion
// should subscribe to a rectified image when using the error function
//
class LinkCameraTargetReprjErrorPK
{
public:
  LinkCameraTargetReprjErrorPK(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Pose6d link_pose,
                               Point3d point)
    : ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), link_pose_(link_pose), point_(point)
  {
    link_posei_ = link_pose_.getInverse();
  }

  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters */
                  const T* const c_p2, /** 6Dof transform of target points into world frame */
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    const T* target_aa(&c_p2[0]);
    const T* target_tx(&c_p2[3]);
    T world_point[3];  /** point in world coordinates */
    T link_point[3];   /** point in link coordinates */
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    transformPoint3d(target_aa, target_tx, point_, world_point);
    poseTransformPoint(link_posei_, world_point, link_point);
    transformPoint(camera_aa, camera_tx, link_point, camera_point);

    /** compute project point into image plane and compute residual */
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double fx, const double fy,
                                     const double cx, const double cy, Pose6d pose, Point3d pnt)

  {
    return (new ceres::AutoDiffCostFunction<LinkCameraTargetReprjErrorPK, 2, 6, 6>(
        new LinkCameraTargetReprjErrorPK(o_x, o_y, fx, fy, cx, cy, pose, pnt)));
  }
  double ox_;         /** observed x location of object in image */
  double oy_;         /** observed y location of object in image */
  double fx_;         /*!< known focal length of camera in x */
  double fy_;         /*!< known focal length of camera in y */
  double cx_;         /*!< known optical center of camera in x */
  double cy_;         /*!< known optical center of camera in y */
  Pose6d link_pose_;  /*!< transform from world to link coordinates */
  Pose6d link_posei_; /*!< transform from link to world coordinates */
  Point3d point_;     /*! location of point in target coordinates */
};

// WARNING, ASSUMES CIRCLE LIES IN XY PLANE OF WORLD
class CircleCameraReprjErrorWithDistortion
{
public:
  CircleCameraReprjErrorWithDistortion(double ob_x, double ob_y, double c_dia)
    : ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1,  /** extrinsic parameters [6]*/
                  const T* const c_p2,  /** intrinsic parameters of camera fx,fy,cx,cy,k1,k2,k2,p1,p2 [9]*/
                  const T* const point, /** point described in target frame that is being seen [3]*/
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;
    extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    T camera_point[3]; /** point in camera coordinates*/
    T R_TtoC[9];

    /** transform point into camera coordinates */
    transformPoint(camera_aa, camera_tx, point, camera_point);

    // find rotation from target to camera frame
    ceres::AngleAxisToRotationMatrix(camera_aa, R_TtoC);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidualDist(camera_point, circle_diameter, R_TtoC, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia)
  {
    return (new ceres::AutoDiffCostFunction<CircleCameraReprjErrorWithDistortion, 2, 6, 9, 3>(
        new CircleCameraReprjErrorWithDistortion(o_x, o_y, c_dia)));
  }
  double ox_;               /** observed x location of object in image */
  double oy_;               /** observed y location of object in image */
  double circle_diameter_;  //** diameter of circle being observed */
};

// WARNING, ASSUMES CIRCLE LIES IN XY PLANE OF WORLD
class CircleCameraReprjErrorWithDistortionPK
{
public:
  CircleCameraReprjErrorWithDistortionPK(double ob_x, double ob_y, double c_dia, Point3d point)
    : ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), point_(point)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
                  const T* const c_p2, /** intrinsic parameters of camera fx,fy,cx,cy,k1,k2,k2,p1,p2 [9] */
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;
    extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    T camera_point[3]; /** point in camera coordinates */
    T R_TtoC[9];       /** rotation from target to camera coordinates
      
          /** find point in camera coordinates */
    transformPoint3d(camera_aa, camera_tx, point_, camera_point);

    // find rotation from target to camera coordinates
    T T2C_angle_axis[3];
    T2C_angle_axis[0] = T(-camera_aa[0]);
    T2C_angle_axis[1] = T(-camera_aa[1]);
    T2C_angle_axis[2] = T(-camera_aa[2]);
    ceres::AngleAxisToRotationMatrix(T2C_angle_axis, R_TtoC);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidualDist(camera_point, circle_diameter, R_TtoC, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, Point3d point)
  {
    return (new ceres::AutoDiffCostFunction<CircleCameraReprjErrorWithDistortionPK, 2, 6, 9>(
        new CircleCameraReprjErrorWithDistortionPK(o_x, o_y, c_dia, point)));
  }
  double ox_;               /** observed x location of object in image */
  double oy_;               /** observed y location of object in image */
  double circle_diameter_;  //** diameter of circle being observed */
  Point3d point_;
};

class CircleCameraReprjError
{
public:
  CircleCameraReprjError(double ob_x, double ob_y, double c_dia, double fx, double fy, double cx, double cy)
    : ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), fx_(fx), fy_(fy), cx_(cx), cy_(cy)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1,  /** extrinsic parameters [6]*/
                  const T* const point, /** point described in target frame that is being seen [3]*/
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    T camera_point[3]; /** point in camera coordinates */
    T R_TtoC[9];       /** rotation from target to camera coordinates */

    /** transform point into camera coordinates */
    transformPoint(camera_aa, camera_tx, point, camera_point);

    // find rotation from target to camera coordinates
    ceres::AngleAxisToRotationMatrix(camera_aa, R_TtoC);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidual(camera_point, circle_diameter, R_TtoC, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, const double fx,
                                     const double fy, const double cx, const double cy)
  {
    return (new ceres::AutoDiffCostFunction<CircleCameraReprjError, 2, 6, 3>(
        new CircleCameraReprjError(o_x, o_y, c_dia, fx, fy, cx, cy)));
  }
  double ox_;              /** observed x location of object in image */
  double oy_;              /** observed y location of object in image */
  double circle_diameter_; /** diameter of circle being observed */
  double fx_;              /** focal length of camera in x (pixels) */
  double fy_;              /** focal length of camera in y (pixels) */
  double cx_;              /** focal center of camera in x (pixels) */
  double cy_;              /** focal center of camera in y (pixels) */
};

class CircleCameraReprjErrorPK
{
public:
  CircleCameraReprjErrorPK(double ob_x, double ob_y, double c_dia, double fx, double fy, double cx, double cy,
                           Point3d point)
    : ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), fx_(fx), fy_(fy), cx_(cx), cy_(cy), point_(point)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    T camera_point[3]; /* point in camera coordinates */
    T R_TtoC[9];       /** rotation from target to camera coordinates */

    /** rotate and translate point into camera coordinates*/
    transformPoint3d(camera_aa, camera_tx, point_, camera_point);

    // find rotation from target to camera coordinates
    ceres::AngleAxisToRotationMatrix(camera_aa, R_TtoC);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidual(camera_point, circle_diameter, R_TtoC, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, const double fx,
                                     const double fy, const double cx, const double cy, Point3d point)
  {
    return (new ceres::AutoDiffCostFunction<CircleCameraReprjErrorPK, 2, 6>(
        new CircleCameraReprjErrorPK(o_x, o_y, c_dia, fx, fy, cx, cy, point)));
  }
  double ox_;               /** observed x location of object in image */
  double oy_;               /** observed y location of object in image */
  double circle_diameter_;  //** diameter of circle being observed */
  double fx_;               /** focal length of camera in x (pixels) */
  double fy_;               /** focal length of camera in y (pixels) */
  double cx_;               /** focal center of camera in x (pixels) */
  double cy_;               /** focal center of camera in y (pixels) */
  Point3d point_;           /** location of point in target coordinates */
};

class FixedCircleTargetCameraReprjErrorWithDistortion
{
public:
  FixedCircleTargetCameraReprjErrorWithDistortion(double ob_x, double ob_y, double c_dia)
    : ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1,  /** extrinsic parameters [6]*/
                  const T* const c_p2,  /** intrinsic parameters of camera fx,fy,cx,cy,k1,k2,k2,p1,p2 [9]*/
                  const T* const c_p3,  /** 6Dof transform of target into world frame [6]*/
                  const T* const point, /** point described in target frame that is being seen [3]*/
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    const T* target_aa(&c_p3[0]);
    const T* target_tx(&c_p3[3]);
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;
    extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    T world_point[3];  /** point in world coordinates */
    T camera_point[3]; /** point in camera coordinates*/
    T R_WtoC[9];       // rotation from world to camera coordinates
    T R_TtoW[9];       // rotation from target to world coordinates
    T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

    /** compute necessary rotation matrices */
    ceres::AngleAxisToRotationMatrix(camera_aa, R_WtoC);
    ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);

    /** transform point into camera coordinates */
    transformPoint(target_aa, target_tx, point, world_point);
    transformPoint(camera_aa, camera_tx, world_point, camera_point);

    // find rotation from target to camera coordinates
    rotationProduct(R_WtoC, R_TtoW, R_TtoC);  // R_WtoC*R_TtoW = R_TtoC

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidualDist(camera_point, circle_diameter, R_TtoC, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia)
  {
    return (new ceres::AutoDiffCostFunction<FixedCircleTargetCameraReprjErrorWithDistortion, 2, 6, 6, 9, 3>(
        new FixedCircleTargetCameraReprjErrorWithDistortion(o_x, o_y, c_dia)));
  }
  double ox_;               /** observed x location of object in image */
  double oy_;               /** observed y location of object in image */
  double circle_diameter_;  //** diameter of circle being observed */
};

class FixedCircleTargetCameraReprjErrorWithDistortionPK
{
public:
  FixedCircleTargetCameraReprjErrorWithDistortionPK(double ob_x, double ob_y, double c_dia, Point3d point)
    : ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), point_(point)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters [6]*/
                  const T* const c_p2, /** intrinsic parameters of camera fx,fy,cx,cy,k1,k2,k2,p1,p2 [9]*/
                  const T* const c_p3, /** 6Dof transform of target into world frame [6]*/
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    const T* target_aa(&c_p3[0]);
    const T* target_tx(&c_p3[3]);
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;
    extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    T world_point[3];  /** point in world coordinates */
    T camera_point[3]; /** point in camera coordinates*/
    T R_WtoC[9];       // rotation from world to camera coordinates
    T R_TtoW[9];       // rotation from target to world coordinates
    T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

    /** compute necessary rotation matrices */
    ceres::AngleAxisToRotationMatrix(camera_aa, R_WtoC);
    ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);

    /** transform point into camera coordinates */
    transformPoint3d(target_aa, target_tx, point_, world_point);
    transformPoint(camera_aa, camera_tx, world_point, camera_point);

    // find rotation from target to camera coordinates
    rotationProduct(R_WtoC, R_TtoW, R_TtoC);  // R_WtoC*R_TtoW = R_TtoC

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidualDist(camera_point, circle_diameter, R_TtoC, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, Point3d point)
  {
    return (new ceres::AutoDiffCostFunction<FixedCircleTargetCameraReprjErrorWithDistortionPK, 2, 6, 9, 3>(
        new FixedCircleTargetCameraReprjErrorWithDistortionPK(o_x, o_y, c_dia, point)));
  }
  double ox_;              /** observed x location of object in image */
  double oy_;              /** observed y location of object in image */
  double circle_diameter_; /** diameter of circle being observed */
  Point3d point_;          /** location of point in target coordinates */
};

class CircleTargetCameraReprjErrorWithDistortion
{
public:
  CircleTargetCameraReprjErrorWithDistortion(double ob_x, double ob_y, double c_dia)
    : ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1,  /** extrinsic parameters [6]*/
                  const T* const c_p2,  /** intrinsic parameters of camera fx,fy,cx,cy,k1,k2,k2,p1,p2 [9]*/
                  const T* const point, /** point described in target frame that is being seen [3]*/
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;
    extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    T world_point[3];  /** point in world coordinates */
    T camera_point[3]; /** point in camera coordinates*/
    T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

    /** compute necessary rotation matrices */
    ceres::AngleAxisToRotationMatrix(camera_aa, R_TtoC);

    /** transform point into camera coordinates */
    transformPoint(camera_aa, camera_tx, point, camera_point);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidualDist(camera_point, circle_diameter, R_TtoC, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia)
  {
    return (new ceres::AutoDiffCostFunction<CircleTargetCameraReprjErrorWithDistortion, 2, 6, 9, 3>(
        new CircleTargetCameraReprjErrorWithDistortion(o_x, o_y, c_dia)));
  }
  double ox_;               /** observed x location of object in image */
  double oy_;               /** observed y location of object in image */
  double circle_diameter_;  //** diameter of circle being observed */
};

// circle target is the origin, camera's location not known, distortion also unknown
class SimpleCircleTargetCameraReprjErrorWithDistortionPK
{
public:
  SimpleCircleTargetCameraReprjErrorWithDistortionPK(const double& ob_x, const double& ob_y, const double& c_dia,
                                                     const Point3d& point)
    : ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), point_(point)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
                  const T* const c_p2, /** intrinsic parameters[9] */
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;
    extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    T camera_point[3];        /** point in camera coordinates */
    T R_optical_to_target[9]; /** rotation from optical frame to target frame */
    T circle_diameter = T(circle_diameter_);
    T ox = T(ox_);
    T oy = T(oy_);
    T point[3];
    point[0] = T(point_.x);
    point[1] = T(point_.y);
    point[2] = T(point_.z);

    /** get necessary rotation matrices */
    ceres::AngleAxisToRotationMatrix(camera_aa, R_optical_to_target);

    /** transform point into camera coordinates */
    transformPoint(camera_aa, camera_tx, point, camera_point);

    /** compute project point into image plane and compute residual */
    cameraCircResidualDist(camera_point, circle_diameter, R_optical_to_target, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox,
                           oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double& o_x, const double& o_y, const double& c_dia, Point3d& point)
  {
    return (new ceres::AutoDiffCostFunction<SimpleCircleTargetCameraReprjErrorWithDistortionPK, 2, 6, 9>(
        new SimpleCircleTargetCameraReprjErrorWithDistortionPK(o_x, o_y, c_dia, point)));
  }
  double ox_;               /** observed x location of object in image */
  double oy_;               /** observed y location of object in image */
  double circle_diameter_;  //** diameter of circle being observed */
  Point3d point_;           /** point expressed in target coordinates */
};

class CircleTargetCameraReprjErrorWithDistortionPK
{
public:
  CircleTargetCameraReprjErrorWithDistortionPK(const double ob_x, const double ob_y, const double c_dia,
                                               const Point3d point)
    : ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), point_(point)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
                  const T* const c_p2, /** 6Dof transform of target into world frame [6] */
                  const T* const c_p3, /** intrinsic parameters of camera fx,fy,cx,cy,k1,k2,k2,p1,p2 [9] */
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    const T* target_aa(&c_p3[0]);
    const T* target_tx(&c_p3[3]);
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;
    extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    T world_point[3];  /** point in world coordinates */
    T camera_point[3]; /** point in world coordinates */
    T R_WtoC[9];       // rotation from world to camera coordinates
    T R_TtoW[9];       // rotation from target to world coordinates
    T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

    /** compute necessary rotation matrices */
    ceres::AngleAxisToRotationMatrix(camera_aa, R_WtoC);
    ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);

    /** transform point into camera coordinates */
    transformPoint3d(target_aa, target_tx, point_, world_point);
    transformPoint(camera_aa, camera_tx, world_point, camera_point);

    // find rotation from target to camera coordinates
    rotationProduct(R_WtoC, R_TtoW, R_TtoC);  // R_WtoC*R_TtoW = R_TtoC

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidualDist(camera_point, circle_diameter, R_TtoC, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, Point3d point)
  {
    return (new ceres::AutoDiffCostFunction<CircleTargetCameraReprjErrorWithDistortionPK, 2, 6, 6, 9>(
        new CircleTargetCameraReprjErrorWithDistortionPK(o_x, o_y, c_dia, point)));
  }
  double ox_;               /** observed x location of object in image */
  double oy_;               /** observed y location of object in image */
  double circle_diameter_;  //** diameter of circle being observed */
  Point3d point_;
};

class CircleTargetCameraReprjError
{
public:
  CircleTargetCameraReprjError(double ob_x, double ob_y, double c_dia, double fx, double fy, double cx, double cy)
    : ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), fx_(fx), fy_(fy)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1,  /** extrinsic parameters [6]*/
                  const T* const c_p2,  /** 6Dof transform of target into world frame [6]*/
                  const T* const point, /** point described in target frame that is being seen [3]*/
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    const T* target_aa(&c_p2[0]);
    const T* target_tx(&c_p2[3]);
    T world_point[3];  /** point in world coordinates */
    T camera_point[3]; /** point in camera coordinates*/
    T R_WtoC[9];       // rotation from world to camera coordinates
    T R_TtoW[9];       // rotation from target to world coordinates
    T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

    /** compute necessary rotation matrices */
    ceres::AngleAxisToRotationMatrix(camera_aa, R_WtoC);
    ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);

    /** transform point into camera coordinates */
    transformPoint(target_aa, target_tx, point, world_point);
    transformPoint(camera_aa, camera_tx, world_point, camera_point);

    /** find rotation from target to camera coordinates */
    rotationProduct(R_WtoC, R_TtoW, R_TtoC);  // R_WtoC*R_TtoW = R_TtoC

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidual(camera_point, circle_diameter, R_TtoC, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, const double fx,
                                     const double fy, const double cx, const double cy)
  {
    return (new ceres::AutoDiffCostFunction<CircleTargetCameraReprjError, 2, 6, 6, 3>(
        new CircleTargetCameraReprjError(o_x, o_y, c_dia, fx, fy, cx, cy)));
  }
  double ox_;               /** observed x location of object in image */
  double oy_;               /** observed y location of object in image */
  double circle_diameter_;  //** diameter of circle being observed */
  double fx_;               /** focal length of camera in x (pixels) */
  double fy_;               /** focal length of camera in y (pixels) */
  double cx_;               /** focal center of camera in x (pixels) */
  double cy_;               /** focal center of camera in y (pixels) */
};

class CircleTargetCameraReprjErrorPK
{
public:
  CircleTargetCameraReprjErrorPK(double ob_x, double ob_y, double c_dia, double fx, double fy, double cx, double cy,
                                 Point3d point)
    : ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), fx_(fx), fy_(fy), cx_(cx), cy_(cy), point_(point)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
                  const T* const c_p2, /** 6Dof transform of target into world frame [6] */
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    const T* target_aa(&c_p2[0]);
    const T* target_tx(&c_p2[3]);
    T world_point[3];  /** point in world coordinates */
    T camera_point[3]; /** point in camera coordinates */
    T R_WtoC[9];       // rotation from world to camera coordinates
    T R_TtoW[9];       // rotation from target to world coordinates
    T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

    /** compute necessary rotation matrices */
    ceres::AngleAxisToRotationMatrix(camera_aa, R_WtoC);
    ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);

    /** transform point into camera frame */
    transformPoint3d(target_aa, target_tx, point_, world_point);
    transformPoint(camera_aa, camera_tx, world_point, camera_point);

    /** find rotation from target to camera coordinates */
    rotationProduct(R_WtoC, R_TtoW, R_TtoC);  // R_WtoC*R_TtoW = R_TtoC

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidual(camera_point, circle_diameter, R_TtoC, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, const double fx,
                                     const double fy, const double cx, const double cy, Point3d point)
  {
    return (new ceres::AutoDiffCostFunction<CircleTargetCameraReprjErrorPK, 2, 6, 6>(
        new CircleTargetCameraReprjErrorPK(o_x, o_y, c_dia, fx, fy, cx, cy, point)));
  }
  double ox_;               /** observed x location of object in image */
  double oy_;               /** observed y location of object in image */
  double circle_diameter_;  //** diameter of circle being observed */
  double fx_;               /** focal length of camera in x (pixels) */
  double fy_;               /** focal length of camera in y (pixels) */
  double cx_;               /** focal center of camera in x (pixels) */
  double cy_;               /** focal center of camera in y (pixels) */
  Point3d point_;
};

class LinkCircleTargetCameraReprjError
{
public:
  LinkCircleTargetCameraReprjError(double ob_x, double ob_y, double c_dia, double fx, double fy, double cx, double cy,
                                   Pose6d link_pose)
    : ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), fx_(fx), fy_(fy), cx_(cx), cy_(cy), link_pose_(link_pose)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1,  /** extrinsic parameters [6]*/
                  const T* const c_p2,  /** 6Dof transform of target into world frame [6]*/
                  const T* const point, /** point described in target frame that is being seen [3]*/
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    const T* target_aa(&c_p2[0]);
    const T* target_tx(&c_p2[3]);
    T link_point[3];   /** point in link coordinates */
    T world_point[3];  /** point in world coordinates */
    T camera_point[3]; /** point in camera coordinates*/
    T R_WtoC[9];       // rotation from world to camera coordinates
    T R_TtoL[9];       // rotation from target to linkcoordinates
    T R_LtoW[9];       // rotation from link to world coordinates
    T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)
    T R_LtoC[9];  // rotation from link to camera coordinates

    /** get necessary rotation matrices */
    ceres::AngleAxisToRotationMatrix(camera_aa, R_WtoC);
    ceres::AngleAxisToRotationMatrix(target_aa, R_TtoL);
    poseRotationMatrix(link_pose_, R_LtoW);

    /** transform point into camera coordinates */
    transformPoint(target_aa, target_tx, point, link_point);
    poseTransformPoint(link_pose_, link_point, world_point);
    transformPoint(camera_aa, camera_tx, world_point, camera_point);

    /** find rotation from target to camera coordinates */
    rotationProduct(R_WtoC, R_LtoW, R_LtoC);
    rotationProduct(R_LtoC, R_TtoL, R_TtoC);  // R_WtoC*R_LtoW*R_TtoL = R_TtoC

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidual(camera_point, circle_diameter, R_TtoC, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, const double fx,
                                     const double fy, const double cx, const double cy, const Pose6d pose)
  {
    return (new ceres::AutoDiffCostFunction<LinkCircleTargetCameraReprjError, 2, 6, 6, 3>(
        new LinkCircleTargetCameraReprjError(o_x, o_y, c_dia, fx, fy, cx, cy, pose)));
  }
  double ox_;              /** observed x location of object in image */
  double oy_;              /** observed y location of object in image */
  double circle_diameter_; /** diameter of circle being observed */
  Pose6d link_pose_;       /** transform from link to world coordinates*/
  double fx_;              /** focal length of camera in x (pixels) */
  double fy_;              /** focal length of camera in y (pixels) */
  double cx_;              /** focal center of camera in x (pixels) */
  double cy_;              /** focal center of camera in y (pixels) */
};

class LinkCircleTargetCameraReprjErrorPK
{
public:
  LinkCircleTargetCameraReprjErrorPK(double ob_x, double ob_y, double c_dia, double fx, double fy, double cx, double cy,
                                     Pose6d link_pose, Point3d point)
    : ox_(ob_x)
    , oy_(ob_y)
    , circle_diameter_(c_dia)
    , fx_(fx)
    , fy_(fy)
    , cx_(cx)
    , cy_(cy)
    , link_pose_(link_pose)
    , point_(point)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
                  const T* const c_p2, /** 6Dof transform of target into world frame [6] */
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    const T* target_aa(&c_p2[0]);
    const T* target_tx(&c_p2[3]);
    T link_point[3];   /** point in link coordinates */
    T world_point[3];  /** point in world coordinates */
    T camera_point[3]; /** point in camera coordinates */
    T R_WtoC[9];       // rotation from world to camera coordinates
    T R_TtoL[9];       // rotation from target to linkcoordinates
    T R_LtoW[9];       // rotation from link to world coordinates
    T R_LtoC[9];       // rotation from link to camera coordinataes
    T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

    /** compute necessary rotation matrices */
    ceres::AngleAxisToRotationMatrix(camera_aa, R_WtoC);
    ceres::AngleAxisToRotationMatrix(target_aa, R_TtoL);
    poseRotationMatrix(link_pose_, R_LtoW);

    /** transform point into camera frame */
    transformPoint3d(target_aa, target_tx, point_, link_point);
    poseTransformPoint(link_pose_, link_point, world_point);
    transformPoint(camera_aa, camera_tx, world_point, camera_point);

    /** find rotation from target to camera coordinates */
    rotationProduct(R_WtoC, R_LtoW, R_LtoC);
    rotationProduct(R_LtoC, R_TtoL, R_TtoC);  // R_WtoC*R_LtoW*R_TtoL = R_TtoC

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidual(camera_point, circle_diameter, R_TtoC, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, const double fx,
                                     const double fy, const double cx, const double cy, const Pose6d pose,
                                     Point3d point)
  {
    return (new ceres::AutoDiffCostFunction<LinkCircleTargetCameraReprjErrorPK, 2, 6, 6>(
        new LinkCircleTargetCameraReprjErrorPK(o_x, o_y, c_dia, fx, fy, cx, cy, pose, point)));
  }
  double ox_;               /** observed x location of object in image */
  double oy_;               /** observed y location of object in image */
  double circle_diameter_;  //** diameter of circle being observed */
  Pose6d link_pose_;        /** transform from link to world coordinates*/
  double fx_;               /** focal length of camera in x (pixels) */
  double fy_;               /** focal length of camera in y (pixels) */
  double cx_;               /** focal center of camera in x (pixels) */
  double cy_;               /** focal center of camera in y (pixels) */
  Point3d point_;           /** point expressed in target coordinates */
};

class LinkCameraCircleTargetReprjError
{
public:
  LinkCameraCircleTargetReprjError(double ob_x, double ob_y, double c_dia, double fx, double fy, double cx, double cy,
                                   Pose6d link_pose)
    : ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), fx_(fx), fy_(fy), cx_(cx), cy_(cy), link_pose_(link_pose)
  {
    link_posei_ = link_pose_.getInverse();
  }

  template <typename T>
  bool operator()(const T* const c_p1,  /** extrinsic parameters [6]*/
                  const T* const c_p2,  /** 6Dof transform of target into world frame [6]*/
                  const T* const point, /** point described in target frame that is being seen [3]*/
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    const T* target_aa(&c_p2[0]);
    const T* target_tx(&c_p2[3]);
    T world_point[3];  /** point in world coordinates */
    T link_point[3];   /** point in link coordinates */
    T camera_point[3]; /** point in camera coordinates*/
    T R_LtoC[9];       // rotation from link to camera coordinates
    T R_WtoL[9];       // rotation from world to link coordinates
    T R_WtoC[9];       // rotation from world to camera coordinates, and intermediate transform
    T R_TtoW[9];       // rotation from target to world coordinates
    T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

    /** compute necessary rotation matrices */
    ceres::AngleAxisToRotationMatrix(camera_aa, R_LtoC);
    poseRotationMatrix(link_pose_, R_WtoL);
    ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);

    /** transform point into camera coordinates */
    transformPoint(target_aa, target_tx, point, world_point);
    poseTransformPoint(link_posei_, world_point, link_point);
    transformPoint(camera_aa, camera_tx, link_point, camera_point);

    /** find rotation from target to camera coordinates */
    rotationProduct(R_LtoC, R_WtoL, R_WtoC);
    rotationProduct(R_WtoC, R_TtoW, R_TtoC);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidual(camera_point, circle_diameter, R_TtoC, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, double fx, double fy,
                                     double cx, double cy, const Pose6d pose)
  {
    return (new ceres::AutoDiffCostFunction<LinkCameraCircleTargetReprjError, 2, 6, 6, 3>(
        new LinkCameraCircleTargetReprjError(o_x, o_y, c_dia, fx, fy, cx, cy, pose)));
  }
  double ox_;              /** observed x location of object in image */
  double oy_;              /** observed y location of object in image */
  double circle_diameter_; /** diameter of circle being observed */
  Pose6d link_pose_;       /** transform from link to world coordinates*/
  Pose6d link_posei_;      /** transform from world to link coordinates*/
  double fx_;              /** focal length of camera in x (pixels) */
  double fy_;              /** focal length of camera in y (pixels) */
  double cx_;              /** focal center of camera in x (pixels) */
  double cy_;              /** focal center of camera in y (pixels) */
};

class LinkCameraCircleTargetReprjErrorPK
{
public:
  LinkCameraCircleTargetReprjErrorPK(const double& ob_x, const double& ob_y, const double& c_dia, const double& fx,
                                     const double& fy, const double& cx, const double& cy, const Pose6d& link_pose,
                                     const Point3d& point)
    : ox_(ob_x)
    , oy_(ob_y)
    , circle_diameter_(c_dia)
    , fx_(fx)
    , fy_(fy)
    , cx_(cx)
    , cy_(cy)
    , link_pose_(link_pose)
    , point_(point)
  {
    link_posei_ = link_pose_.getInverse();
  }

  void test_residual(const double* c_p1, const double* c_p2, double* residual)
  {
    const double* camera_aa(&c_p1[0]);
    const double* camera_tx(&c_p1[3]);
    const double* target_aa(&c_p2[0]);
    const double* target_tx(&c_p2[3]);
    double point[3];        /** point in target coordinates */
    double world_point[3];  /** point in world coordinates */
    double link_point[3];   /** point in link coordinates */
    double camera_point[3]; /** point in camera coordinates*/
    double R_LtoC[9];       // rotation from link to camera coordinates
    double R_WtoL[9];       // rotation from world to link coordinates
    double R_WtoC[9];       // rotation from world to camera coordinates, and intermediate transform
    double R_TtoW[9];       // rotation from target to world coordinates
    double R_TtoC[9];       // rotation from target to camera coordinates (assume circle lies in x-y plane of target
                            // coordinates)
    /** get necessary rotation matrices */
    ceres::AngleAxisToRotationMatrix(camera_aa, R_LtoC);
    printf("camera_aa = %6.3lf %6.3lf %6.3lf\n", camera_aa[0], camera_aa[1], camera_aa[2]);
    printf("R_camera \n");
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        printf("%6.3lf", R_LtoC[i + j * 3]);
      }
      printf("\n");
    }
    poseRotationMatrix(link_posei_, R_WtoL);
    printf("R_inverse link\n");
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        printf("%6.3lf", R_WtoL[i + j * 3]);
      }
      printf("\n");
    }

    ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);
    printf("R_target\n");
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        printf("%6.3lf", R_TtoW[i + j * 3]);
      }
      printf("\n");
    }

    printf("point_ = %6.3lf  %6.3lf %6.3lf\n", point_.x, point_.y, point_.z);
    /** transform point into camera coordinates */
    transformPoint3d(target_aa, target_tx, point_, world_point);
    printf("world_point = %6.3lf  %6.3lf %6.3lf\n", world_point[0], world_point[1], world_point[2]);
    poseTransformPoint(link_posei_, world_point, link_point);
    printf("link_point = %6.3lf  %6.3lf %6.3lf\n", link_point[0], link_point[1], link_point[2]);
    transformPoint(camera_aa, camera_tx, link_point, camera_point);
    printf("camera_point = %6.3lf  %6.3lf %6.3lf\n", camera_point[0], camera_point[1], camera_point[2]);

    /** find rotation from target to camera coordinates */
    rotationProduct(R_LtoC, R_WtoL, R_WtoC);
    rotationProduct(R_WtoC, R_TtoW, R_TtoC);

    /** compute project point into image plane and compute residual */
    double circle_diameter = circle_diameter_;
    double fx = fx_;
    double fy = fy_;
    double cx = cx_;
    double cy = cy_;
    double ox = ox_;
    double oy = oy_;
    //      cameraCircResidual(camera_point, circle_diameter, R_TtoC, fx, fy,cx,cy, ox, oy, residual);
    cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy, residual);
  }
  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
                  const T* const c_p2, /** 6Dof transform of target into world frame [6] */
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    const T* target_aa(&c_p2[0]);
    const T* target_tx(&c_p2[3]);
    T point[3];        /** point in target coordinates */
    T world_point[3];  /** point in world coordinates */
    T link_point[3];   /** point in link coordinates */
    T camera_point[3]; /** point in camera coordinates*/
    T R_LtoC[9];       // rotation from link to camera coordinates
    T R_WtoL[9];       // rotation from world to link coordinates
    T R_WtoC[9];       // rotation from world to camera coordinates, and intermediate transform
    T R_TtoW[9];       // rotation from target to world coordinates
    T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

    /** get necessary rotation matrices */
    ceres::AngleAxisToRotationMatrix(camera_aa, R_LtoC);
    poseRotationMatrix(link_posei_, R_WtoL);
    ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);

    /** transform point into camera coordinates */
    transformPoint3d(target_aa, target_tx, point_, world_point);
    poseTransformPoint(link_posei_, world_point, link_point);
    transformPoint(camera_aa, camera_tx, link_point, camera_point);

    /** find rotation from target to camera coordinates */
    rotationProduct(R_LtoC, R_WtoL, R_WtoC);
    rotationProduct(R_WtoC, R_TtoW, R_TtoC);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidual(camera_point, circle_diameter, R_TtoC, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double& o_x, const double& o_y, const double& c_dia, const double& fx,
                                     const double& fy, const double& cx, const double& cy, const Pose6d& pose,
                                     Point3d& point)
  {
    return (new ceres::AutoDiffCostFunction<LinkCameraCircleTargetReprjErrorPK, 2, 6, 6>(
        new LinkCameraCircleTargetReprjErrorPK(o_x, o_y, c_dia, fx, fy, cx, cy, pose, point)));
  }
  double ox_;               /** observed x location of object in image */
  double oy_;               /** observed y location of object in image */
  double circle_diameter_;  //** diameter of circle being observed */
  Pose6d link_pose_;        /** transform from link to world coordinates*/
  Pose6d link_posei_;       /** transform from world to link coordinates*/
  double fx_;               /** focal length of camera in x (pixels) */
  double fy_;               /** focal length of camera in y (pixels) */
  double cx_;               /** focal center of camera in x (pixels) */
  double cy_;               /** focal center of camera in y (pixels) */
  Point3d point_;           /** point expressed in target coordinates */
};

class FixedCircleTargetCameraReprjErrorPK
{
public:
  FixedCircleTargetCameraReprjErrorPK(const double& ob_x, const double& ob_y, const double& c_dia, const double& fx,
                                      const double& fy, const double& cx, const double& cy, const Pose6d& target_pose,
                                      const Pose6d& camera_mounting_pose, const Point3d& point)
    : ox_(ob_x)
    , oy_(ob_y)
    , circle_diameter_(c_dia)
    , fx_(fx)
    , fy_(fy)
    , cx_(cx)
    , cy_(cy)
    , target_pose_(target_pose)
    , camera_mounting_pose_(camera_mounting_pose)
    , point_(point)
  {
    // compute pose which transforms point in target frame to the mounting frame
    mount_to_target_pose_ = camera_mounting_pose_.getInverse() * target_pose_;
  }

  void test_residual(const double* c_p1, double* resid)
  {
    const double* camera_aa(&c_p1[0]);
    const double* camera_tx(&c_p1[3]);
    double mount_point[3];  /** point in world coordinates */
    double camera_point[3]; /** point in camera coordinates */
    double point[3];        /** point in world coordinates  */
    point[0] = point_.x;
    point[1] = point_.y;
    point[2] = point_.z;
    /** transform point into camera coordinates */
    poseTransformPoint(mount_to_target_pose_, point, mount_point);
    printf("mount_point = %6.3lf  %6.3lf %6.3lf\n", mount_point[0], mount_point[1], mount_point[2]);
    transformPoint(camera_aa, camera_tx, mount_point, camera_point);
    printf("camera_point = %6.3lf  %6.3lf %6.3lf\n", camera_point[0], camera_point[1], camera_point[2]);

    double R_optical_to_mount[9]; /** rotation from optical to camera mounting frame */
    double R_mount_to_target[9];  /** rotation from mounting frame to target frame */

    /** get necessary rotation matrices */
    ceres::AngleAxisToRotationMatrix(camera_aa, R_optical_to_mount);
    poseRotationMatrix(mount_to_target_pose_, R_mount_to_target);

    /** compute project point into image plane and compute residual */
    double circle_diameter = circle_diameter_;
    double fx = fx_;
    double fy = fy_;
    double cx = cx_;
    double cy = cy_;
    double ox = ox_;
    double oy = oy_;
    cameraCircResidual(camera_point, circle_diameter, R_mount_to_target, fx, fy, cx, cy, ox, oy, resid);
  }
  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
                  T* residual) const
  {
    const T* camera_aa(&c_p1[0]);
    const T* camera_tx(&c_p1[3]);
    T mount_point[3];         /** point in world coordinates */
    T camera_point[3];        /** point in camera coordinates */
    T R_optical_to_mount[9];  /** rotation from optical to camera mounting frame */
    T R_mount_to_target[9];   /** rotation from mounting frame to target frame */
    T R_optical_to_target[9]; /** rotation from optical frame to target frame */
    T point[3];
    point[0] = T(point_.x);
    point[1] = T(point_.y);
    point[2] = T(point_.z);

    /** get necessary rotation matrices */
    ceres::AngleAxisToRotationMatrix(camera_aa, R_optical_to_mount);
    poseRotationMatrix(mount_to_target_pose_, R_mount_to_target);

    /** transform point into camera coordinates */
    poseTransformPoint(mount_to_target_pose_, point, mount_point);
    transformPoint(camera_aa, camera_tx, mount_point, camera_point);

    /** find rotation from target to camera coordinates */
    rotationProduct(R_optical_to_mount, R_mount_to_target, R_optical_to_target);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidual(camera_point, circle_diameter, R_optical_to_target, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double& o_x, const double& o_y, const double& c_dia, const double& fx,
                                     const double& fy, const double& cx, const double& cy, const Pose6d& target_pose,
                                     const Pose6d& camera_mounting_pose, Point3d& point)
  {
    return (new ceres::AutoDiffCostFunction<FixedCircleTargetCameraReprjErrorPK, 2, 6>(
        new FixedCircleTargetCameraReprjErrorPK(o_x, o_y, c_dia, fx, fy, cx, cy, target_pose, camera_mounting_pose,
                                                point)));
  }
  double ox_;                   /** observed x location of object in image */
  double oy_;                   /** observed y location of object in image */
  double circle_diameter_;      //** diameter of circle being observed */
  Pose6d target_pose_;          /** pose of target relative to the reference coordinate frame */
  Pose6d camera_mounting_pose_; /** pose camera mounting frame relative to the reference coordinate frame */
  Pose6d mount_to_target_pose_; /** pose of target frame relative to camera's mounting frame */
  double fx_;                   /** focal length of camera in x (pixels) */
  double fy_;                   /** focal length of camera in y (pixels) */
  double cx_;                   /** focal center of camera in x (pixels) */
  double cy_;                   /** focal center of camera in y (pixels) */
  Point3d point_;               /** point expressed in target coordinates */
};

class RailICal
{
public:
  RailICal(double ob_x, double ob_y, double rail_position, Point3d point)
    : ox_(ob_x), oy_(ob_y), rail_position_(rail_position), point_(point)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1, /**intrinsics [9] */
                  const T* const c_p2, /**target_pose [6] */
                  T* residual) const
  {
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;  // extract intrinsics
    extractCameraIntrinsics(c_p1, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    const T* target_aa(&c_p2[0]);  // extract target's angle axis
    const T* target_tx(&c_p2[3]);  // extract target's position

    /** transform point into camera frame */
    T camera_point[3]; /** point in camera coordinates */
    transformPoint3d(target_aa, target_tx, point_, camera_point);
    camera_point[2] = camera_point[2] + T(rail_position_);  // transform to camera's location along rail

    /** compute project point into image plane and compute residual */
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidualDist(camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double rail_position, Point3d point)
  {
    return (new ceres::AutoDiffCostFunction<RailICal, 2, 9, 6>(new RailICal(o_x, o_y, rail_position, point)));
  }
  double ox_;            /** observed x location of object in image */
  double oy_;            /** observed y location of object in image */
  double rail_position_; /** location of camera along rail */
  Point3d point_;        /** point expressed in target coordinates */
};

class  RailICal3
{
public:
  RailICal3(double ob_x, double ob_y, Point3d rail_position, Point3d point) :
    ox_(ob_x), oy_(ob_y), rail_position_(rail_position), point_(point)
  {
  }
  
  template<typename T>
  bool operator()(	    const T* const c_p1,  /**intrinsics [9] */
			    const T* const c_p2,  /**target_pose [6] */
			    T* residual) const
  {
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;      // extract intrinsics
    extractCameraIntrinsics(c_p1, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    const T *target_aa(& c_p2[0]); // extract target's angle axis
    const T *target_tx(& c_p2[3]); // extract target's position
    
    /** transform point into camera frame */
    T camera_point[3]; /** point in camera coordinates */
    transformPoint3d(target_aa, target_tx, point_, camera_point);
    camera_point[0] = camera_point[0] + T(rail_position_.x); // transform to camera's location along rail
    camera_point[1] = camera_point[1] + T(rail_position_.y); // transform to camera's location along rail
    camera_point[2] = camera_point[2] + T(rail_position_.z); // transform to camera's location along rail
    
    /** compute project point into image plane and compute residual */
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidualDist(camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy,  residual);
    
    return true;
  } /** end of operator() */
  
    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, 
				     Point3d rail_position,
				     Point3d point)
  {
    return (new ceres::AutoDiffCostFunction<RailICal3, 2, 9, 6>
	    (
	     new RailICal3(o_x, o_y, rail_position, point)
	     )
	    );
  }
  double ox_; /** observed x location of object in image */
  double oy_; /** observed y location of object in image */
  Point3d rail_position_; /** location of camera along rail */
  Point3d point_; /** point expressed in target coordinates */
};
  
class  RailSCal
{
public:
  RailSCal(double left_ob_x,  double left_ob_y,  Point3d left_point,
	   double right_ob_x, double right_ob_y, Point3d right_point, Point3d rail_position,
	   double lfx, double lfy, double lcx, double lcy, double lk1, double lk2, double lk3, double lp1, double lp2,
	   double rfx, double rfy, double rcx, double rcy, double rk1, double rk2, double rk3, double rp1, double rp2) :
    lox_(left_ob_x),   loy_(left_ob_y),   left_point_(left_point),
    rox_(right_ob_x),  roy_(right_ob_y),  right_point_(right_point),
    rail_position_(rail_position),
    lfx_(lfx), lfy_(lfy), lcx_(lcx), lcy_(lcy), lk1_(lk1), lk2_(lk2), lk3_(lk3), lp1_(lp1), lp2_(lp2),
    rfx_(rfx), rfy_(rfy), rcx_(rcx), rcy_(rcy), rk1_(rk1), rk2_(rk2), rk3_(rk3), rp1_(rp1), rp2_(rp2)
  {
  }
  
  template<typename T>
  bool operator()(	    const T* const c_p1,  /** pose of right camera relative to left frame [6] */
			    const T* const c_p2,  /** target_pose[6] */
			    T* residual) const    /** this residual has 4 terms one for each camera */
  {
    const T *C1toC2_aa(& c_p1[0]); // extract right camera's angle axis
    const T *C1toC2_tx(& c_p1[3]); // extract right camera's position
    const T *target_aa(& c_p2[0]); // extract target's angle axis
    const T *target_tx(& c_p2[3]); // extract target's position
    
    /** transform both points into left camera frame, note, that usually these would be exactly the same point.
	However, the camera observer may not provide observations in the same order. */
    T left_camera_point[3]; 
    T right_point_in_left_frame[3]; 
    transformPoint3d(target_aa, target_tx, right_point_, right_point_in_left_frame);
    transformPoint3d(target_aa, target_tx, left_point_, left_camera_point);
    left_camera_point[0] = left_camera_point[0] + T(rail_position_.x); // transform to camera's location along rail
    left_camera_point[1] = left_camera_point[1] + T(rail_position_.y); // transform to camera's location along rail
    left_camera_point[2] = left_camera_point[2] + T(rail_position_.z); // transform to camera's location along rail
    right_point_in_left_frame[0] = right_point_in_left_frame[0] + T(rail_position_.x); // transform to camera's location along rail
    right_point_in_left_frame[1] = right_point_in_left_frame[1] + T(rail_position_.y); // transform to camera's location along rail
    right_point_in_left_frame[2] = right_point_in_left_frame[2] + T(rail_position_.z); // transform to camera's location along rail
    
    /** transform right point in right camera frame */
    T right_camera_point[3]; 
    transformPoint(C1toC2_aa, C1toC2_tx, right_point_in_left_frame, right_camera_point);
    
    /** compute project point into image plane and compute residual */
    T lox = T(lox_);
    T loy = T(loy_);
    T rox = T(rox_);
    T roy = T(roy_);
    
    T lfx = T(lfx_);
    T lfy = T(lfy_);
    T lcx = T(lcx_);
    T lcy = T(lcy_);
    T lk1 = T(lk1_);
    T lk2 = T(lk2_);
    T lk3 = T(lk3_);	    
    T lp1 = T(lp1_);
    T lp2 = T(lp2_);
    
    T rfx = T(rfx_);
    T rfy = T(rfy_);
    T rcx = T(rcx_);
    T rcy = T(rcy_);
    T rk1 = T(rk1_);
    T rk2 = T(rk2_);
    T rk3 = T(rk3_);	    
    T rp1 = T(rp1_);
    T rp2 = T(rp2_);
    cameraPntResidualDist(left_camera_point,  lk1, lk2, lk3, lp1, lp2, lfx, lfy, lcx, lcy, lox, loy,  &(residual[0]));
    cameraPntResidualDist(right_camera_point, rk1, rk2, rk3, rp1, rp2, rfx, rfy, rcx, rcy, rox, roy,  &(residual[2]));			   
    return true;
  } /** end of operator() */
  
    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
  static ceres::CostFunction* Create(const double lo_x, const double lo_y, Point3d lpoint,
				     const double ro_x, const double ro_y, Point3d rpoint,
				     Point3d rail_position,
				     const double lfx, const double lfy,
				     const double lcx, const double lcy,
				     const double lk1, const double lk2, const double lk3,
				     const double lp1, const double lp2,
				     const double rfx, const double rfy,
				     const double rcx, const double rcy,
				     const double rk1, const double rk2, const double rk3,
				     const double rp1, const double rp2)
  {
    return (new ceres::AutoDiffCostFunction<RailSCal, 4, 6, 6>
	    (
	     new RailSCal(lo_x, lo_y, lpoint,
			  ro_x, ro_y, rpoint,
			  rail_position,
			  lfx, lfy, lcx, lcy, lk1, lk2, lk3, lp1, lp2,
			  rfx, rfy, rcx, rcy, rk1, rk2, rk3, rp1, rp2)
	     )
	    );
  }
  double lox_; /** observed x location of left_point in left image */
  double loy_; /** observed y location of left_point in left image */
  double rox_; /** observed x location of right point in right image */
  double roy_; /** observed x location of right point in right image */
  Point3d rail_position_; /** location of camera along rail */
  Point3d left_point_; /** left point expressed in target coordinates */
  Point3d right_point_; /** right point expressed in target coordinates */
  double lfx_,lfy_,lcx_,lcy_,lk1_,lk2_,lk3_,lp1_,lp2_; /** left camera intrinsics */
  double rfx_,rfy_,rcx_,rcy_,rk1_,rk2_,rk3_,rp1_,rp2_; /** right camera intrinsics */
};

class  StereoTargetLocator
{
public:
  StereoTargetLocator(double left_ob_x,  double left_ob_y,  Point3d left_point,
		      double right_ob_x, double right_ob_y, Point3d right_point, Pose6d C1toC2,
		      double lfx, double lfy, double lcx, double lcy, double lk1, double lk2, double lk3, double lp1, double lp2,
		      double rfx, double rfy, double rcx, double rcy, double rk1, double rk2, double rk3, double rp1, double rp2) :
    lox_(left_ob_x),   loy_(left_ob_y),   left_point_(left_point),
    rox_(right_ob_x),  roy_(right_ob_y),  right_point_(right_point),
    C1toC2_(C1toC2),
    lfx_(lfx), lfy_(lfy), lcx_(lcx), lcy_(lcy), lk1_(lk1), lk2_(lk2), lk3_(lk3), lp1_(lp1), lp2_(lp2),
    rfx_(rfx), rfy_(rfy), rcx_(rcx), rcy_(rcy), rk1_(rk1), rk2_(rk2), rk3_(rk3), rp1_(rp1), rp2_(rp2)
  {
  }
  
  template<typename T>
  bool operator()(	    const T* const c_p1,  /** target_pose[6] */
			    T* residual) const    /** this residual has 4 terms one for each camera */
  {
    const T *target_aa(& c_p1[0]); // extract target's angle axis
    const T *target_tx(& c_p1[3]); // extract target's position
    
    /** transform both points into left camera frame, note, that usually these would be exactly the same point.
	However, the camera observer may not provide observations in the same order. */
    T left_camera_point[3]; 
    T right_point_in_left_frame[3]; 
    transformPoint3d(target_aa, target_tx, right_point_, right_point_in_left_frame);
    transformPoint3d(target_aa, target_tx, left_point_, left_camera_point);

    /** transform right point in right camera frame */
    T right_camera_point[3]; 
    poseTransformPoint(C1toC2_, right_point_in_left_frame, right_camera_point);

    /** compute project point into image plane and compute residual */
    T lox = T(lox_);
    T loy = T(loy_);
    T rox = T(rox_);
    T roy = T(roy_);
      
    T lfx = T(lfx_);
    T lfy = T(lfy_);
    T lcx = T(lcx_);
    T lcy = T(lcy_);
    T lk1 = T(lk1_);
    T lk2 = T(lk2_);
    T lk3 = T(lk3_);	    
    T lp1 = T(lp1_);
    T lp2 = T(lp2_);
      
    T rfx = T(rfx_);
    T rfy = T(rfy_);
    T rcx = T(rcx_);
    T rcy = T(rcy_);
    T rk1 = T(rk1_);
    T rk2 = T(rk2_);
    T rk3 = T(rk3_);	    
    T rp1 = T(rp1_);
    T rp2 = T(rp2_);
    cameraPntResidualDist(left_camera_point,  lk1, lk2, lk3, lp1, lp2, lfx, lfy, lcx, lcy, lox, loy,  &(residual[0]));
    cameraPntResidualDist(right_camera_point, rk1, rk2, rk3, rp1, rp2, rfx, rfy, rcx, rcy, rox, roy,  &(residual[2]));			   
    return true;
  } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
  static ceres::CostFunction* Create(const double lo_x, const double lo_y, Point3d lpoint,
				     const double ro_x, const double ro_y, Point3d rpoint,
				     Pose6d C1toC2,
				     const double lfx, const double lfy,
				     const double lcx, const double lcy,
				     const double lk1, const double lk2, const double lk3,
				     const double lp1, const double lp2,
				     const double rfx, const double rfy,
				     const double rcx, const double rcy,
				     const double rk1, const double rk2, const double rk3,
				     const double rp1, const double rp2)
  {
    return (new ceres::AutoDiffCostFunction<StereoTargetLocator, 4, 6>
	    (
	     new StereoTargetLocator(lo_x, lo_y, lpoint,
				     ro_x, ro_y, rpoint,
				     C1toC2,
				     lfx, lfy, lcx, lcy, lk1, lk2, lk3, lp1, lp2,
				     rfx, rfy, rcx, rcy, rk1, rk2, rk3, rp1, rp2)
	     )
	    );
  }
  double lox_; /** observed x location of left_point in left image */
  double loy_; /** observed y location of left_point in left image */
  double rox_; /** observed x location of right point in right image */
  double roy_; /** observed x location of right point in right image */
  Pose6d C1toC2_; /** transforms points expressed in left camera frame into right camera frame */
  Point3d left_point_; /** left point expressed in target coordinates */
  Point3d right_point_; /** right point expressed in target coordinates */
  double lfx_,lfy_,lcx_,lcy_,lk1_,lk2_,lk3_,lp1_,lp2_; /** left camera intrinsics */
  double rfx_,rfy_,rcx_,rcy_,rk1_,rk2_,rk3_,rp1_,rp2_; /** right camera intrinsics */
};
  
class DistortedCameraFinder
{
public:
  DistortedCameraFinder(double ob_x, double ob_y, double fx, double fy,
			double cx, double cy, double k1, double k2, double k3, double p1, double p2,
			Point3d point) :
    ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy),
    k1_(k1), k2_(k2), k3_(k3), p1_(p1), p2_(p2),
    point_(point)
  {
  }

  template<typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters */
		  T* residual) const
  {
    const T *camera_aa(&c_p1[0]);
    const T *camera_tx(&c_p1[3]);
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    transformPoint3d(camera_aa, camera_tx, point_, camera_point);

    /** compute project point into image plane and compute residual */
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    T k1 = T(k1_);
    T k2 = T(k2_);
    T k3 = T(k3_);
    T p1 = T(p1_);
    T p2 = T(p2_);
    cameraPntResidualDist(camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, 
				     const double fx, const double fy, 
				     const double cx, const double cy,
				     const double k1, const double k2, const double k3,
				     const double p1, const double p2,
				     Point3d point)
  {
    return (new ceres::AutoDiffCostFunction<DistortedCameraFinder, 2, 6>(
									 new DistortedCameraFinder(o_x, o_y, fx, fy, cx, cy, k1, k2, k3, p1, p2, point)));
  }
  double ox_; /** observed x location of object in image */
  double oy_; /** observed y location of object in image */
  double fx_; /*!< known focal length of camera in x */
  double fy_; /*!< known focal length of camera in y */
  double cx_; /*!< known optical center of camera in x */
  double cy_; /*!< known optical center of camera in y */
  double k1_; /*!< known radial distorition k1 */
  double k2_; /*!< known radial distorition k2 */
  double k3_; /*!< known radial distorition k3 */
  double p1_; /*!< known decentering distorition p1 */
  double p2_; /*!< known decentering distorition p2 */
  Point3d point_; /*!< known location of point in target coordinates */
};

class  RailICalNoDistortion
{
public:
  RailICalNoDistortion(double ob_x, double ob_y, double rail_position, Point3d point) :
    ox_(ob_x), oy_(ob_y), rail_position_(rail_position), point_(point)
  {
  }

  template<typename T>
  bool operator()(	    const T* const c_p1,  /**intrinsics [4] */
			    const T* const c_p2,  /**target_pose [6] */
			    T* residual) const
  {
    T fx, fy, cx, cy;      // extract intrinsics
    extractCameraIntrinsics(c_p1, fx, fy, cx, cy);
    const T *target_aa(& c_p2[0]); // extract target's angle axis
    const T *target_tx(& c_p2[3]); // extract target's position

    /** transform point into camera frame */
    T camera_point[3]; /** point in camera coordinates */
    transformPoint3d(target_aa, target_tx, point_, camera_point);
    camera_point[2] = camera_point[2] + T(rail_position_); // transform to camera's location along rail

    /** compute project point into image plane and compute residual */
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy,  residual);

    return true;
  } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y,
				     const double rail_position,
				     Point3d point)
  {
    return (new ceres::AutoDiffCostFunction<RailICal, 2, 4, 6>
	    (
	     new RailICal(o_x, o_y, rail_position, point)
	     )
	    );
  }
  double ox_; /** observed x location of object in image */
  double oy_; /** observed y location of object in image */
  double rail_position_; /** location of camera along rail */
  Point3d point_; /** point expressed in target coordinates */
};
  /* @brief rangeSensorExtrinsic This cost function is to be used for extrinsic calibration of a 3D camera. 
   * This is used when the focal lenghts, optical center and distortion parameters used to get the 3D data
   * are not available. Instead, we have the point cloud and an intensity image. We use the intensity
   * image to find the centers of the cirlces of the calibration target and then assume that the same index 
   * in the point cloud corresponds to the x,y,z location of this point.
   * This assumption is probably erroneous because rectification of the intensity image is usually performed 
   * prior to generation of the point cloud. 
   * Solving extrinsic cal for a range sensor is equivalent to the inner computation of ICP where correspondence
   * is perfectly known. There exists an analytic solution. 
   */ 
class  RangeSensorExtrinsicCal
{
public:
  RangeSensorExtrinsicCal(double ob_x, double ob_y, double ob_z, Point3d point) :
    ox_(ob_x), oy_(ob_y), oz_(ob_z),
    point_(point)
  {
  }

  template<typename T>
  bool operator()(	    const T* const c_p1,  /**extriniscs [6] */
			    T* residual) const
  {
    const T *camera_aa(& c_p1[0]); // extract camera's angle axis
    const T *camera_tx(& c_p1[3]); // extract camera's position

    /** transform point into camera frame */
    T camera_point[3]; /** point in camera coordinates */
    transformPoint3d(camera_aa, camera_tx, point_, camera_point);

    /** compute residual */
    residual[0] = camera_point[0] - T(ox_);
    residual[1] = camera_point[1] - T(oy_);
    residual[2] = camera_point[2] - T(oz_);

    return true;
  } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y,  const double o_z,
				     Point3d point)
  {
    return (new ceres::AutoDiffCostFunction<RangeSensorExtrinsicCal, 3, 6>
	    (
	     new RangeSensorExtrinsicCal(o_x, o_y, o_z, point)
	     )
	    );
  }
  double ox_; /** observed x location of object in 3D data */
  double oy_; /** observed y location of object in 3D data */
  double oz_; /** observed z location of object in 3D data */
  Point3d point_; /** point expressed in target coordinates */
};


} // end of namespace
#endif
