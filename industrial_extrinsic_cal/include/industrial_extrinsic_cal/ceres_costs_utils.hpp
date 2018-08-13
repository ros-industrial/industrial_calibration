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
double PI = 4*atan(1);

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

template <typename T>
void extractPoseExtrinsics(Pose6d P, T E[6] );
template <typename T>
inline void extractPoseExtrinsics(Pose6d P, T E[6])
{
  E[0] = T(P.ax);
  E[1] = T(P.ay);
  E[2] = T(P.az);
  E[3] = T(P.x);
  E[4] = T(P.y);
  E[5] = T(P.z);
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
 *  @param point, the original point
 *  @param t_point, the transformed point
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


/*! \brief ceres compliant function to apply an angle-axis and translation to transform a point
 *  @param E, all 6 extrinsic parameters, ax,ay,az,tx,ty,tz
 *  @param point, the original point
 *  @param t_point, the transformed point
 */
template <typename T>
inline void eTransformPoint(const T E[6], const T point[3], T t_point[3]);
template <typename T>
inline void eTransformPoint(const T E[6], const T point[3], T t_point[3])
{
  
  ceres::AngleAxisRotatePoint(E, point, t_point);
  t_point[0] = t_point[0] + E[3];
  t_point[1] = t_point[1] + E[4];
  t_point[2] = t_point[2] + E[5];
}

/*! \brief ceres compliant function to apply dh-parameters to transform a point down the link
 *  @param a     link x axis length
 *  @param alpha link twist around x axis
 *  @param d     link z axis length       (joint valure for prismatic joints) 
 *  @param theta link twist around z axis (joint value for rotary joints)
 *  @param point, the original point
 *  @param t_point, the transformed point
 */
template <typename T>
inline void dhTransformPoint(const T a, const T alpha, const T d, const T theta, const T point[3], T t_point[3]);
template <typename T>
inline void dhTransformPoint(const T a, const T alpha, const T d, const T theta, const T point[3], T t_point[3])
{
  T M[3][4];
  T st = sin(theta);
  T ct = cos(theta);
  T sa = sin(alpha);
  T ca = cos(alpha);
  M[0][0] = ct;      M[0][1] = -st*ca; M[0][2] =  st*sa; M[0][3] = a*ct;
  M[1][0] = st;      M[1][1] =  ct*ca; M[1][2] = -ct*sa; M[1][3] = a*st;
  M[2][0] = T(0.0);  M[2][1] =  sa;    M[2][2] =  ca;    M[2][3] = d;

  t_point[0] = M[0][0]*point[0] + M[0][1]*point[1] + M[0][2]*point[2] + M[0][3];
  t_point[1] = M[1][0]*point[0] + M[1][1]*point[1] + M[1][2]*point[2] + M[1][3];
  t_point[1] = M[2][0]*point[0] + M[2][1]*point[1] + M[2][2]*point[2] + M[2][3];
}

/*! \brief ceres compliant function to apply dh-parameters to transform a Point3d down the link
 *  @param a     link x axis length
 *  @param alpha link twist around x axis
 *  @param d     link z axis length       (joint valure for prismatic joints) 
 *  @param theta link twist around z axis (joint value for rotary joints)
 *  @param point, the original point
 *  @param t_point, the transformed point
 */
template <typename T>
inline void dhTransformPoint3d(const T a, const T alpha, const T d, const T theta, const Point3d& point_, T t_point[3]);
template <typename T>
inline void dhTransformPoint3d(const T a, const T alpha, const T d, const T theta, const Point3d& point_, T t_point[3])
{
  T point[3];
  point[0] = T(point_.x);
  point[1] = T(point_.y);
  point[2] = T(point_.z);
  dhTransformPoint(a, alpha, d, theta, point, t_point); // use subroutine call rather than duplicate code
}

/*! \brief ceres compliant function to apply dh-parameters to transform a point up the link
 *  @param a     link x axis length
 *  @param alpha link twist around x axis
 *  @param d     link z axis length       (joint valure for prismatic joints) 
 *  @param theta link twist around z axis (joint value for rotary joints)
 *  @param point, the original point
 *  @param t_point, the transformed point
 */
template <typename T>
inline void dhInvTransformPoint(const T a, const T alpha, const T d, const T theta, const T point[3], T t_point[3]);
template <typename T>
inline void dhInvTransformPoint(const T a, const T alpha, const T d, const T theta, const T point[3], T t_point[3])
{
  T M[3][4];
  T st = sin(theta);
  T ct = cos(theta);
  T sa = sin(alpha);
  T ca = cos(alpha);


  /* original M matrix
  M[0][0] = ct;   M[0][1] = -st*ca; M[0][2] =  st*sa; M[0][3] = a*ct;
  M[1][0] = st;   M[1][1] =  ct*ca; M[1][2] = -ct*sa; M[1][3] = a*st;
  M[2][0] = 0.0;  M[2][1] =  sa;    M[2][2] =  ca;    M[2][3] = d;

  The inverse is the transpose of the rotation part, and if M is composed of four vectors M=[R | v] = [n | o | a| v] then
  the upper 3x3 of M_inv is the transpose of the upper 3x3 of M and
  the last column of M_inv = [  -n^t v ; -o^t v ; a^t v ]
  */

  M[0][0] =    ct;    M[0][1] =  st;    M[0][2] =  T(0.0);  M[0][3] = -(  ct   * a*ct   +  st    * a*st  + T(0.0) * d);
  M[1][0] = -st*ca;   M[1][1] =  ct*ca; M[1][2] =  sa;      M[1][3] = -(-st*ca * a*ct   +  ct*ca * a*st  + sa     * d);
  M[2][0] =  st*sa;   M[2][1] = -ct*sa; M[2][2] =  ca;      M[2][3] = -( st*sa * a*ct   + -ct*sa * a*st  + ca     * d);

  t_point[0] = M[0][0]*point[0] + M[0][1]*point[1] + M[0][2]*point[2] + M[0][3];
  t_point[1] = M[1][0]*point[0] + M[1][1]*point[1] + M[1][2]*point[2] + M[1][3];
  t_point[1] = M[2][0]*point[0] + M[2][1]*point[1] + M[2][2]*point[2] + M[2][3];
}

/*! \brief ceres compliant function to apply dh-parameters to transform a Point3d
 *  @param a     link x axis length
 *  @param alpha link twist around x axis
 *  @param d     link z axis length       (joint valure for prismatic joints) 
 *  @param theta link twist around z axis (joint value for rotary joints)
 *  @param point, the original point
 *  @param t_point, the transformed point
 */
template <typename T>
inline void dhInvTransformPoint3d(const T a, const T alpha, const T d, const T theta, const Point3d& point_, T t_point[3]);
template <typename T>
inline void dhInvTransformPoint3d(const T a, const T alpha, const T d, const T theta, const Point3d& point_, T t_point[3])
{
  T point[3];
  point[0] = T(point_.x);
  point[1] = T(point_.y);
  point[2] = T(point_.z);
  dhInvTransformPoint(a, alpha, d, theta, point, t_point); // use subroutine call rather than duplicate code
}

/*! \brief ceres compliant function to apply a pose to transform a point
 *  @param pose, contains both rotation and translation in a structure
 *  @param point, the original point
 *  @param t_point, the transformed point
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
 *  @param tx, translation tx, ty and tz
 *  @param point, the original point in a Point3d form
 *  @param t_point, the transformed point
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


/*! \brief ceres compliant function to apply an angle-axis and translation to transform a point in Point3d form
 *  @param E extrinsic parameters  ax, ay, az, tx, ty, tz
 *  @param point, the original point in a Point3d form
 *  @param t_point, the transformed point
 */
template <typename T>
void eTransformPoint3d(const T E[6],  const Point3d& point, T t_point[3]);
template <typename T>
inline void eTransformPoint3d(const T E[6], const Point3d& point, T t_point[3])
{
  T point_[3];
  point_[0] = T(point.x);
  point_[1] = T(point.y);
  point_[2] = T(point.z);
  ceres::AngleAxisRotatePoint(E, point_, t_point);
  t_point[0] = t_point[0] + E[3];
  t_point[1] = t_point[1] + E[4];
  t_point[2] = t_point[2] + E[5];
}

/*! \brief ceres compliant function get a templated rotation from a Pose6d structure
 *  @param pose, the input pose
 *  @param R, the output rotatation matrix
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

/*! \brief ceres compliant function to multiply two 6DoF angle-axis-tx poses 
 *  @param T1 6Dof angle-axis-tx pose 1
 *  @param T2 6Dof angle-axis-tx pose 2
 *  @param T1T2 6Dof product in angle-axis-tx form
 */
template <typename T>
void extrinsicsMult(const T T1[6], const T T2[6], T T1T2[6]);
template <typename T>
inline void extrinsicsMult(const T T1[6], const T T2[6], T T1T2[6])
{
  T R1[9],R2[9],R3[9];
  const T* aa1(&T1[0]); // angle axis from T1
  const T* aa2(&T2[0]); // angle axis from T2
  const T* tx1(&T1[3]); // position part of T1
  const T* tx2(&T2[3]); // position part of T2
  T aa3[3], tx3[3];
  ceres::AngleAxisToRotationMatrix(aa1, R1);
  ceres::AngleAxisToRotationMatrix(aa2, R2);
  rotationProduct(R1, R2, R3);
  ceres::RotationMatrixToAngleAxis(R3,aa3);
  eTransformPoint(T1, tx2, tx3);

  T1T2[0] = aa3[0];
  T1T2[1] = aa3[1];
  T1T2[2] = aa3[2];
  T1T2[3] = tx3[0];
  T1T2[4] = tx3[1];
  T1T2[5] = tx3[2];

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

/*! \brief ceres compliant function to compute projection of a point into the image plane of a pinhole camera with lens
 * distortion
 *  @param point[3] the input point
 *  @param fx focal length in x
 *  @param fy focal length in y
 *  @param cx optical center in x
 *  @param cy optical center in y
 *  @param k1 lens distortion k1
 *  @param k2 lens distortion k2
 *  @param k3 lens distortion k3
 *  @param p1 lens distortion p1
 *  @param p2 lens distortion p2
 *  @param ox observation in x
 *  @param oy observation in y
 */
template <typename T>
void projectPntDist(T point[3], T& fx, T& fy, T& cx, T& cy, T& k1, T & k2, T& k3, T& p1, T& p2, T& ox, T& oy);
template <typename T>
inline void projectPntDist(T point[3], T& fx, T& fy, T& cx, T& cy, T& k1, T & k2, T& k3, T& p1, T& p2, T& ox, T& oy)
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
  ox = fx * xpp + cx;
  oy = fy * ypp + cy;
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
void cameraCircResidualDist(T point[3], T& circle_diameter,const T TtoC[6], T& k1, T& k2, T& k3, T& p1, T& p2, T& fx,
                            T& fy, T& cx, T& cy, T& ox, T& oy, T residual[2]);
template <typename T>
inline void cameraCircResidualDist(T point[3], T& circle_diameter, const T TtoC[6], T& k1, T& k2, T& k3, T& p1, T& p2,
                                   T& fx, T& fy, T& cx, T& cy, T& ox, T& oy, T residual[2])

{  
  // Circle Delta is the difference between the projection of the center of the circle
  // and the center of the projected ellipse

  // Concept for predicion:
  // one point on the perimeter of the circle will be farther away than all the others
  // one point on the perimeter of the circle will be closer than all the others
  // The projection of these two points into the camera frame forms the minor axis of the observed ellipse
  // the center of that line segment is the observed center

  // The Equations:
  // all points on the perimeter in target coordinates defined by pt =
  // xt = xp + rcos(theta)
  // yt = yp + rsin(theta)
  // zt = 0                   where r is the circle radius
  // The homogeneous transform of points into camera frame is defined by:
  // R1 R2 R3 tx
  // R4 R5 R6 ty
  // R7 R8 R9 tz
  // 0  0  0  1
  // therefore, the z value of the perimeter points is given by
  // pt_z = R7*xt + R8*yt + R9*zt + tz
  // we want to find where pt_z is both largest and smallest
  // take the first derivative with respect to theta and set to zero
  // 0 = -R7rsin(theta) + R8rcos(theta)
  // theta = R8rcos(theta)/R7rsin(theta) = atan(R8/R7) or atan(R8/R7) + PI
  // we now have the two points pt_1 and pt_t
  // pt_1 = (xp + rcos(atan(R8/R7))), yp + rsin(atan(R8/R7)), 0)
  // pt_2 = (xp + rcos(atan(R8/R7) + PI)), yp + rsin(atan(R8/R7) + PI), 0)

  T xp1 = point[0]; // center of circle in target coordinates
  T yp1 = point[1];
  T zp1 = point[2];
  T pt_1[3]; // max/min point in target coordinates
  T pt_2[3]; // max/min point in target coordinates
  T camera_point1[3]; // max/min point in camera coordinates
  T camera_point2[3]; // max/min point in camera coordinates
  T r   = T(circle_diameter/2.0);
  const T* target_to_camera_aa(&TtoC[0]);
  const T* target_to_camera_tx(&TtoC[3]);
  T R[9];

  ceres::AngleAxisToRotationMatrix(target_to_camera_aa, R);
  T R7     = R[6];
  T R8     = R[7];

  // find max/min points 1 and 2
  pt_1[0] = xp1 + r*cos(atan(R8/R7));
  pt_1[1] = yp1 + r*sin(atan(R8/R7));
  pt_1[2] = zp1; // this should be zero

  pt_2[0] = xp1 + r*cos(atan(R8/R7) + T(PI));
  pt_2[1] = yp1 + r*sin(atan(R8/R7) + T(PI));
  pt_2[2] = zp1; // this should be zero

  eTransformPoint(TtoC, pt_1, camera_point1);
  eTransformPoint(TtoC, pt_2, camera_point2);

  T pnt1_ix, pnt1_iy;  // image of point1
  T pnt2_ix, pnt2_iy;  // image of point2
  projectPntDist(camera_point1, fx, fy, cx, cy, k1, k2, k3, p1, p2, pnt1_ix, pnt1_iy);
  projectPntDist(camera_point2, fx, fy, cx, cy, k1, k2, k3, p1, p2, pnt2_ix, pnt2_iy);

  /* compute the residual */
  residual[0] = (pnt1_ix + pnt2_ix)/T(2.0) - ox;
  residual[1] = (pnt1_iy + pnt2_iy)/T(2.0) - oy;

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
void cameraCircResidual(T point[3], T& circle_diameter, const T TtoC[6], T& fx, T& fy, T& cx, T& cy, T& ox, T& oy,
                        T residual[2]);
template <typename T>
inline void cameraCircResidual(T point[3], T& circle_diameter, const T TtoC[6], T& fx, T& fy, T& cx, T& cy, T& ox, T& oy,
                               T residual[2])
{
  
  // Circle Delta is the difference between the projection of the center of the circle
  // and the center of the projected ellipse

  // Concept for predicion:
  // one point on the perimeter of the circle will be farther away than all the others
  // one point on the perimeter of the circle will be closer than all the others
  // The projection of these two points into the camera frame forms the minor axis of the observed ellipse
  // the center of that line segment is the observed center

  // The Equations:
  // all points on the perimeter in target coordinates defined by pt =
  // xt = xp + rcos(theta)
  // yt = yp + rsin(theta)
  // zt = 0                   where r is the circle radius
  // The homogeneous transform of points into camera frame is defined by:
  // R1 R2 R3 tx
  // R4 R5 R6 ty
  // R7 R8 R9 tz
  // 0  0  0  1
  // therefore, the z value of the perimeter points is given by
  // pt_z = R7*xt + R8*yt + R9*zt + tz
  // we want to find where pt_z is both largest and smallest
  // take the first derivative with respect to theta and set to zero
  // 0 = -R7rsin(theta) + R8rcos(theta)
  // theta = R8rcos(theta)/R7rsin(theta) = atan(R8/R7) or atan(R8/R7) + PI
  // we now have the two points pt_1 and pt_t
  // pt_1 = (xp + rcos(atan(R8/R7))), yp + rsin(atan(R8/R7)), 0)
  // pt_2 = (xp + rcos(atan(R8/R7) + PI)), yp + rsin(atan(R8/R7) + PI), 0)

  T xp1 = point[0]; // center of circle in target coordinates
  T yp1 = point[1];
  T zp1 = point[2];
  T pt_1[3]; // max/min point in target coordinates
  T pt_2[3]; // max/min point in target coordinates
  T camera_point1[3]; // max/min point in camera coordinates
  T camera_point2[3]; // max/min point in camera coordinates
  T r   = T(circle_diameter/2.0);
  const T* target_to_camera_aa(&TtoC[0]);
  const T* target_to_camera_tx(&TtoC[3]);
  T predict_p1[2], predict_p2[2]; // predicted image of max/min point1 and point2
  T R[9];

  ceres::AngleAxisToRotationMatrix(target_to_camera_aa, R);
  T R7     = R[6];
  T R8     = R[7];

  // find max/min points 1 and 2
  pt_1[0] = xp1 + r*cos(atan(R8/R7));
  pt_1[1] = yp1 + r*sin(atan(R8/R7));
  pt_1[2] = zp1; // this should be zero

  pt_2[0] = xp1 + r*cos(atan(R8/R7) + T(PI));
  pt_2[1] = yp1 + r*sin(atan(R8/R7) + T(PI));
  pt_2[2] = zp1; // this should be zero

  eTransformPoint(TtoC,pt_1, camera_point1);
  eTransformPoint(TtoC,pt_2, camera_point2);
  
  T pnt1_ix, pnt1_iy;  // image of point1
  T pnt2_ix, pnt2_iy;  // image of point2
  projectPntNoDistortion(camera_point1, fx, fy, cx, cy, pnt1_ix, pnt1_iy);
  projectPntNoDistortion(camera_point2, fx, fy, cx, cy, pnt2_iy, pnt2_iy);

  /* compute the residual */
  /* compute the residual */
  residual[0] = (pnt1_ix + pnt2_ix)/T(2.0) - ox;
  residual[1] = (pnt1_iy + pnt2_iy)/T(2.0) - oy;
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
/********************************** THE COST FUNCTIONS ************************************
 *  The main thing to pay attention to in determining which cost function you want is:
 *  What variables are known and what variables are adjusted
 *  known variables appear in the constructor
 *  unknown variables appear in the cost evaluation function operator()
 *
 *  All cost functions do the following:
 *  1. Transform a point being observed into the camera's coordinates
 *  2. Predict the image of that point
 *  3. Return the difference between the predicted image of the point and the observed image of that point
 *  Ceres simply adjusts the unknowns to minimize the sum of squared costs
 *
 *  The second thing to pay attention to is whether the location of the point is known and fixed
 *  in the target coordinate. MOST of the time it is. If point appears in the operator() then
 *  you are allowing ceres to adjust the location of the point on the target.
 *  This is rarely useful. Most of the time you will want a Points known cost function (PK)
 *
 *  The third thing to pay attention to is whether you used a circle target or not
 *  With a checkerboard, the locations of the intersections simply defined, but CANNOT be found
 *  with as high an accuracy as a circle may be located. The checkerboard points are formed by the
 *  intersection between two lines. In the worst case, a line can lie either along a row or a column
 *  in this case, it is impossible to provide better than 1/2 pixel accuracy.
 *  The perimeter of a circle on the other hand can be fit to an ellipse with an accuracy of better
 *  than 1/4 pixel as long as the diameter is greater than 10 pixels. Typically we see .05 pixel accuracy
 *  for circles. We fit an ellipse instead of a circle because the image of a circle is an ellipse.
 *  Unfortunately, the projection of the center of a circle is NOT the center of the projected ellipse.
 *  I'll say it again with more detail to be clear.
 *  The image of the perimeter of a circle forms an elipse in the image. The center of the observed ellipse
 *  is defined as the middle of the major axis and minor axis of the observed ellipse. This
 *  observed center is NOT in the same place as the image of the center point of the circle.
 *  The offset is small, typically much less than 1/10th of a pixel. The offset is largest at 45degrees.
 *  It gets bigger with bigger circles. Because the error is so small, it is always safe to use standard
 *  cost functions that don't have the word Circle in their name. However, sometimes you may want
 *  to try them.
 *
 *  The fourth thing to pay attention to is the extra transform information. For example, sometimes
 *  either the camera or the target is mounted on a robot's link. In these cases, we usually
 *  collect the transform from the world or robot base frame to the end of the link using tf.
 *  These cost functions accept the link transform and have the word link in their names
 *
 *  Most common calibration tasks and which cost functions they might use
 *  1. find the pose of the target or the camera relative to one another
 *     a. unknown variables are the 6Dof extrinsics
 *     b. known variables are the intrinsics and the location of the point on the target
 *     CameraReprjErrorPK()           
 *     CircleCameraReprjErrorPK()
 *     DistortedCameraFinder()      same as CameraRepjErrorPK but for unrectified images
 *
 *  2. Find the pose of a camera mounted on the robot using a target in the workcell
 *     a. unknown variables: Extrinsics of camera relative to link, Extrinsic of target relative to base/world
 *     b. known variables:  Transform from base/world to link, camera intrinsices, point location in target frame
 *     LinkCameraTargetReprjErrorPK()          Both of these assume a rectified image is used
 *     LinkCameraCircleTargetReprjErrorPK()
 *     
 *  3. Find the pose of a camera in the workcell by viewing a target mounted on the robot
 *     a. unknown variables: Extrinsics of camera relative to world/base, Extrinsic of target relative to link
 *     b. known variables:  Transform from base/world to link, camera intrinsices, point location in target frame
 *     LinkTargetCameraReprjErrorPK()           Both of these assume a rectified image is used
 *     LinkCircleTargetCameraReprjErrorPK()
 *
 *  4. The standard intrinsic calibration method used by opencv and matlab
 *     a. unknown variables: Extrinsics of the target, and intrinsics of the camera
 *     b. known variables:   Point location in target frame,
 *     CameraReprjErrorWithDistortionPK()
 *     CircleCameraReprjErrorWithDistortionPK()
 *
 ******************************************************************************************/
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
    const T* camera_T(&c_p1[0]);
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;
    extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    T camera_point[3]; /** point in camera coordinates*/

    /** transform point into camera coordinates */
    eTransformPoint(camera_T, point, camera_point);

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
    const T* camera_T(&c_p1[0]);
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;
    extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    T camera_point[3];

    /** transform point into camera coordinates */
    eTransformPoint3d(camera_T, point_, camera_point);

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
    const T* camera_T(&c_p1[0]);
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    eTransformPoint(camera_T, point, camera_point);

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
  bool operator()(const T* point, /** point being projected, yes this has 3 parameters */
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
    const T* camera_T(&c_p1[0]);
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    eTransformPoint3d(camera_T, point_, camera_point);

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
    const T* camera_T(&c_p1[0]);
    const T* target_T(&c_p2[0]);

    T world_point[3];  /** point in world coordinates */
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    eTransformPoint(target_T, point, world_point);
    eTransformPoint(camera_T, world_point, camera_point);

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
    const T* camera_T(&c_p1[0]);
    const T* target_T(&c_p2[0]);
    T world_point[3];  /** point in world coordinates */
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    eTransformPoint3d(target_T, point_, world_point);
    eTransformPoint(camera_T, world_point, camera_point);

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
    const T* camera_T(&c_p1[0]);
    const T* target_T(&c_p2[0]);
    T link_point[3];   /** point in link coordinates */
    T world_point[3];  /** point in world coordinates */
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    eTransformPoint(target_T, point, link_point);
    poseTransformPoint(link_pose_, link_point, world_point);
    eTransformPoint(camera_T, world_point, camera_point);

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
    const T* camera_T(&c_p1[0]);
    const T* target_T(&c_p2[0]);
    T link_point[3];   /** point in link coordinates */
    T world_point[3];  /** point in worls coordinates */
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    eTransformPoint3d(target_T, point_, link_point);
    poseTransformPoint(link_pose_, link_point, world_point);
    eTransformPoint(camera_T, world_point, camera_point);

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

// used whenever the target or camera is on the wrist of a robot while the camera or target is in the workcell
// The 
class WristCal
{
public:
  WristCal(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Pose6d targetm_to_cameram,
                               Point3d point)
    : ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), targetm_to_cameram_(targetm_to_cameram), point_(point)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters */
                  const T* const c_p2, /** 6Dof transform of target points into world frame */
                  T* residual) const
  {
    const T* camera_T(&c_p1[0]);
    const T* target_T(&c_p2[0]);
    T target_mount_point[3];   /** point in link coordinates */
    T camera_mount_point[3];  /** point in worls coordinates */
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    eTransformPoint3d(target_T, point_, target_mount_point);
    poseTransformPoint(targetm_to_cameram_, target_mount_point, camera_mount_point);
    eTransformPoint(camera_T, camera_mount_point, camera_point);

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
  double ox_;                 /** observed x location of object in image */
  double oy_;                 /** observed y location of object in image */
  double fx_;                 /*!< known focal length of camera in x */
  double fy_;                 /*!< known focal length of camera in y */
  double cx_;                 /*!< known optical center of camera in x */
  double cy_;                 /*!< known optical center of camera in y */
  Point3d point_;             /*! location of point in target coordinates */
  Pose6d targetm_to_cameram_; /*!< transform from targets mount to cameras mount frame. 
                                 (e.g. camera attached to tool0 and target attached to world this becomes tool0 to world)    
                                 (e.g. camera attached to world and target attached to tool0 this becomes world to tool0)    
				 in the camera attached to tool0 case, 
                                 the transform takes points in the world frame and expresses them in tool0 frame
			      */
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
    const T* camera_T(&c_p1[0]);
    T world_point[3];  /** point in worls coordinates */
    T camera_point[3]; /** point in camera coordinates */
    T point[3];
    point[0] = T(point_.x);
    point[1] = T(point_.y);
    point[2] = T(point_.z);
    /** transform point into camera coordinates */
    poseTransformPoint(target_pose_, point, world_point);
    eTransformPoint(camera_T, world_point, camera_point);

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
    const T* camera_T(&c_p1[0]);
    const T* target_T(&c_p2[0]);
    T world_point[3];  /** point in world coordinates */
    T link_point[3];   /** point in link coordinates */
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    eTransformPoint(target_T, point, world_point);
    poseTransformPoint(link_posei_, world_point, link_point);
    eTransformPoint(camera_T, link_point, camera_point);

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
    const T* camera_T(&c_p1[0]);
    const T* target_T(&c_p2[0]);
    T world_point[3];  /** point in world coordinates */
    T link_point[3];   /** point in link coordinates */
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    eTransformPoint3d(target_T, point_, world_point);
    poseTransformPoint(link_posei_, world_point, link_point);
    eTransformPoint(camera_T, link_point, camera_point);

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
    const T* camera_T(&c_p1[0]); // all six parameters with a new name
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;
    extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    T camera_point[3]; /** point in camera coordinates*/

    /** transform point into camera coordinates */
    eTransformPoint(camera_T, point, camera_point);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidualDist(camera_point, circle_diameter, camera_T, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

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
    const T* camera_T(&c_p1[0]);
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;
    extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    T camera_point[3]; /** point in camera coordinates */
    T R_TtoC[9];       /** rotation from target to camera coordinates
      
          /** find point in camera coordinates */
    eTransformPoint3d(camera_T, point_, camera_point);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidualDist(camera_point, circle_diameter, camera_T, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

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
    const T* camera_T(&c_p1[0]); // all six parameters with a new name
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    eTransformPoint(camera_T, point, camera_point);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidual(camera_point, circle_diameter, camera_T, fx, fy, cx, cy, ox, oy, residual);

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
    const T* camera_T(&c_p1[0]); // all six parameters with a new name
    T camera_point[3]; /* point in camera coordinates */

    /** rotate and translate point into camera coordinates*/
    eTransformPoint3d(camera_T, point_, camera_point);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidual(camera_point, circle_diameter, camera_T, fx, fy, cx, cy, ox, oy, residual);

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
    const T* camera_T(&c_p1[0]); // full 6dof parameters for camera pose
    const T* target_T(&c_p3[0]); // full 6dof parameters for target pose
    T TtoC[6];                   // full 6dof parameters for transforming target points into camera frame
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;
    T camera_point[3];
    extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);

    extrinsicsMult(camera_T, target_T, TtoC);
    eTransformPoint(TtoC, point, camera_point);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidualDist(camera_point, circle_diameter, TtoC, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

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
    const T* camera_T(&c_p1[0]); // full 6dof parameters for camera pose
    const T* target_T(&c_p3[0]); // full 6dof parameters for target pose
    T TtoC[6];                   // full 6dof parameters for transforming target points into camera frame
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;
    T camera_point[3];
    extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);

    extrinsicsMult(camera_T, target_T, TtoC);
    eTransformPoint3d(TtoC, point_, camera_point);


    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidualDist(camera_point, circle_diameter, camera_T, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

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
    const T* camera_T(&c_p1[0]); // all six parameters with a new name
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;
    T camera_point[3];
    extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);

    /** transform point into camera coordinates */
    eTransformPoint(camera_T, point, camera_point);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidualDist(camera_point, circle_diameter, camera_T, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

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
    const T* camera_T(&c_p1[0]); // all six parameters with a new name
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;
    extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    T camera_point[3];        /** point in camera coordinates */

    /** transform point into camera coordinates */
    eTransformPoint3d(camera_T, point_, camera_point);

    T circle_diameter = T(circle_diameter_);
    T ox = T(ox_);
    T oy = T(oy_);

    /** compute project point into image plane and compute residual */
    cameraCircResidualDist(camera_point, circle_diameter, camera_T, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

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
    const T* camera_T(&c_p1[0]); // all six parameters for camera pose
    const T* target_T(&c_p3[0]); // all six parameters for target pose
    T        TtoC[6];            // all six parameters for target to camera transform
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;
    T camera_point[3]; /** point in world coordinates */

    extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    
    // compute transform taking points in target frame into camera frame
    extrinsicsMult(camera_T, target_T, TtoC);

    /** transform point into camera coordinates */
    eTransformPoint3d(TtoC, point_, camera_point);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidualDist(camera_point, circle_diameter, TtoC, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

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
    const T* camera_T(&c_p1[0]); // all six parameters for camera pose
    const T* target_T(&c_p2[0]); // all six parameters for target pose
    T        TtoC[6];            // all six parameters for target to camera transform
    T camera_point[3];
    
    // compute transform taking points in target frame into camera frame
    extrinsicsMult(camera_T, target_T, TtoC);

    /** transform point into camera coordinates */
    eTransformPoint(TtoC, point, camera_point);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidual(camera_point, circle_diameter, TtoC, fx, fy, cx, cy, ox, oy, residual);

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
    const T* camera_T(&c_p1[0]); // all six parameters for camera pose
    const T* target_T(&c_p2[0]); // all six parameters for target pose
    T        TtoC[6];            // all six parameters for target to camera transform
    T camera_point[3];
    
    // compute transform taking points in target frame into camera frame
    extrinsicsMult(camera_T, target_T, TtoC);

    /** transform point into camera coordinates */
    eTransformPoint3d(TtoC, point_, camera_point);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidual(camera_point, circle_diameter, TtoC, fx, fy, cx, cy, ox, oy, residual);

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
    const T* camera_T(&c_p1[0]); // all six parameters for camera pose
    const T* target_T(&c_p2[0]); // all six parameters for target pose
    T        TtoC[6];            // all six parameters for target to camera transform
    T        TtoL[6];            // target to link transform
    T      link_T[6];            // world to link transform
    T camera_point[3];
    
    // compute transform taking points in target frame into camera frame
    extractPoseExtrinsics(link_pose_,link_T);
    extrinsicsMult(link_T, target_T, TtoL);
    extrinsicsMult(camera_T, TtoL, TtoC);

    /** transform point into camera coordinates */
    eTransformPoint(TtoC, point, camera_point);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidual(camera_point, circle_diameter, TtoC, fx, fy, cx, cy, ox, oy, residual);

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
    const T* camera_T(&c_p1[0]); // all six parameters for camera pose
    const T* target_T(&c_p2[0]); // all six parameters for target pose
    T        TtoC[6];            // all six parameters for target to camera transform
    T        TtoL[6];            // target to link transform
    T      link_T[6];            // world to link transform
    T camera_point[3];
    
    // compute transform taking points in target frame into camera frame
    extractPoseExtrinsics(link_pose_, link_T);
    extrinsicsMult(link_T, target_T, TtoL);
    extrinsicsMult(camera_T, TtoL, TtoC);

    /** transform point into camera coordinates */
    eTransformPoint3d(TtoC, point_, camera_point);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidual(camera_point, circle_diameter, TtoC, fx, fy, cx, cy, ox, oy, residual);

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
    const T* camera_T(&c_p1[0]);
    const T* target_T(&c_p2[0]);
    T TtoL[6];
    T TtoC[6];
    T          linki_T[6];
    T camera_point[3];
    
    extractPoseExtrinsics(link_posei_, linki_T);
    extrinsicsMult(linki_T,target_T,TtoL);
    extrinsicsMult(camera_T,TtoL,TtoC);
    eTransformPoint(TtoC, point, camera_point);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidual(camera_point, circle_diameter, TtoC, fx, fy, cx, cy, ox, oy, residual);

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
    const double* camera_T(&c_p1[0]);
    const double* target_T(&c_p2[0]);
    double TtoC[6];
    double linki_T[6];
    double TtoL[6];
    double camera_point[3];
    
    extractPoseExtrinsics(link_posei_,linki_T);
    extrinsicsMult(linki_T,target_T,TtoL);
    extrinsicsMult(camera_T,TtoL,TtoC);
    eTransformPoint3d(TtoC, point_, camera_point);

    /** compute project point into image plane and compute residual */
    double circle_diameter = circle_diameter_;
    double fx = fx_;
    double fy = fy_;
    double cx = cx_;
    double cy = cy_;
    double ox = ox_;
    double oy = oy_;

    cameraCircResidual(camera_point, circle_diameter, TtoC, fx, fy, cx, cy, ox, oy, residual);

  }
  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
                  const T* const c_p2, /** 6Dof transform of target into world frame [6] */
                  T* residual) const
  {
    const T* camera_T(&c_p1[0]);
    const T* target_T(&c_p2[0]);
    T linki_T[6];
    T TtoL[6];
    T TtoC[6];
    T camera_point[3];
    
    extractPoseExtrinsics(link_posei_, linki_T);
    extrinsicsMult(linki_T, target_T, TtoL);
    extrinsicsMult(camera_T, TtoL, TtoC);
    eTransformPoint3d(TtoC, point_, camera_point);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidual(camera_point, circle_diameter, TtoC, fx, fy, cx, cy, ox, oy, residual);

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

  void test_residual(const double* c_p1, double* residual)
  {
    const double* camera_T(&c_p1[0]);
    double mount_T[6];
    double MtoC[6];
    double camera_point[3];        /** point in camera coordinates */

    extractPoseExtrinsics(mount_to_target_pose_, mount_T);
    extrinsicsMult(camera_T, mount_T, MtoC);
    eTransformPoint3d(MtoC, point_, camera_point);

    /** compute project point into image plane and compute residual */
    double circle_diameter = circle_diameter_;
    double fx = fx_;
    double fy = fy_;
    double cx = cx_;
    double cy = cy_;
    double ox = ox_;
    double oy = oy_;
    cameraCircResidual(camera_point, circle_diameter, MtoC, fx, fy, cx, cy, ox, oy, residual);
  }
  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
                  T* residual) const
  {
    const T* camera_T(&c_p1[0]);
    T mount_T[6];
    T MtoC[6];
    T camera_point[3];        /** point in camera coordinates */
    
    extractPoseExtrinsics(mount_to_target_pose_, mount_T);
    extrinsicsMult(camera_T, mount_T, MtoC);
    eTransformPoint3d(MtoC, point_, camera_point);

    /** compute project point into image plane and compute residual */
    T circle_diameter = T(circle_diameter_);
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T ox = T(ox_);
    T oy = T(oy_);
    cameraCircResidual(camera_point, circle_diameter, MtoC, fx, fy, cx, cy, ox, oy, residual);

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
    const T* target_T(&c_p2[0]);  // extract target's angle axis

    /** transform point into camera frame */
    T camera_point[3]; /** point in camera coordinates */
    eTransformPoint3d(target_T, point_, camera_point);
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
    const T *target_T(& c_p2[0]); // extract target's angle axis
    
    /** transform point into camera frame */
    T camera_point[3]; /** point in camera coordinates */
    eTransformPoint3d(target_T, point_, camera_point);
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

// Estimate the axis of motion as part of the optimization for intrinic cal with camera on a rail
class RailICal4
{
public:
  RailICal4(double ob_x, double ob_y, double rail_position, Point3d point)
    : ox_(ob_x), oy_(ob_y), rail_position_(rail_position), point_(point)
  {}

  template<typename T>
  bool operator()(const T* const c_p1, // Intrinsics (9 params)
                  const T* const c_p2, // Target origin (6 params)
                  const T* const c_p3, // Camera skew (2 params)
                  T* residual) const
  {
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;      // extract intrinsics
    extractCameraIntrinsics(c_p1, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    const T *target_aa(& c_p2[0]); // extract target's angle axis
    const T *target_tx(& c_p2[3]); // extract target's position

    /** transform point into camera frame */
    T camera_point[3]; /** point in camera coordinates */
    transformPoint3d(target_aa, target_tx, point_, camera_point);
    
    // Now let's move the camera point by the rail travel
    // This involves two steps:
    // 1. Estimating the axis of motion (relative to camera z)
    T nominal_axis[3]; // Nominally we move back in camera z
    nominal_axis[0] = T(0.0);
    nominal_axis[1] = T(0.0);
    nominal_axis[2] = T(1.0);

    T rotation_axis[3]; // Here we skew the axis of motion w/ 2 rotation params
    rotation_axis[0] = c_p3[0];
    rotation_axis[1] = c_p3[1];
    rotation_axis[2] = T(0.0);

    T motion_axis[3];
    ceres::AngleAxisRotatePoint(rotation_axis, nominal_axis, motion_axis);

    // 2. Moving the camera back by that the skewed vector
    camera_point[0] = camera_point[0] + T(rail_position_) * motion_axis[0];
    camera_point[1] = camera_point[1] + T(rail_position_) * motion_axis[1];
    camera_point[2] = camera_point[2] + T(rail_position_) * motion_axis[2];

    /** compute project point into image plane and compute residual */
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidualDist(camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

    return true;
  }

  static ceres::CostFunction* Create(const double o_x, const double o_y, double rail_position, Point3d point)
  {
    return new ceres::AutoDiffCostFunction<RailICal4, 2, 9, 6, 2>(new RailICal4(o_x, o_y, rail_position, point));
  }

  double ox_; /** observed x location of object in image */
  double oy_; /** observed y location of object in image */
  double rail_position_; /** location of camera along rail */
  Point3d point_; /** point expressed in target coordinates */
};

// Estimate the axis of motion as part of the optimization for intrinic cal using target on a rail
class RailICal5
{
public:
  RailICal5(double ob_x, double ob_y, double rail_position, Point3d point)
    : ox_(ob_x), oy_(ob_y), rail_position_(rail_position), point_(point)
  {}

  template<typename T>
  bool operator()(const T* const c_p1, // Intrinsics (9 params)
                  const T* const c_p2, // Target origin (6 params)
                  const T* const c_p3, // Camera skew (2 params)
                  T* residual) const
  {
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;      // extract intrinsics
    extractCameraIntrinsics(c_p1, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    const T *target_aa(& c_p2[0]); // extract target's angle axis
    const T *target_tx(& c_p2[3]); // extract target's position

    // 1. Estimating the axis of motion (relative to initial target pose)
    T nominal_axis[3]; // Nominally we move back in camera z
    nominal_axis[0] = T(0.0);
    nominal_axis[1] = T(0.0);
    nominal_axis[2] = T(1.0);

    T rotation_axis[3]; // Here we skew the axis of motion w/ 2 rotation params
    rotation_axis[0] = c_p3[0];
    rotation_axis[1] = c_p3[1];
    rotation_axis[2] = T(0.0);

    T motion_axis[3];
    ceres::AngleAxisRotatePoint(rotation_axis, nominal_axis, motion_axis);

    // 2. Move the target point backwards along the rail
    T rail_point[3]; /** point in camera coordinates */
    rail_point[0] = point_.x - T(rail_position_) * motion_axis[0];
    rail_point[1] = point_.y - T(rail_position_) * motion_axis[1];
    rail_point[2] = point_.z - T(rail_position_) * motion_axis[2];

    /** transform point in rail coordinates into camera frame */
    T camera_point[3]; /** point in camera coordinates */
    transformPoint(target_aa, target_tx, rail_point, camera_point);

    /** compute project point into image plane and compute residual */
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidualDist(camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

    return true;
  }

  static ceres::CostFunction* Create(const double o_x, const double o_y, double rail_position, Point3d point)
  {
    return new ceres::AutoDiffCostFunction<RailICal5, 2, 9, 6, 2>(new RailICal5(o_x, o_y, rail_position, point));
  }

  double ox_; /** observed x location of object in image */
  double oy_; /** observed y location of object in image */
  double rail_position_; /** location of camera along rail */
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
    const T *C1toC2_T(& c_p1[0]); // extract right camera's angle axis
    const T *target_T(& c_p2[0]); // extract target's angle axis
    
    /** transform both points into left camera frame, note, that usually these would be exactly the same point.
	However, the camera observer may not provide observations in the same order. */
    T left_camera_point[3]; 
    T right_point_in_left_frame[3]; 
    eTransformPoint3d(target_T, right_point_, right_point_in_left_frame);
    eTransformPoint3d(target_T, left_point_, left_camera_point);
    left_camera_point[0] = left_camera_point[0] + T(rail_position_.x); // transform to camera's location along rail
    left_camera_point[1] = left_camera_point[1] + T(rail_position_.y); // transform to camera's location along rail
    left_camera_point[2] = left_camera_point[2] + T(rail_position_.z); // transform to camera's location along rail
    right_point_in_left_frame[0] = right_point_in_left_frame[0] + T(rail_position_.x); // transform to camera's location along rail
    right_point_in_left_frame[1] = right_point_in_left_frame[1] + T(rail_position_.y); // transform to camera's location along rail
    right_point_in_left_frame[2] = right_point_in_left_frame[2] + T(rail_position_.z); // transform to camera's location along rail
    
    /** transform right point in right camera frame */
    T right_camera_point[3]; 
    eTransformPoint(C1toC2_T, right_point_in_left_frame, right_camera_point);
    
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

  /* use to compute the pose between two intrinsically calibrated cameras used as a stereo pair when eiither
   *  the camera or the target is on the wrist of a robot. The transform from the left camera to the target
   *  is provided as an initial guess for each scene
   *  For N scenes there will be N*6 target pose parameters and 6 camera to camera pose parameters
   */
class  WristStereoCal
{
public:
  WristStereoCal(double left_ob_x,  double left_ob_y,  Point3d left_point,
		      double right_ob_x, double right_ob_y, Point3d right_point,
		      double lfx, double lfy, double lcx, double lcy,
		      double rfx, double rfy, double rcx, double rcy) :
    lox_(left_ob_x),   loy_(left_ob_y),   left_point_(left_point),
    rox_(right_ob_x),  roy_(right_ob_y),  right_point_(right_point),
    lfx_(lfx), lfy_(lfy), lcx_(lcx), lcy_(lcy),
    rfx_(rfx), rfy_(rfy), rcx_(rcx), rcy_(rcy)
  {
  }
  
  template<typename T>
  bool operator()(	    const T* const c_p1,  /** target_pose_relative to left camera[6] */
			    const T* const c_p2,  /** left camera relative to right camera[6] */
			    T* residual) const    /** this residual has 4 terms one for each camera */
  {
    const T *target_T(& c_p1[0]); // extract target's extrinsics
    const T *Stereo_T(& c_p2[0]); // extract right_camera extrinsics
    
    // Even though both cameras observe the same target and therefore have the same number of observations,
    // we don't assume the points observed are in the same order. Point1 will likely be the same as point2
    T left_camera_point[3]; // point observed by left camera in left camera frame
    T left_camera_point2[3]; // point observed by right camera in left frame
    T right_camera_point[3]; // point observed by right camera in right camera fraem
    eTransformPoint3d(target_T, left_point_, left_camera_point);// transform both points into left frame, probably the same point twice
    eTransformPoint3d(target_T, right_point_,left_camera_point2);
    eTransformPoint(Stereo_T, left_camera_point2, right_camera_point);    /** transform right point in right camera frame */

    /** project point into image plane and compute residual in both images*/
    T lox = T(lox_);
    T loy = T(loy_);
    T rox = T(rox_);
    T roy = T(roy_);
    T lfx = T(lfx_);
    T lfy = T(lfy_);
    T lcx = T(lcx_);
    T lcy = T(lcy_);
    T rfx = T(rfx_);
    T rfy = T(rfy_);
    T rcx = T(rcx_);
    T rcy = T(rcy_);
    cameraPntResidual(left_camera_point, lfx, lfy, lcx, lcy, lox, loy,  &(residual[0]));
    cameraPntResidual(right_camera_point,rfx, rfy, rcx, rcy, rox, roy,  &(residual[2]));			   
    return true;
  } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
  static ceres::CostFunction* Create(const double lo_x, const double lo_y, Point3d lpoint,
				     const double ro_x, const double ro_y, Point3d rpoint,
				     const double lfx, const double lfy,
				     const double lcx, const double lcy,
				     const double rfx, const double rfy,
				     const double rcx, const double rcy)
  {
    return (new ceres::AutoDiffCostFunction<WristStereoCal, 4, 6, 6>
	    (
	     new WristStereoCal(lo_x, lo_y, lpoint,
				     ro_x, ro_y, rpoint,
				     lfx, lfy, lcx, lcy,
				     rfx, rfy, rcx, rcy)
	     )
	    );
  }
  double lox_; /** observed x location of left_point in left image */
  double loy_; /** observed y location of left_point in left image */
  double rox_; /** observed x location of right point in right image */
  double roy_; /** observed x location of right point in right image */
  Point3d left_point_; /** left point expressed in target coordinates */
  Point3d right_point_; /** right point expressed in target coordinates */
  double lfx_,lfy_,lcx_,lcy_;/** left camera intrinsics */
  double rfx_,rfy_,rcx_,rcy_; /** right camera intrinsics */
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
    const T *target_T(& c_p1[0]); // extract target's extrinsics
    
    /** transform both points into left camera frame, note, that usually these would be exactly the same point.
	However, the camera observer may not provide observations in the same order. */
    T left_camera_point[3]; 
    T right_point_in_left_frame[3]; 
    eTransformPoint3d(target_T, right_point_, right_point_in_left_frame);
    eTransformPoint3d(target_T, left_point_, left_camera_point);

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

  // used to locate a stereo pair in world using a target on the end of arm tooling (right camera version)
class  TargetOnLinkRtStereo
{
public:
  TargetOnLinkRtStereo( double ob_x, double ob_y, Point3d point, Pose6d C1toC2, Pose6d ForwardK,
		        double fx, double fy, double cx, double cy, double k1, double k2, double k3, double p1, double p2) :
    ox_(ob_x),  oy_(ob_y),  point_(point),
    C1toC2_(C1toC2), ForwardK_(ForwardK),
    fx_(fx), fy_(fy), cx_(cx), cy_(cy), k1_(k1), k2_(k2), k3_(k3), p1_(p1), p2_(p2)
  {
  }
  
  template<typename T>
  bool operator()(	    const T* const c_p1,  /** target_pose[6] */
			    const T* const c_p2,  /** left_camera_extrinsics_pose[6] */
			    T* residual) const    /** this residual has 4 terms one for each camera */
  {
    const T *target_ex(& c_p1[0]); // extract target's extrinsics
    const T *Left_C_ex(& c_p2[0]); // left camera's extrinsics
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T k1 = T(k1_);
    T k2 = T(k2_);
    T k3 = T(k3_);	    
    T p1 = T(p1_);
    T p2 = T(p2_);
    T ox = T(ox_);
    T oy = T(oy_);
   
    /** transform point into right camera frame **/
    T tool_point[3];
    T world_point[3];
    T left_camera_point[3];
    T right_camera_point[3]; 
    eTransformPoint3d(target_ex, point_, tool_point);
    poseTransformPoint(ForwardK_, tool_point, world_point);
    eTransformPoint(Left_C_ex, world_point, left_camera_point);
    poseTransformPoint(C1toC2_, left_camera_point, right_camera_point);

    /** compute project point into image plane and compute residual */
    cameraPntResidualDist(right_camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);			   

    return true;
  } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, Point3d point,
				     Pose6d C1toC2, Pose6d ForwardK,
				     const double fx, const double fy,
				     const double cx, const double cy,
				     const double k1, const double k2, const double k3,
				     const double p1, const double p2)
  {
    return (new ceres::AutoDiffCostFunction<TargetOnLinkRtStereo, 2, 6, 6>
	    (
	     new TargetOnLinkRtStereo(o_x, o_y, point,
				      C1toC2, ForwardK,
				      fx, fy, cx, cy, k1, k2, k3, p1, p2)
	     )
	    );
  }
  double ox_; /** observed x location */
  double oy_; /** observed y location */
  Pose6d C1toC2_; /** transforms points expressed in left camera frame into right camera frame */
  Pose6d ForwardK_; /** transforms points expressed in tool to world */
  Point3d point_; /** point expressed in target coordinates */
  double fx_,fy_,cx_,cy_,k1_,k2_,k3_,p1_,p2_; /** camera intrinsics */
};

  // used to locate a stereo pair in world using a target on the end of arm tooling (right camera version)
class  TargetOnLinkLtStereo
{
public:
  TargetOnLinkLtStereo( double ob_x, double ob_y, Point3d point, Pose6d ForwardK,
		        double fx, double fy, double cx, double cy, double k1, double k2, double k3, double p1, double p2) :
    ox_(ob_x),  oy_(ob_y),  point_(point),
    ForwardK_(ForwardK),
    fx_(fx), fy_(fy), cx_(cx), cy_(cy), k1_(k1), k2_(k2), k3_(k3), p1_(p1), p2_(p2)
  {
  }
  
  template<typename T>
  bool operator()(	    const T* const c_p1,  /** target_pose[6] */
			    const T* const c_p2,  /** left_camera_extrinsics_pose[6] */
			    T* residual) const    /** this residual has 4 terms one for each camera */
  {
    const T *target_ex(& c_p1[0]); // extract target's extrinsics
    const T *Left_C_ex(& c_p2[0]); // left camera's extrinsics
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T k1 = T(k1_);
    T k2 = T(k2_);
    T k3 = T(k3_);	    
    T p1 = T(p1_);
    T p2 = T(p2_);
    T ox = T(ox_);
    T oy = T(oy_);
    
    /** transform point into left camera frame **/
    T tool_point[3];
    T world_point[3];
    T left_camera_point[3];
    eTransformPoint3d(target_ex, point_, tool_point);
    poseTransformPoint(ForwardK_, tool_point, world_point);
    eTransformPoint(Left_C_ex, world_point, left_camera_point);

    /** compute project point into image plane and compute residual */
    cameraPntResidualDist(left_camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);			   

    return true;
  } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, Point3d point,
				     Pose6d ForwardK,
				     const double fx, const double fy,
				     const double cx, const double cy,
				     const double k1, const double k2, const double k3,
				     const double p1, const double p2)
  {
    return (new ceres::AutoDiffCostFunction<TargetOnLinkLtStereo, 2, 6, 6>
	    (
	     new TargetOnLinkLtStereo(o_x, o_y, point,
				      ForwardK,
				      fx, fy, cx, cy, k1, k2, k3, p1, p2)
	     )
	    );
  }
  double ox_; /** observed x location */
  double oy_; /** observed y location */
  Pose6d ForwardK_; /** transforms points expressed in tool to world */
  Point3d point_; /** point expressed in target coordinates */
  double fx_,fy_,cx_,cy_,k1_,k2_,k3_,p1_,p2_; /** camera intrinsics */
};


  // used to locate a stereo pair on link using a target on the end of arm tooling (right camera version)
class  StereoOnLinkRt
{
public:
  StereoOnLinkRt( double ob_x, double ob_y, Point3d point, Pose6d C1toC2, Pose6d ForwardK,
		        double fx, double fy, double cx, double cy, double k1, double k2, double k3, double p1, double p2) :
    ox_(ob_x),  oy_(ob_y),  point_(point),
    C1toC2_(C1toC2), 
    fx_(fx), fy_(fy), cx_(cx), cy_(cy), k1_(k1), k2_(k2), k3_(k3), p1_(p1), p2_(p2)
  {
    ForwardK_inv_ = ForwardK.getInverse();
  }
  
  template<typename T>
  bool operator()(	    const T* const c_p1,  /** target_pose[6] */
			    const T* const c_p2,  /** left_camera_extrinsics_pose[6] */
			    T* residual) const    /** this residual has 4 terms one for each camera */
  {
    const T *target_ex(& c_p1[0]); // extract target's extrinsics
    const T *Left_C_ex(& c_p2[0]); // left camera's extrinsics
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T k1 = T(k1_);
    T k2 = T(k2_);
    T k3 = T(k3_);	    
    T p1 = T(p1_);
    T p2 = T(p2_);
    T ox = T(ox_);
    T oy = T(oy_);
    
    /** transform point into right camera frame **/
    T world_point[3];
    T tool_point[3];
    T left_camera_point[3];
    T right_camera_point[3]; 
    eTransformPoint3d(target_ex, point_, world_point);
    poseTransformPoint(ForwardK_inv_, world_point, tool_point);
    eTransformPoint(Left_C_ex, tool_point, left_camera_point);
    poseTransformPoint(C1toC2_, left_camera_point, right_camera_point);

    /** compute project point into image plane and compute residual */
    cameraPntResidualDist(right_camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);			   

    return true;
  } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, Point3d point,
				     Pose6d C1toC2, Pose6d ForwardK,
				     const double fx, const double fy,
				     const double cx, const double cy,
				     const double k1, const double k2, const double k3,
				     const double p1, const double p2)
  {
    return (new ceres::AutoDiffCostFunction<StereoOnLinkRt, 2, 6, 6>
	    (
	     new StereoOnLinkRt(o_x, o_y, point,
				C1toC2, ForwardK,
				fx, fy, cx, cy, k1, k2, k3, p1, p2)
	     )
	    );
  }
  double ox_; /** observed x location */
  double oy_; /** observed y location */
  Pose6d C1toC2_; /** transforms points expressed in left camera frame into right camera frame */
  Pose6d ForwardK_inv_; /** transforms points expressed in tool to world */
  Point3d point_; /** point expressed in target coordinates */
  double fx_,fy_,cx_,cy_,k1_,k2_,k3_,p1_,p2_; /** camera intrinsics */
};

  // used to locate a stereo pair on link using a target on the end of arm tooling (left camera version)
class  StereoOnLinkLt
{
public:
  StereoOnLinkLt( double ob_x, double ob_y, Point3d point, Pose6d ForwardK,
		        double fx, double fy, double cx, double cy, double k1, double k2, double k3, double p1, double p2) :
    ox_(ob_x),  oy_(ob_y),  point_(point),
    fx_(fx), fy_(fy), cx_(cx), cy_(cy), k1_(k1), k2_(k2), k3_(k3), p1_(p1), p2_(p2)
  {
    ForwardK_inv_ = ForwardK.getInverse();
  }
  
  template<typename T>
  bool operator()(	    const T* const c_p1,  /** target_pose[6] */
			    const T* const c_p2,  /** left_camera_extrinsics_pose[6] */
			    T* residual) const    /** this residual has 4 terms one for each camera */
  {
    const T *target_ex(& c_p1[0]); // extract target's extrinsics
    const T *Left_C_ex(& c_p2[0]); // left camera's extrinsics
    T fx = T(fx_);
    T fy = T(fy_);
    T cx = T(cx_);
    T cy = T(cy_);
    T k1 = T(k1_);
    T k2 = T(k2_);
    T k3 = T(k3_);	    
    T p1 = T(p1_);
    T p2 = T(p2_);
    T ox = T(ox_);
    T oy = T(oy_);
    
    /** transform point into left camera frame **/
    T world_point[3];
    T tool_point[3];
    T left_camera_point[3];
    eTransformPoint3d(target_ex, point_, world_point);
    poseTransformPoint(ForwardK_inv_, world_point, tool_point);
    eTransformPoint(Left_C_ex, tool_point, left_camera_point);

    /** compute project point into image plane and compute residual */
    cameraPntResidualDist(left_camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);			   

    return true;
  } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, Point3d point,
				     Pose6d ForwardK,
				     const double fx, const double fy,
				     const double cx, const double cy,
				     const double k1, const double k2, const double k3,
				     const double p1, const double p2)
  {
    return (new ceres::AutoDiffCostFunction<StereoOnLinkLt, 2, 6, 6>
	    (
	     new StereoOnLinkLt(o_x, o_y, point,
				ForwardK,
				fx, fy, cx, cy, k1, k2, k3, p1, p2)
	     )
	    );
  }
  double ox_; /** observed x location */
  double oy_; /** observed y location */
  Pose6d ForwardK_inv_; /** transforms points expressed in tool to world */
  Point3d point_; /** point expressed in target coordinates */
  double fx_,fy_,cx_,cy_,k1_,k2_,k3_,p1_,p2_; /** camera intrinsics */
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
    const T *camera_T(&c_p1[0]);
    T camera_point[3]; /** point in camera coordinates */

    /** transform point into camera coordinates */
    eTransformPoint3d(camera_T, point_, camera_point);

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
    const T *target_T(& c_p2[0]); // extract target's angle axis

    /** transform point into camera frame */
    T camera_point[3]; /** point in camera coordinates */
    eTransformPoint3d(target_T, point_, camera_point);
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
    const T *camera_T(& c_p1[0]); // extract camera's extrinsics

    /** transform point into camera frame */
    T camera_point[3]; /** point in camera coordinates */
    eTransformPoint3d(camera_T, point_, camera_point);

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

  /* @brief used to calibrate a camera mounted on wrist and the joint offsets and the pose of the target in a workcell */
class  CameraOnWristWithJointOffsets
{
public:
  CameraOnWristWithJointOffsets(double ob_x, double ob_y,
				std::vector<DHParameters> DH,
				Pose6d& target_IT,
				Pose6d& robot_IT,
				double fx, double fy,
				double cx, double cy,
				double k1, double k2, double k3,
				double p1, double p2,
				Point3d point) :
    ox_(ob_x), oy_(ob_y), 
    target_IT_(target_IT),
    robot_IT_(robot_IT),
    fx_(fx), fy_(fy), cx_(cx), cy_(cy),
    k1_(k1), k2_(k2), k3_(k3), p1_(p1), p2_(p2),
    point_(point)
  {
    N_joints_ = DH.size();
    for(int i=0; i<N_joints_; i++){
      DH_.push_back(DH[i]);
    }
  }

  template<typename T>
  bool operator()(	    const T* const c_p1,  /** target extriniscs [6] */
			    const T* const c_p2,  /** camera extrinsics [6] */
			    const T* const c_p3,  /** joint offsets, one for each joint */
			    T* residual) const
  {
    const T *target_T(& c_p1[0]); // extract target's extrinsics
    const T *camera_T(& c_p2[0]); // extract camera's extrinsics

    T target_mt_point[3];  /** point in frame of target's mounting frame */
    T ref_point[3];        /** point in frame of ref_frame               */
    T camera_point[3];     /** point in frame of camera's optical frame  */
    T linki_point[3];      /** point in linki frame                      */
    T link_ip1_point[3];   /** point in link i+1 frame                   */

    eTransformPoint3d(target_T, point_, target_mt_point);      // point in target's mounting_frame
    poseTransformPoint(target_IT_, target_mt_point, ref_point); // point in ref_frame
    poseTransformPoint(robot_IT_, ref_point, linki_point);     // point in the robot base frame 
    for(int i=0;i<N_joints_;i++){                              // transform point up each link until it reaches the camera
      T a(DH_[i].a);
      T alpha(DH_[i].alpha);
      T d(DH_[i].d);
      T theta(DH_[i].theta);
      dhInvTransformPoint(a, alpha, d, theta, linki_point, link_ip1_point);
      linki_point[0] = link_ip1_point[0];
      linki_point[1] = link_ip1_point[1];
      linki_point[2] = link_ip1_point[2];
    }
    eTransformPoint(camera_T, linki_point, camera_point);    // point in camera's optical frame

    /** compute residual */
    T fx(fx_);
    T fy(fy_);
    T cx(cx_);
    T cy(cy_);
    T k1(k1_);
    T k2(k2_);
    T k3(k3_);	
    T p1(p1_);
    T p2(p2_);
    T ox(ox_);
    T oy(oy_);
    cameraPntResidualDist(camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
  static ceres::CostFunction* Create(double ob_x, double ob_y,
				std::vector<DHParameters> DH,
				Pose6d& target_IT,
				Pose6d& robot_IT,
				double fx, double fy,
				double cx, double cy,
				double k1, double k2, double k3,
				double p1, double p2,
				Point3d point) 

  {
    return (new ceres::AutoDiffCostFunction<CameraOnWristWithJointOffsets, 2, 6, 6, 5 >
	    (
	     new CameraOnWristWithJointOffsets(ob_x, ob_y,
					       DH,
					       target_IT,
					       robot_IT,
					       fx, fy,
					       cx,  cy,
					       k1,  k2,  k3,
					       p1,  p2,
					       point) 
	     )
	    );
  }
  double ox_; /** observed x location of object in 3D data */
  double oy_; /** observed y location of object in 3D data */
  Point3d point_; /** point expressed in target coordinates */
  int N_joints_;
  // DH parameters for robot with up to 12 joints
  std::vector<DHParameters> DH_; // link dh parameters
  Pose6d target_IT_; // transform from target mounting point to ref_frame
  Pose6d robot_IT_; // transform from base of robot to ref_frame
  double fx_; /*!< known focal length of camera in x */
  double fy_; /*!< known focal length of camera in y */
  double cx_; /*!< known optical center of camera in x */
  double cy_; /*!< known optical center of camera in y */
  double k1_; /*!< known radial distorition k1 */
  double k2_; /*!< known radial distorition k2 */
  double k3_; /*!< known radial distorition k3 */
  double p1_; /*!< known decentering distorition p1 */
  double p2_; /*!< known decentering distorition p2 */
};


} // end of namespace
#endif
