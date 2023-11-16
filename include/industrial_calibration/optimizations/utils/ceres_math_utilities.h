#pragma once

#include <industrial_calibration/types.h>
#include <industrial_calibration/camera_intrinsics.h>
#include <industrial_calibration/pose_6d.h>

#include <ceres/rotation.h>

namespace industrial_calibration
{
template <typename T>
inline void transformPoint(const T angle_axis[3], const T tx[3], const T point[3], T t_point[3])
{
  ceres::AngleAxisRotatePoint(angle_axis, point, t_point);

  t_point[0] = t_point[0] + tx[0];
  t_point[1] = t_point[1] + tx[1];
  t_point[2] = t_point[2] + tx[2];
}

template <typename T>
inline void poseTransformPoint(const Pose6d& pose, const T point[3], T t_point[3])
{
  T angle_axis[3];

  angle_axis[0] = T(pose.rx());
  angle_axis[1] = T(pose.ry());
  angle_axis[2] = T(pose.rz());

  T translation[3];
  translation[0] = T(pose.x());
  translation[1] = T(pose.y());
  translation[2] = T(pose.z());

  transformPoint(angle_axis, translation, point, t_point);
}

template <typename T>
inline void projectPoint(const CameraIntrinsics& intr, const T point[3], T xy_image[2])
{
  T xp1 = point[0];
  T yp1 = point[1];
  T zp1 = point[2];

  // Scale into the image plane by distance away from camera
  T xp, yp;

  if (zp1 == T(0))  // Avoid divide by zero
  {
    xp = xp1;
    yp = yp1;
  }
  else
  {
    xp = xp1 / zp1;
    yp = yp1 / zp1;
  }

  // Perform projection using focal length and camera optical center into image
  // plane
  xy_image[0] = intr.fx() * xp + intr.cx();
  xy_image[1] = intr.fy() * yp + intr.cy();
}

template <typename T>
inline Eigen::Matrix<T, 2, 1> projectPoint(const CameraIntrinsics& intr, const Eigen::Matrix<T, 3, 1>& point)
{
  // Scale the input point by its distance from the camera (i.e. z-coordinate)
  Eigen::Matrix<T, 3, 1> scaled_point(point);
  scaled_point(2) = T(1.0);

  // Avoid divide by zero
  if (abs(point.z() - T(0.0)) > T(1.e-10))
  {
    scaled_point.x() = point.x() / point.z();
    scaled_point.y() = point.y() / point.z();
  }

  /* Create the camera parameters matrix
   * | fx  0   cx | * | x/z | = | u |
   * | 0   fy  cy | * | y/z | = | v |
   * | 0   0   1  | * |  1  | = | 1 |
   */
  Eigen::Matrix3d camera_matrix;
  camera_matrix << intr.fx(), 0.0, intr.cx(), 0.0, intr.fy(), intr.cy(), 0.0, 0.0, 1.0;

  // Perform projection using focal length and camera optical center into image plane
  Eigen::Matrix<T, 3, 1> image_pt = camera_matrix.cast<T>() * scaled_point;

  // Return only the top two elements of the vector
  return image_pt.template head<2>();
}

template <typename T>
void projectPoints2(const T* const camera_intr, const T* const pt_in_camera, T* pt_in_image)
{
  T xp1 = pt_in_camera[0];
  T yp1 = pt_in_camera[1];
  T zp1 = pt_in_camera[2];

  CalibCameraIntrinsics<T> intr(camera_intr);

  // Scale into the image plane by distance away from camera
  T xp;
  T yp;
  if (zp1 == T(0))  // Avoid dividing by zero.
  {
    xp = xp1;
    yp = yp1;
  }
  else
  {
    xp = xp1 / zp1;
    yp = yp1 / zp1;
  }

  // Temporary variables for distortion model.
  T xp2 = xp * xp;   // x^2
  T yp2 = yp * yp;   // y^2
  T r2 = xp2 + yp2;  // r^2 radius squared
  T r4 = r2 * r2;    // r^4
  T r6 = r2 * r4;    // r^6

  // Apply the distortion coefficients to refine pixel location
  T xpp = xp + intr.k1() * r2 * xp           // 2nd order term
          + intr.k2() * r4 * xp              // 4th order term
          + intr.k3() * r6 * xp              // 6th order term
          + intr.p2() * (r2 + T(2.0) * xp2)  // tangential
          + intr.p1() * xp * yp * T(2.0);    // other tangential term

  T ypp = yp + intr.k1() * r2 * yp           // 2nd order term
          + intr.k2() * r4 * yp              // 4th order term
          + intr.k3() * r6 * yp              // 6th order term
          + intr.p1() * (r2 + T(2.0) * yp2)  // tangential term
          + intr.p2() * xp * yp * T(2.0);    // other tangential term

  // Perform projection using focal length and camera center into image plane
  pt_in_image[0] = intr.fx() * xpp + intr.cx();
  pt_in_image[1] = intr.fy() * ypp + intr.cy();
}

template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry> toIsometry(const T* angle_axis_ptr, const T* translation_ptr)
{
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Isometry3 = Eigen::Transform<T, 3, Eigen::Isometry>;

  Eigen::Map<const Vector3> angle_axis(angle_axis_ptr);
  Eigen::Map<const Vector3> translation(translation_ptr);
  Isometry3 pose = Isometry3::Identity();
  pose = Eigen::Translation<T, 3>(translation) * Eigen::AngleAxis<T>(angle_axis.norm(), angle_axis.normalized());
  return pose;
}

Pose6d poseEigenToCal(const Eigen::Isometry3d& pose);

Eigen::Isometry3d poseCalToEigen(const Pose6d& pose);

}  // namespace industrial_calibration
