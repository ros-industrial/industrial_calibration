#pragma once

#include <industrial_calibration/core/types.h>

#include <industrial_calibration/core/types.h>
#include <industrial_calibration/core/camera_intrinsics.h>
#include <industrial_calibration/core/pose_6d.h>

namespace industrial_calibration
{
/**
 * @brief Projects a 3D point (relative to the camera frame) into image coordinates
 */
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

  // Perform projection using focal length and camera optical center into image plane
  xy_image[0] = intr.fx() * xp + intr.cx();
  xy_image[1] = intr.fy() * yp + intr.cy();
}

/**
 * @brief Projects a 3D point (relative to the camera frame) into image coordinates
 */
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

/**
 * @brief Projects a vector of 3D points (relative to the camera frame) into a vector of image coordinates
 */
template <typename T>
VectorVector2<T> projectPoints(const Eigen::Transform<T, 3, Eigen::Isometry>& camera_to_target,
                               const CameraIntrinsics& intr, const VectorVector3<T>& target_points)
{
  VectorVector2<T> reprojections;
  for (const auto& point_in_target : target_points)
  {
    Eigen::Matrix<T, 3, 1> in_camera = camera_to_target * point_in_target;
    reprojections.push_back(projectPoint<T>(intr, in_camera));
  }
  return reprojections;
}

/**
 * @brief Projects a 3D point (relative to the camera) into image coordinates, accounting for image distortion
 * @param camera_intr Camera intrinsic parameters with distortion (size 9). See CalibCameraIntrinsics for details.
 * @param pt_in_camera 3D point in the camera frame (size 3)
 * @param pt_in_image (Output) 2D image coordinates (size 2)
 */
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

/*
 * @brief Computes the homography matrix of a set of correspondences
 * @details There is a 3x3 homography matrix, H, that can transform a point from one plane onto a different plane
 *
 * | u | = k * | H00 H01 H02 | * | x |
 * | v |       | H10 H11 H12 |   | y |
 * | 1 |       | H20 H12  1  |   | 1 |
 *
 * In our case we have 2 sets of known corresponding planar points: points on the planar target, and points in the
 * image plane Therefore, there is some matrix, H, which can transform target points into the image plane. If the
 * target points and camera points actually match, we should be able to:
 *   1. Calculate H for a subset of corresponding points
 *   2. Transform the remaining target points by H to obtain estimates of their locations in the image plane
 *   3. Compare the calculated estimations to the actual image points to make sure they are very close. If they are
 * not close, we know that the correspondences are not valid
 *
 * The matrix H has 8 unique values.
 * These 8 values of the homography matrix can be solved for, given a set of (at least) 8 corresponding planar
 * vectors, by rearranging the above equations:
 *
 * A * H = b, where
 * H = inv(A) * b
 *   - A is matrix (size 2*n x 8), where n is the number of corresponding vectors
 *   - H is a vector (size 8 x 1) of the unknown elements of the homography matrix
 *   - B is a vector (size 2*n x 1) representing the elements of one set of planar vectors
 *
 *                  A               *              H    =    b
 * |-x0 -y0  -1   0    0    0   u0*x0   u0*y0 | * | H00 | = | -u0 |
 * | 0   0   0  -x0   -y0  -1   v0*x0   v0*y0 | * | H01 | = | -v0 |
 *                              ...
 * |-x7 -y7  -1   0    0    0   u7*x7   u7*y7 | * | H20 | = | -u7 |
 * | 0   0   0  -x7   -y7  -1   v7*x7   v7*y7 | * | H21 | = | -v7 |
 *
 */
Eigen::Matrix<double, 3, 3, Eigen::RowMajor> calculateHomography(const Correspondence2D3D::Set correspondences);

/**
 * @brief Projects a set of planar 2D points in world coordinates into image coordinates using a homography matrix
 * @param H Homography matrix
 * @param xy 2D planar points in world coordinates
 */
Eigen::Matrix2Xd projectHomography(const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>& H, const Eigen::Matrix2Xd& xy);

}  // namespace industrial_calibration
