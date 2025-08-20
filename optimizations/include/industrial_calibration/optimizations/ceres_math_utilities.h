#pragma once

#include <industrial_calibration/core/types.h>
#include <industrial_calibration/core/camera_intrinsics.h>
#include <industrial_calibration/core/pose_6d.h>
#include <industrial_calibration/core/utils.h>  // For backwards compatibility when functions from this file were moved

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

}  // namespace industrial_calibration
