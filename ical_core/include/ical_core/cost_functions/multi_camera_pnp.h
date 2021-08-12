#pragma once

#include <ical_core/ceres_math_utilities.h>
#include <ical_core/types.h>

#include <Eigen/Geometry>

namespace industrial_calibration
{
/**
 * @brief Cost function for multi-camera perspective-n-point estimation
 */
class MultiCameraPnPCost
{
public:
  MultiCameraPnPCost(const Eigen::Vector2d& obs, const CameraIntrinsics& intr, const Eigen::Isometry3d& camera_to_base,
                     const Eigen::Vector3d& point_in_target)
    : obs_(obs), intr_(intr), camera_to_base_pose_(poseEigenToCal(camera_to_base)), target_pt_(point_in_target)
  {
  }

  template <typename T>
  bool operator()(const T* const pose_base_to_target, T* residual) const
  {
    const T* target_angle_axis = pose_base_to_target + 0;
    const T* target_position = pose_base_to_target + 3;

    T world_point[3];   // Point in world coordinates (base of robot)
    T camera_point[3];  // Point in camera coordinates

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(target_pt_(0));
    target_pt[1] = T(target_pt_(1));
    target_pt[2] = T(target_pt_(2));

    transformPoint(target_angle_axis, target_position, target_pt, world_point);
    poseTransformPoint(camera_to_base_pose_, world_point, camera_point);

    // Compute projected point into image plane and compute residual
    T xy_image[2];
    projectPoint(intr_, camera_point, xy_image);

    residual[0] = xy_image[0] - obs_.x();
    residual[1] = xy_image[1] - obs_.y();

    return true;
  }

private:
  Eigen::Vector2d obs_;
  CameraIntrinsics intr_;
  Pose6d camera_to_base_pose_;
  Eigen::Vector3d target_pt_;
};

}  // namespace industrial_calibration
