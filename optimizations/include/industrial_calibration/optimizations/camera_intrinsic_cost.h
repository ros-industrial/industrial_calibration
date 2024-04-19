#pragma once

#include <industrial_calibration/core/types.h>
#include <industrial_calibration/optimizations/ceres_math_utilities.h>

namespace industrial_calibration
{
/**
 * @brief Cost function for performing 2D camera intrinsic calibration
 */
class CameraIntrinsicCost
{
public:
  CameraIntrinsicCost(const Eigen::Vector3d& in_target, const Eigen::Vector2d& in_image)
    : in_target_(in_target), in_image_(in_image)
  {
  }

  template <typename T>
  bool operator()(const T* const target_pose, const T* const camera_intr, T* const residual) const
  {
    const T* target_angle_axis = target_pose + 0;
    const T* target_position = target_pose + 3;

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(in_target_(0));
    target_pt[1] = T(in_target_(1));
    target_pt[2] = T(in_target_(2));

    T camera_point[3];  // Point in camera coordinates
    transformPoint(target_angle_axis, target_position, target_pt, camera_point);

    T xy_image[2];
    projectPoints2(camera_intr, camera_point, xy_image);

    residual[0] = xy_image[0] - in_image_.x();
    residual[1] = xy_image[1] - in_image_.y();

    return true;
  }

private:
  Eigen::Vector3d in_target_;
  Eigen::Vector2d in_image_;
};

}  // namespace industrial_calibration
