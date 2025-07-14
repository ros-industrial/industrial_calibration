#pragma once

#include <industrial_calibration/core/types.h>
#include <industrial_calibration/optimizations/ceres_math_utilities.h>

#include <Eigen/Geometry>

namespace industrial_calibration
{
/**
 * @brief Cost function for single camera perspective-n-point estimation
 * @ingroup optimizations_pnp
 */
struct PnPCost
{
  PnPCost(const CameraIntrinsics& intr, const Eigen::Vector3d& pt_in_target, const Eigen::Vector2d& pt_in_image)
    : intr_(intr), in_target_(pt_in_target), in_image_(pt_in_image)
  {
  }

  template <typename T>
  bool operator()(const T* const cam_to_tgt_angle_axis_ptr, const T* const cam_to_tgt_translation_ptr,
                  T* const residual) const
  {
    using Isometry3 = Eigen::Transform<T, 3, Eigen::Isometry>;
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using Vector2 = Eigen::Matrix<T, 2, 1>;

    Isometry3 camera_to_target = toIsometry(cam_to_tgt_angle_axis_ptr, cam_to_tgt_translation_ptr);

    // Transform points into camera coordinates
    Vector3 camera_pt = camera_to_target * in_target_.cast<T>();

    Vector2 xy_image = projectPoint(intr_, camera_pt);

    residual[0] = xy_image[0] - in_image_.x();
    residual[1] = xy_image[1] - in_image_.y();

    return true;
  }

  CameraIntrinsics intr_;
  Eigen::Vector3d in_target_;
  Eigen::Vector2d in_image_;
};

/**
 * @brief Cost function for single-camera perspective-n-point estimation using 3D features
 * @ingroup optimizations_pnp
 */
struct PnP3DCost
{
public:
  PnP3DCost(const Eigen::Vector3d& pt_in_target, const Eigen::Vector3d& pt_in_image)
    : in_target_(pt_in_target), in_image_(pt_in_image)
  {
  }

  template <typename T>
  bool operator()(const T* const cam_to_tgt_angle_axis_ptr, const T* const cam_to_tgt_translation_ptr,
                  T* const residual) const
  {
    using Isometry3 = Eigen::Transform<T, 3, Eigen::Isometry>;
    using Vector3 = Eigen::Matrix<T, 3, 1>;

    Isometry3 camera_to_target = toIsometry(cam_to_tgt_angle_axis_ptr, cam_to_tgt_translation_ptr);

    // Transform points into camera coordinates
    Vector3 camera_pt = camera_to_target * in_target_.cast<T>();

    residual[0] = camera_pt[0] - in_image_.x();
    residual[1] = camera_pt[1] - in_image_.y();
    residual[2] = camera_pt[2] - in_image_.z();
    return true;
  }

  Eigen::Vector3d in_target_;
  Eigen::Vector3d in_image_;
};

}  // namespace industrial_calibration
