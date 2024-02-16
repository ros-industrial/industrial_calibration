#pragma once

#include <industrial_calibration/optimizations/ceres_math_utilities.h>
#include <industrial_calibration/core/types.h>

#include <Eigen/Geometry>

namespace industrial_calibration
{
/**
 * @brief Base class for hand-eye calibration cost functions
 */
template <Eigen::Index OBS_DIMENSION>
class ExtrinsicHandEyeCost
{
public:
  /**
   * @brief A Ceres cost function class that represents a single observation of a 3D camera and target
   * @param obs - The observation of a feature in the camera frame
   * @param camera_mount_to_base - The transform from the camera "mount" frame to the common base frame
   * @param base_to_target_mount - The transform from the common base frame to the target "mount" frame
   * @param point_in_target - The corresponding feature in the target frame
   */
  ExtrinsicHandEyeCost(const Eigen::Matrix<double, OBS_DIMENSION, 1>& obs,
                       const Eigen::Isometry3d& camera_mount_to_base, const Eigen::Isometry3d& base_to_target_mount,
                       const Eigen::Vector3d& point_in_target)
    : obs_(obs)
    , camera_mount_to_base_(poseEigenToCal(camera_mount_to_base))
    , base_to_target_mount_(poseEigenToCal(base_to_target_mount))
    , target_pt_(point_in_target)
  {
  }

  template <typename T>
  bool operator()(const T* pose_camera_to_camera_mount, const T* pose_target_mount_to_target, T* residual) const;

  template <typename T>
  void getTargetPointInCamera(const T* pose_camera_to_camera_mount, const T* pose_target_mount_to_target,
                              T* camera_point) const
  {
    const T* camera_angle_axis = pose_camera_to_camera_mount + 0;
    const T* camera_position = pose_camera_to_camera_mount + 3;

    const T* target_angle_axis = pose_target_mount_to_target + 0;
    const T* target_position = pose_target_mount_to_target + 3;

    T target_mount_point[3];  // Point in target mount coordinates
    T world_point[3];         // Point in world coordinates
    T camera_mount_point[3];  // Point in camera mount coordinates

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(target_pt_(0));
    target_pt[1] = T(target_pt_(1));
    target_pt[2] = T(target_pt_(2));
    transformPoint(target_angle_axis, target_position, target_pt, target_mount_point);
    poseTransformPoint(base_to_target_mount_, target_mount_point, world_point);
    poseTransformPoint(camera_mount_to_base_, world_point, camera_mount_point);
    transformPoint(camera_angle_axis, camera_position, camera_mount_point, camera_point);
  }

protected:
  Eigen::Matrix<double, OBS_DIMENSION, 1> obs_;
  Pose6d camera_mount_to_base_;
  Pose6d base_to_target_mount_;
  Eigen::Vector3d target_pt_;
};

/**
 * @brief Cost function for a hand-eye calibration using 2D observations of 3D features
 */
class ExtrinsicHandEye2D3DCost : public ExtrinsicHandEyeCost<2>
{
public:
  ExtrinsicHandEye2D3DCost(const Eigen::Vector2d& obs, const Eigen::Isometry3d& camera_mount_to_base,
                           const Eigen::Isometry3d& base_to_target_mount, const Eigen::Vector3d& point_in_target,
                           const CameraIntrinsics& intr)
    : ExtrinsicHandEyeCost(obs, camera_mount_to_base, base_to_target_mount, point_in_target), intr_(intr)
  {
  }

  template <typename T>
  bool operator()(const T* pose_camera_to_camera_mount, const T* pose_target_mount_to_target, T* residual) const
  {
    // Get the point in the frame of the camera
    T camera_point[3];
    getTargetPointInCamera(pose_camera_to_camera_mount, pose_target_mount_to_target, camera_point);

    // Project the point into the image
    T xy_image[2];
    projectPoint(intr_, camera_point, xy_image);

    // Calculate the residual error
    residual[0] = xy_image[0] - obs_.x();
    residual[1] = xy_image[1] - obs_.y();

    return true;
  }

private:
  CameraIntrinsics intr_;
};

/**
 * @brief Cost function for a hand-eye calibration using 3D observations of 3D features
 */
class ExtrinsicHandEye3D3DCost : public ExtrinsicHandEyeCost<3>
{
public:
  using ExtrinsicHandEyeCost<3>::ExtrinsicHandEyeCost;

  template <typename T>
  bool operator()(const T* pose_camera_to_camera_mount, const T* pose_target_mount_to_target, T* residual) const
  {
    // Get the target point in the frame of the camera
    T camera_point[3];
    getTargetPointInCamera(pose_camera_to_camera_mount, pose_target_mount_to_target, camera_point);

    // Calculate the residual error
    residual[0] = camera_point[0] - obs_.x();
    residual[1] = camera_point[1] - obs_.y();
    residual[2] = camera_point[2] - obs_.z();

    return true;
  }
};

}  // namespace industrial_calibration
