#pragma once

#include <industrial_calibration/optimizations/utils/ceres_math_utilities.h>
#include <industrial_calibration/types.h>

#include <Eigen/Geometry>

namespace industrial_calibration
{
/**
 * @brief Cost function for extrinsic calibration of multiple statically mounted 2D cameras
 */
class ExtrinsicMultiStaticCameraCost
{
public:
  ExtrinsicMultiStaticCameraCost(const Eigen::Vector2d& obs, const CameraIntrinsics& intr,
                                 const Eigen::Isometry3d& base_to_wrist, const Eigen::Vector3d& point_in_target)
    : obs_(obs), intr_(intr), wrist_pose_(poseEigenToCal(base_to_wrist)), target_pt_(point_in_target)
  {
  }

  template <typename T>
  bool operator()(const T* const pose_camera_to_base, const T* pose_wrist_to_target, T* residual) const
  {
    const T* camera_angle_axis = pose_camera_to_base + 0;
    const T* camera_position = pose_camera_to_base + 3;

    const T* target_angle_axis = pose_wrist_to_target + 0;
    const T* target_position = pose_wrist_to_target + 3;

    T link_point[3];    // Point in wrist coordinates
    T world_point[3];   // Point in world coordinates (base of robot)
    T camera_point[3];  // Point in camera coordinates

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(target_pt_(0));
    target_pt[1] = T(target_pt_(1));
    target_pt[2] = T(target_pt_(2));

    transformPoint(target_angle_axis, target_position, target_pt, link_point);
    poseTransformPoint(wrist_pose_, link_point, world_point);
    transformPoint(camera_angle_axis, camera_position, world_point, camera_point);

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
  Pose6d wrist_pose_;
  Eigen::Vector3d target_pt_;
};

/**
 * @brief Cost function for extrinsic calibration of multiple statically mounted 2D cameras
 */
class ExtrinsicMultiStaticFreeCameraCost
{
public:
  ExtrinsicMultiStaticFreeCameraCost(const Eigen::Vector2d& obs, const CameraIntrinsics& intr,
                                     const Eigen::Vector3d& point_in_target)
    : obs_(obs), intr_(intr), target_pt_(point_in_target)
  {
  }

  template <typename T>
  bool operator()(const T* const pose_camera_to_base, const T* pose_base_to_target, T* residual) const
  {
    const T* camera_angle_axis = pose_camera_to_base + 0;
    const T* camera_position = pose_camera_to_base + 3;

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
    transformPoint(camera_angle_axis, camera_position, world_point, camera_point);

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
  Eigen::Vector3d target_pt_;
};

/**
 * @brief Cost function for extrinsic calibration of multiple statically mounted 2D cameras
 */
class ExtrinsicMultiStaticFixedCameraCost
{
public:
  ExtrinsicMultiStaticFixedCameraCost(const Eigen::Vector2d& obs, const CameraIntrinsics& intr,
                                      const Eigen::Isometry3d& base_to_camera, const Eigen::Vector3d& point_in_target)
    : obs_(obs), intr_(intr), camera_to_base_(poseEigenToCal(base_to_camera.inverse())), target_pt_(point_in_target)
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
    poseTransformPoint(camera_to_base_, world_point, camera_point);

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
  Pose6d camera_to_base_;
  Eigen::Vector3d target_pt_;
};

/**
 * @brief Cost function for extrinsic calibration of multiple statically mounted 2D cameras
 */
class ExtrinsicMultiStaticCameraWristOnlyCost
{
public:
  ExtrinsicMultiStaticCameraWristOnlyCost(const Eigen::Vector2d& obs, const CameraIntrinsics& intr,
                                          const Eigen::Isometry3d& base_to_wrist,
                                          const Eigen::Isometry3d& base_to_camera,
                                          const Eigen::Vector3d& point_in_target)
    : obs_(obs)
    , intr_(intr)
    , wrist_pose_(poseEigenToCal(base_to_wrist))
    , camera_to_base_orig_(poseEigenToCal(base_to_camera.inverse()))
    , target_pt_(point_in_target)
  {
  }

  template <typename T>
  bool operator()(const T* const pose_camera_to_base_correction, const T* pose_wrist_to_target, T* residual) const
  {
    const T* camera_angle_axis = pose_camera_to_base_correction + 0;
    const T* camera_position = pose_camera_to_base_correction + 3;

    const T* target_angle_axis = pose_wrist_to_target + 0;
    const T* target_position = pose_wrist_to_target + 3;

    T link_point[3];         // Point in wrist coordinates
    T world_point[3];        // Point in world coordinates (base of robot)
    T camera_point_orig[3];  // Point in camera coordinates before correction
    T camera_point[3];       // Point in camera coordinates

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(target_pt_(0));
    target_pt[1] = T(target_pt_(1));
    target_pt[2] = T(target_pt_(2));

    transformPoint(target_angle_axis, target_position, target_pt, link_point);
    poseTransformPoint(wrist_pose_, link_point, world_point);
    transformPoint(camera_angle_axis, camera_position, world_point, camera_point_orig);
    poseTransformPoint(camera_to_base_orig_, camera_point_orig, camera_point);

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
  Pose6d wrist_pose_;
  Pose6d camera_to_base_orig_;
  Eigen::Vector3d target_pt_;
};

}  // namespace industrial_calibration
