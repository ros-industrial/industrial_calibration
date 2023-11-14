#pragma once

#include <industrial_calibration_tests/utilities.h>
#include <industrial_calibration_tests/pose_generator.h>

namespace industrial_calibration
{
namespace test
{
/**
 * @brief Observes a set of simulated 2D-3D correspondences given a 2D camera and target definition
 * This method performs validity checks to ensure target observations appear in the camera field of view
 * @param camera_pose - the transform from a common frame to the camera
 * @param target_pose - the transform from a common frame to the target
 * @param camera - the camera intrinsic parameters
 * @param target - the observation target definition
 * @param require_all - flag to require all target observations be seen in the camera
 * @return A set of all "seen" correspondences
 * @throws Exception if @ref require_all is true and not all observations are seen
 */
Correspondence2D3D::Set getCorrespondences(const Eigen::Isometry3d& camera_pose, const Eigen::Isometry3d& target_pose,
                                           const Camera& camera, const Target& target, const bool require_all = false);

/**
 * @brief Observes a set of simulated 3D-3D correspondences given a target definition.
 * Note this method does not consider whether or not target observations can be seen by the camera
 * @param camera_pose - the transform from a common frame to the camera
 * @param target_pose - the transform from a common frame to the target
 * @param target - the observation target definition
 * @return
 */
Correspondence3D3D::Set getCorrespondences(const Eigen::Isometry3d& camera_pose, const Eigen::Isometry3d& target_pose,
                                           const Target& target) noexcept;

/**
 * @brief Creates a 2D-3D observation set
 * @param camera - camera intrinsics
 * @param target - target definition
 * @param pose_generators - vector of shared pointers to variants of base class for camera pose generation
 * @param true_target_mount_to_target - the true transform from the target mount to the target
 * @param true_camera_mount_to_camera - the true transform from the camera mount to the camera
 * @param camera_base_to_target_base - the transform from the camera base frame to the target base frame (typically
 * identity)
 * @return
 */
Observation2D3D::Set createObservations(
    const Camera& camera, const Target& target, const std::vector<std::shared_ptr<PoseGenerator>>& pose_generators,
    const Eigen::Isometry3d& true_target_mount_to_target, const Eigen::Isometry3d& true_camera_mount_to_camera,
    const Eigen::Isometry3d& camera_base_to_target_base = Eigen::Isometry3d::Identity());

/**
 * @brief Creates a 3D-3D observation set
 * @param target - target definition
 * @param pose_generators - vector of shared pointers to variants of base class for camera pose generation
 * @param true_target_mount_to_target - the true transform from the target mount to the target
 * @param true_camera_mount_to_camera - the true transform from the camera mount to the camera
 * @param camera_base_to_target_base - the transform from the camera base frame to the target base frame (typically
 * identity)
 * @return
 */
Observation3D3D::Set createObservations(
    const Target& target, const std::vector<std::shared_ptr<PoseGenerator>>& pose_generators,
    const Eigen::Isometry3d& true_target_mount_to_target, const Eigen::Isometry3d& true_camera_mount_to_camera,
    const Eigen::Isometry3d& camera_base_to_target_base = Eigen::Isometry3d::Identity());

}  // namespace test
}  // namespace industrial_calibration
