#pragma once

#include <ical_core/dh_chain.h>
#include <ical_core/types.h>
#include <ical_core_tests/utilities.h>

namespace industrial_calibration
{
namespace test
{
/**
 * @brief Create a number of 3D-3D observations by randomly actuating kinematic chains to which the camera and target
 * are fixed This method assumes a "magical" camera that can see target features regardless of view point
 * @param to_camera_chain - DH chain to the camera
 * @param to_target_chain - DH chain to the target
 * @param true_mount_to_camera - The true transform from the camera mount to the camera
 * @param true_mount_to_target - The true transform from the target mount to the target
 * @param camera_base_to_target_base - Transformation between the root link of the camera chain to the root link of the
 * target chain
 * @param target - Observation target definition
 * @param n - Number of samples
 * @return A vector of 3D-3D observations
 */
KinObservation3D3D::Set createKinematicObservations(const DHChain& to_camera_chain, const DHChain& to_target_chain,
                                                    const Eigen::Isometry3d& true_mount_to_camera,
                                                    const Eigen::Isometry3d& true_mount_to_target,
                                                    const Eigen::Isometry3d& camera_base_to_target_base,
                                                    const Target& target, const std::size_t n);

/**
 * @brief Creates a number of 3D-3D observations, but does not return the joint states used to create the observations
 * @param to_camera_chain - DH chain to the camera
 * @param to_target_chain - DH chain to the target
 * @param true_mount_to_camera - The true transform from the camera mount to the camera
 * @param true_mount_to_target - The true transform from the target mount to the target
 * @param camera_base_to_target_base - Transformation between the root link of the camera chain to the root link of the
 * target chain
 * @param target - Observation target definition
 * @param n - Number of samples
 * @return A vector of 3D-3D observations
 */
Observation3D3D::Set createObservations(const DHChain& to_camera_chain, const DHChain& to_target_chain,
                                        const Eigen::Isometry3d& true_mount_to_camera,
                                        const Eigen::Isometry3d& true_mount_to_target,
                                        const Eigen::Isometry3d& camera_base_to_target_base, const Target& target,
                                        const std::size_t n);

/**
 * @brief Creates a number of 2D-3D observations by randomly actuating kinematic chains to which the camera and target
 * are fixed This method creates many random poses and creates correspondences for those in which the target is observed
 * @param to_camera_chain - DH chain to the camera
 * @param to_target_chain - DH chain to the target
 * @param true_mount_to_camera - The true transform from the camera mount to the camera
 * @param true_mount_to_target - The true transform from the target mount to the target
 * @param camera_base_to_target_base - Transformation between the root link of the camera chain to the root link of the
 * target chain
 * @param target - Observation target definition
 * @param camera - 2D camera definition
 * @param n - Number of samples
 * @return
 */
KinObservation2D3D::Set createKinematicObservations(const DHChain& to_camera_chain, const DHChain& to_target_chain,
                                                    const Eigen::Isometry3d& true_mount_to_camera,
                                                    const Eigen::Isometry3d& true_mount_to_target,
                                                    const Eigen::Isometry3d& camera_base_to_target_base,
                                                    const Target& target, const Camera& camera, const std::size_t n);

/**
 * @brief Creates a number of 2D-3D observations, but does not return the joint states used to create the observations
 * @param to_camera_chain - DH chain to the camera
 * @param to_target_chain - DH chain to the target
 * @param true_mount_to_camera - The true transform from the camera mount to the camera
 * @param true_mount_to_target - The true transform from the target mount to the target
 * @param camera_base_to_target_base - Transformation between the root link of the camera chain to the root link of the
 * target chain
 * @param target - Observation target definition
 * @param camera - 2D camera definition
 * @param n - Number of samples
 * @return
 */
Observation2D3D::Set createObservations(const DHChain& to_camera_chain, const DHChain& to_target_chain,
                                        const Eigen::Isometry3d& true_mount_to_camera,
                                        const Eigen::Isometry3d& true_mount_to_target,
                                        const Eigen::Isometry3d& camera_base_to_target_base, const Target& target,
                                        const Camera& camera, const std::size_t n);

/**
 * @brief Creates a number of 6-DoF pose "measurements" from the camera to the target
 * These "measurements" are analogous to data generated by camera tracking or laser tracker systems
 * @param to_camera_chain - DH chain to the camera
 * @param to_target_chain - DH chain to the target
 * @param true_mount_to_camera - The true transform from the camera mount to the camera
 * @param true_mount_to_target - The true transform from the target mount to the target
 * @param camera_base_to_target_base - The transformation between the root link of the camera chain to the root link of
 * the target chain
 * @param n - Number of samples
 * @return
 */
KinematicMeasurement::Set createKinematicMeasurements(const DHChain& to_camera_chain, const DHChain& to_target_chain,
                                                      const Eigen::Isometry3d& true_mount_to_camera,
                                                      const Eigen::Isometry3d& true_mount_to_target,
                                                      const Eigen::Isometry3d& camera_base_to_target_base,
                                                      const std::size_t n);

}  // namespace test
}  // namespace industrial_calibration
