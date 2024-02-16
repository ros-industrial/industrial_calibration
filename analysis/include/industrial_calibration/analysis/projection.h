#pragma once

#include <industrial_calibration/core/camera_intrinsics.h>
#include <industrial_calibration/core/types.h>

#include <Eigen/Dense>

namespace industrial_calibration
{
/**
 * @brief Computes the vector from an input point on a line to the point where the line intersects with the input plane
 * See <a href=https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection>this link</a> for more information
 * @param plane_normal Unit normal vector defining the plane
 * @param plane_pt Point on the plane
 * @param line Unit vector defining the line
 * @param line_pt Point on the line
 * @param epsilon
 */
Eigen::Vector3d computeLinePlaneIntersection(const Eigen::Vector3d& plane_normal, const Eigen::Vector3d& plane_pt,
                                             const Eigen::Vector3d& line, const Eigen::Vector3d& line_pt,
                                             const double epsilon = 1.0e-6);

/**
 * @brief Projects a set of 2D source points (e.g., from an image observation) onto a 3D plane (e.g., a flat calibration
 * target)
 * @param source Set of 2D points
 * @param source_to_plane Matrix that transforms the source points into the frame of the plane (e.g., camera to target
 * transform)
 * @param k 3x3 camera projection matrix
 * @return
 */
Eigen::MatrixX3d project3D(const Eigen::MatrixX2d& source, const Eigen::Isometry3d& source_to_plane,
                           const Eigen::Matrix3d& k);

/**
 * @brief Projects the 2D target features from an observation onto the 3D plane of the calibrated target and computes
 * the difference between the projections and the known target features
 */
Eigen::ArrayXd compute3DProjectionError(const Observation2D3D& obs, const CameraIntrinsics& intr,
                                        const Eigen::Isometry3d& camera_mount_to_camera,
                                        const Eigen::Isometry3d& target_mount_to_target);

/**
 * @brief Projects the 2D target features from a set of observations onto the 3D plane of the calibrated target and
 * computes the difference between the projections and the known target features
 */
Eigen::ArrayXd compute3DProjectionError(const Observation2D3D::Set& obs, const CameraIntrinsics& intr,
                                        const Eigen::Isometry3d& camera_mount_to_camera,
                                        const Eigen::Isometry3d& target_mount_to_target);

}  // namespace industrial_calibration
