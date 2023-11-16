#pragma once

#include <industrial_calibration/types.h>
#include <industrial_calibration/camera_intrinsics.h>

namespace industrial_calibration
{
/**
 * @brief Structure containing the error measurements between 2 virtual correspondence sets within one real
 * correspondence set
 */
struct VirtualCorrespondenceResult
{
  /** @brief Transform from the first virtual correspondence set to the second */
  Eigen::Isometry3d t1_to_t2;
  /** @brief The positional error between the transforms from the first and second virtual correspondence sets (m) */
  double positional_error;
  /** @brief The angular error between the transforms from the first and second virtual correspondence sets (rad) */
  double angular_error;
};

/**
 * @brief Divides a set of correspondences into two halves and measures the difference in position of one half to the
 * other Method:
 *     1. Split the correspondence set into two virtual correspondence sets
 *     2. Solve PnP optimization for each virtual correspondence set to get the transform from the camera to the virtual
 * correspondence set
 *     3. Return the error between the two virtual correspondence sets
 *
 *  Note: the two virtual correspondence sets are composed of different points, but those points are relative to the
 * same origin Therefore, the transformation from one virtual correspondence set to the other should be identity, given
 * perfect camera intrinsic parameters
 *
 * @param correspondences - set of corresponding observed features and target features
 * @param intr - camera intrinsic parameters
 * @param camera_to_target_guess - an initial guess about the location of the target relative to the camera (default:
 * identity)
 * @param pnp_sq_error_threshold - Max squared error allowed for a PnP optimization.
 * This value should be driven by the accuracy of the sensor providing the observations (default: 1.0 pixel^2)
 * @return
 */
VirtualCorrespondenceResult
measureVirtualTargetDiff(const Correspondence2D3D::Set& correspondences, const CameraIntrinsics& intr,
                         const Eigen::Isometry3d& camera_to_target_guess = Eigen::Isometry3d::Identity(),
                         const double pnp_sq_error_threshold = 1.0);

/**
 * @brief Structure containing measurements of the intrinsic calibration accuracy
 */
struct IntrinsicCalibrationAccuracyResult
{
  std::pair<double, double> pos_error; /** @brief mean/stdev pair for positional error */
  std::pair<double, double> ang_error; /** @brief mean/stdev pair for angular error */
};

/**
 * @brief Calculates the mean and variance of the transform between two virtual correspondence sets (extracted from a
 * single correspondence set) for a set of calibration observations Method:
 *     1. Check that the correspondences in all observations are the same size
 *       - Assumptions:
 *         - Correspondences are ordered in the same way for each observation
 *         - The correspondences in each observation can be divided in half to get the same two sets of features
 *     2. Calculate error between two virtual correspondence sets in adjacent observations
 *     3. Calculate the mean and standard deviation of the positional difference in this error for all observations
 *       - Theoretically, this difference should be zero for a perfectly intrinsically calibrated camera
 *         that perfectly observed the target features
 *
 * @param observations - a set of calibration observations
 * @param intr - camera intrinsic parameters
 * @param camera_mount_to_camera - The transformation from the camera mount frame to the camera
 * @param target_mount_to_target - The transformation from the target mount frame to the target
 * @param camera_base_to_target_base - the transform from the camera base frame to the target base frame (typically
 * identity)
 * @param pnp_sq_error_threshold - Max squared error allowed for a PnP optimization.
 * This value should be driven by the accuracy of the sensor providing the observations (default: 1.0 pixel^2)
 * @return
 */
IntrinsicCalibrationAccuracyResult measureIntrinsicCalibrationAccuracy(
    const Observation2D3D::Set& observations, const CameraIntrinsics& intr,
    const Eigen::Isometry3d& camera_mount_to_camera, const Eigen::Isometry3d& target_mount_to_target,
    const Eigen::Isometry3d& camera_base_to_target_base, const double pnp_sq_error_threshold = 1.0);

}  // namespace industrial_calibration
