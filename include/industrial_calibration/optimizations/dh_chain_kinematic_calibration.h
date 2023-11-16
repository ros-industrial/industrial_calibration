#pragma once

#include <industrial_calibration/types.h>
#include <industrial_calibration/camera_intrinsics.h>
#include <industrial_calibration/dh_chain.h>
#include <industrial_calibration/optimizations/utils/ceres_math_utilities.h>
#include <industrial_calibration/optimizations/analysis/covariance_analysis.h>

namespace industrial_calibration
{
/**
 * @brief Create a mask of parameter indices from a matrix of boolean values
 * The indices are calculated in column-wise order because Eigen stores it's values internally in column-wise order by
 * default
 * @param mask
 * @return
 */
inline std::vector<int> createDHMask(const Eigen::Array<bool, Eigen::Dynamic, 4>& mask)
{
  std::vector<int> out;
  out.reserve(mask.size());

  const Eigen::Index rows = mask.rows();
  for (Eigen::Index row = 0; row < mask.rows(); ++row)
  {
    for (Eigen::Index col = 0; col < mask.cols(); ++col)
    {
      if (mask(row, col))
      {
        out.push_back(rows * col + row);
      }
    }
  }

  return out;
}

struct KinematicCalibrationProblem2D3D
{
  KinematicCalibrationProblem2D3D(DHChain camera_chain_, DHChain target_chain_)
    : camera_chain(std::move(camera_chain_))
    , target_chain(std::move(target_chain_))
    , camera_mount_to_camera_guess(Eigen::Isometry3d::Identity())
    , target_mount_to_target_guess(Eigen::Isometry3d::Identity())
    , camera_base_to_target_base_guess(Eigen::Isometry3d::Identity())
  {
  }

  KinObservation2D3D::Set observations;
  CameraIntrinsics intr;

  // Optimization Variables
  DHChain camera_chain;
  DHChain target_chain;
  Eigen::Isometry3d camera_mount_to_camera_guess;
  Eigen::Isometry3d target_mount_to_target_guess;
  Eigen::Isometry3d camera_base_to_target_base_guess;

  /* Create an array of masks
   * 0. Camera DH parameters (size joints x 4)
   * 1. Target DH parameters (size joints x 4)
   * 2. Camera mount to camera position (size 3)
   * 3. Camera mount to camera angle axis (size 3)
   * 4. Target mount to target position (size 3)
   * 5. Target mount to target angle axis (size 3)
   * 6. Camera base to target base position (size 3)
   * 7. Target mount to target base angle axis (size 3)
   */
  std::array<std::vector<int>, 8> mask;

  /** @brief Expected standard deviation of the DH chain offsets for the camera DH chain */
  double camera_chain_offset_stdev = 1.0e-3;
  /** @brief Expected standard deviation of the DH chain offsets for the target DH chain */
  double target_chain_offset_stdev = 1.0e-3;

  std::string label_camera_mount_to_camera = "camera_mount_to_camera";
  std::string label_target_mount_to_target = "target_mount_to_target";
  std::string label_camera_base_to_target = "camera_base_to_target";
};

struct KinematicCalibrationProblemPose6D
{
  KinematicCalibrationProblemPose6D(DHChain camera_chain_, DHChain target_chain_)
    : camera_chain(std::move(camera_chain_))
    , target_chain(std::move(target_chain_))
    , camera_mount_to_camera_guess(Eigen::Isometry3d::Identity())
    , target_mount_to_target_guess(Eigen::Isometry3d::Identity())
    , camera_base_to_target_base_guess(Eigen::Isometry3d::Identity())
  {
  }

  DHChain camera_chain;
  DHChain target_chain;
  KinematicMeasurement::Set observations;
  Eigen::Isometry3d camera_mount_to_camera_guess;
  Eigen::Isometry3d target_mount_to_target_guess;
  Eigen::Isometry3d camera_base_to_target_base_guess;

  /* Create an array of masks
   * 0. Camera DH parameters (size joints x 4)
   * 1. Target DH parameters (size joints x 4)
   * 2. Camera mount to camera position (size 3)
   * 3. Camera mount to camera angle axis (size 3)
   * 4. Target mount to target position (size 3)
   * 5. Target mount to target angle axis (size 3)
   * 6. Camera base to target base position (size 3)
   * 7. Target mount to target base angle axis (size 3)
   */
  std::array<std::vector<int>, 8> mask;

  /** @brief Expected standard deviation of the DH chain offsets for the camera DH chain */
  double camera_chain_offset_stdev = 1.0e-3;
  /** @brief Expected standard deviation of the DH chain offsets for the target DH chain */
  double target_chain_offset_stdev = 1.0e-3;

  std::string label_camera_mount_to_camera = "camera_mount_to_camera";
  std::string label_target_mount_to_target = "target_mount_to_target";
  std::string label_camera_base_to_target = "camera_base_to_target";
};

struct KinematicCalibrationResult
{
  bool converged;
  double initial_cost_per_obs;
  double final_cost_per_obs;

  Eigen::Isometry3d camera_mount_to_camera;
  Eigen::Isometry3d target_mount_to_target;
  Eigen::Isometry3d camera_base_to_target_base;
  Eigen::MatrixX4d camera_chain_dh_offsets;
  Eigen::MatrixX4d target_chain_dh_offsets;

  CovarianceResult covariance;
};

/**
 * @brief Performs the kinematic calibration optimization with 2D-3D correspondences
 * @param problem
 * @return
 */
KinematicCalibrationResult optimize(const KinematicCalibrationProblem2D3D& problem);

/**
 * @brief Performs the kinematic calibration optimization with 6D pose measurements
 * @param problem
 * @param orientation_weight - The value by which the orientation residual should be scaled relative to the position
 * residual
 * @param options - Ceres solver options
 * @return
 */
KinematicCalibrationResult optimize(const KinematicCalibrationProblemPose6D& problem,
                                    const double orientation_weight = 100.0,
                                    const ceres::Solver::Options& options = ceres::Solver::Options());

}  // namespace industrial_calibration
