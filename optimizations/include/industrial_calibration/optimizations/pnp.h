#pragma once

#include <industrial_calibration/core/types.h>
#include <industrial_calibration/core/camera_intrinsics.h>
#include <industrial_calibration/optimizations/covariance_types.h>

namespace industrial_calibration
{
/**
 * @brief Structure containing relevant data for a PnP optimization using 2D data
 * @ingroup optimizations_pnp
 */
struct PnPProblem
{
  CameraIntrinsics intr;
  Correspondence2D3D::Set correspondences;

  Eigen::Isometry3d camera_to_target_guess;

  std::string label_camera_to_target_guess = "camera_to_target";
  const std::array<std::string, 3> labels_translation = { { "x", "y", "z" } };
  const std::array<std::string, 3> labels_rotation = { { "rx", "ry", "rz" } };
};

/**
 * @brief Structure containing relevant data for a PnP optimization using 3D data
 * @ingroup optimizations_pnp
 */
struct PnPProblem3D
{
  Correspondence3D3D::Set correspondences;

  Eigen::Isometry3d camera_to_target_guess;

  std::string label_camera_to_target_guess = "camera_to_target";
  const std::array<std::string, 3> labels_translation = { { "x", "y", "z" } };
  const std::array<std::string, 3> labels_rotation = { { "rx", "ry", "rz" } };
};

/**
 * @brief PnP optimization results structure
 * @ingroup optimizations_pnp
 */
struct PnPResult
{
  bool converged;
  double initial_cost_per_obs;
  double final_cost_per_obs;

  Eigen::Isometry3d camera_to_target;

  CovarianceResult covariance;
};
std::ostream& operator<<(std::ostream& stream, const PnPResult& result);

/**
 * @brief Performs the PnP optimization
 * @ingroup optimizations_pnp
 */
PnPResult optimize(const PnPProblem& params);

/**
 * @brief Performs the PnP 3D optimization
 * @ingroup optimizations_pnp
 */
PnPResult optimize(const PnPProblem3D& params);

}  // namespace industrial_calibration
