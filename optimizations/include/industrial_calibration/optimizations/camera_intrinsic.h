#pragma once

#include <industrial_calibration/optimizations/covariance_types.h>
#include <industrial_calibration/core/camera_intrinsics.h>
#include <industrial_calibration/core/types.h>

namespace industrial_calibration
{
/**
 * @brief Structure containing the relevant data for a camera intrinsic calibration
 */
struct CameraIntrinsicProblem
{
  std::vector<Correspondence2D3D::Set> image_observations;
  CameraIntrinsics intrinsics_guess;
  bool use_extrinsic_guesses;
  std::vector<Eigen::Isometry3d> extrinsic_guesses;

  std::string label_extr = "pose";
  const std::array<std::string, 9> labels_intrinsic_params = { { "fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2",
                                                                 "k3" } };
  const std::array<std::string, 6> labels_isometry3d = { { "x", "y", "z", "rx", "ry", "rz" } };
};

/**
 * @brief Results of the camera intrinsic calibration
 */
struct CameraIntrinsicResult
{
  bool converged;
  double initial_cost_per_obs;
  double final_cost_per_obs;

  CameraIntrinsics intrinsics;
  std::array<double, 5> distortions;

  std::vector<Eigen::Isometry3d> target_transforms;

  CovarianceResult covariance;
};
std::ostream& operator<<(std::ostream& stream, const CameraIntrinsicResult& result);

/** @brief Performs the camera intrinsic calibration */
CameraIntrinsicResult optimize(const CameraIntrinsicProblem& params);

}  // namespace industrial_calibration

namespace YAML
{
class Node;

template <typename T>
struct convert;

template <>
struct convert<industrial_calibration::CameraIntrinsicProblem>
{
  static Node encode(const industrial_calibration::CameraIntrinsicProblem& rhs);
  static bool decode(const Node& node, industrial_calibration::CameraIntrinsicProblem& rhs);
};

template <>
struct convert<industrial_calibration::CameraIntrinsicResult>
{
  static Node encode(const industrial_calibration::CameraIntrinsicResult& rhs);
  static bool decode(const YAML::Node& node, industrial_calibration::CameraIntrinsicResult& rhs);
};

}  // namespace YAML
