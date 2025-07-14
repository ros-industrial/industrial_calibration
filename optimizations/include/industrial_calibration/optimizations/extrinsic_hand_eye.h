/*
 * This file defines a solver for calibrating the EXTRINSIC parameters of a 3D
 * camera attached to the wrist of a moving robot. It works by imaging a static target
 * from many robot wrist positions.
 *
 * The only difference between this the and 2D, pinhole variety is that the correspondences
 * in the observation set are 3D to 3D instead of 2D to 3D. This is meant for 3D sensors where
 * you don't know (or don't want to know) the intrinsics of the sensor or they aren't well
 * described by the pinhole model.
 *
 * For example, this calibration has been used to detect 3D features in a "3D image" produced
 * by the IFM O3D3xx cameras. Sometimes you may want to use this for Openni cameras where
 * because of terrible drivers your calibration does not affect the depth data.
 *
 * See extrinsic_camera_on_wrist.h for a description of the other parameters.
 */
#pragma once

#include <industrial_calibration/core/types.h>
#include <industrial_calibration/core/camera_intrinsics.h>
#include <industrial_calibration/optimizations/covariance_types.h>

#include <Eigen/Dense>
#include <vector>

namespace industrial_calibration
{
/**
 * @ingroup optimizations_extrinsic_hand_eye
 */
struct ExtrinsicHandEyeProblem2D3D
{
  typename Observation2D3D::Set observations;
  CameraIntrinsics intr;
  Eigen::Isometry3d target_mount_to_target_guess;
  Eigen::Isometry3d camera_mount_to_camera_guess;

  std::array<std::string, 6> labels_isometry3d = { { "x", "y", "z", "rx", "ry", "rz" } };
  std::string label_target_mount_to_target = "target_mount_to_target";
  std::string label_camera_mount_to_camera = "camera_mount_to_camera";

  inline bool operator==(const ExtrinsicHandEyeProblem2D3D& rhs) const
  {
    return observations == rhs.observations && intr == rhs.intr &&
           target_mount_to_target_guess.isApprox(rhs.target_mount_to_target_guess) &&
           camera_mount_to_camera_guess.isApprox(rhs.camera_mount_to_camera_guess);
  }
};

/**
 * @ingroup optimizations_extrinsic_hand_eye
 */
struct ExtrinsicHandEyeProblem3D3D
{
  typename Observation3D3D::Set observations;
  Eigen::Isometry3d target_mount_to_target_guess;
  Eigen::Isometry3d camera_mount_to_camera_guess;

  std::array<std::string, 6> labels_isometry3d = { { "x", "y", "z", "rx", "ry", "rz" } };
  std::string label_target_mount_to_target = "target_mount_to_target";
  std::string label_camera_mount_to_camera = "camera_mount_to_camera";

  inline bool operator==(const ExtrinsicHandEyeProblem3D3D& rhs) const
  {
    return observations == rhs.observations &&
           target_mount_to_target_guess.isApprox(rhs.target_mount_to_target_guess) &&
           camera_mount_to_camera_guess.isApprox(rhs.camera_mount_to_camera_guess);
  }
};

/**
 * @ingroup optimizations_extrinsic_hand_eye
 */
struct ExtrinsicHandEyeResult
{
  bool converged;
  double initial_cost_per_obs;
  double final_cost_per_obs;

  Eigen::Isometry3d target_mount_to_target;
  Eigen::Isometry3d camera_mount_to_camera;

  CovarianceResult covariance;
};
std::ostream& operator<<(std::ostream& stream, const ExtrinsicHandEyeResult& result);

/**
 * @ingroup optimizations_extrinsic_hand_eye
 */
ExtrinsicHandEyeResult optimize(const ExtrinsicHandEyeProblem2D3D& params);

/**
 * @ingroup optimizations_extrinsic_hand_eye
 */
ExtrinsicHandEyeResult optimize(const ExtrinsicHandEyeProblem3D3D& params);

}  // namespace industrial_calibration

namespace YAML
{
class Node;

template <typename T>
struct convert;

template <>
struct convert<industrial_calibration::ExtrinsicHandEyeProblem2D3D>
{
  static Node encode(const industrial_calibration::ExtrinsicHandEyeProblem2D3D& rhs);
  static bool decode(const Node& node, industrial_calibration::ExtrinsicHandEyeProblem2D3D& rhs);
};

template <>
struct convert<industrial_calibration::ExtrinsicHandEyeProblem3D3D>
{
  static Node encode(const industrial_calibration::ExtrinsicHandEyeProblem3D3D& rhs);
  static bool decode(const Node& node, industrial_calibration::ExtrinsicHandEyeProblem3D3D& rhs);
};

template <>
struct convert<industrial_calibration::ExtrinsicHandEyeResult>
{
  static Node encode(const industrial_calibration::ExtrinsicHandEyeResult& rhs);
  static bool decode(const YAML::Node& node, industrial_calibration::ExtrinsicHandEyeResult& rhs);
};

}  // namespace YAML
