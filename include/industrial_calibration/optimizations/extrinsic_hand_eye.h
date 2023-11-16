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

#include <industrial_calibration/types.h>
#include <industrial_calibration/camera_intrinsics.h>
#include <industrial_calibration/optimizations/analysis/covariance_types.h>

#include <Eigen/Dense>
#include <vector>

namespace industrial_calibration
{
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

struct ExtrinsicHandEyeResult
{
  bool converged;
  double initial_cost_per_obs;
  double final_cost_per_obs;

  Eigen::Isometry3d target_mount_to_target;
  Eigen::Isometry3d camera_mount_to_camera;

  CovarianceResult covariance;
};

ExtrinsicHandEyeResult optimize(const ExtrinsicHandEyeProblem2D3D& params);
ExtrinsicHandEyeResult optimize(const ExtrinsicHandEyeProblem3D3D& params);

}  // namespace industrial_calibration

using namespace industrial_calibration;

namespace YAML
{
template <>
struct convert<ExtrinsicHandEyeProblem2D3D>
{
  using T = ExtrinsicHandEyeProblem2D3D;

  static Node encode(const T& rhs)
  {
    Node node;

    node["intr"] = rhs.intr;
    node["camera_mount_to_camera_guess"] = rhs.camera_mount_to_camera_guess;
    node["target_mount_to_target_guess"] = rhs.target_mount_to_target_guess;
    node["observations"] = rhs.observations;

    return node;
  }

  static bool decode(const Node& node, T& rhs)
  {
    rhs.intr = getMember<decltype(rhs.intr)>(node, "intr");
    rhs.camera_mount_to_camera_guess = getMember<decltype(rhs.camera_mount_to_camera_guess)>(node, "camera_mount_to_"
                                                                                                   "camera_guess");
    rhs.target_mount_to_target_guess = getMember<decltype(rhs.target_mount_to_target_guess)>(node, "target_mount_to_"
                                                                                                   "target_guess");
    rhs.observations = getMember<decltype(rhs.observations)>(node, "observations");

    return true;
  }
};

template <>
struct convert<ExtrinsicHandEyeProblem3D3D>
{
  using T = ExtrinsicHandEyeProblem3D3D;

  static Node encode(const T& rhs)
  {
    Node node;

    node["camera_mount_to_camera_guess"] = rhs.camera_mount_to_camera_guess;
    node["target_mount_to_target_guess"] = rhs.target_mount_to_target_guess;
    node["observations"] = rhs.observations;

    return node;
  }

  static bool decode(const Node& node, T& rhs)
  {
    rhs.camera_mount_to_camera_guess = getMember<decltype(rhs.camera_mount_to_camera_guess)>(node, "camera_mount_to_"
                                                                                                   "camera_guess");
    rhs.target_mount_to_target_guess = getMember<decltype(rhs.target_mount_to_target_guess)>(node, "target_mount_to_"
                                                                                                   "target_guess");
    rhs.observations = getMember<decltype(rhs.observations)>(node, "observations");

    return true;
  }
};

template <>
struct convert<ExtrinsicHandEyeResult>
{
  using T = ExtrinsicHandEyeResult;
  static Node encode(const T& rhs)
  {
    Node node;
    node["converged"] = rhs.converged;
    node["initial_cost_per_obs"] = rhs.initial_cost_per_obs;
    node["final_cost_per_obs"] = rhs.final_cost_per_obs;
    node["target_mount_to_target_pos"] = Eigen::Vector3d(rhs.target_mount_to_target.translation());
    node["camera_mount_to_camera_pos"] = Eigen::Vector3d(rhs.camera_mount_to_camera.translation());

    // Serialize transform orientation in RPY for convenience
    node["target_mount_to_target_rpy"] =
        Eigen::Vector3d(rhs.target_mount_to_target.rotation().eulerAngles(2, 1, 0).reverse());
    node["camera_mount_to_camera_rpy"] =
        Eigen::Vector3d(rhs.camera_mount_to_camera.rotation().eulerAngles(2, 1, 0).reverse());
    return node;
  }

  static bool decode(const YAML::Node& node, T& rhs)
  {
    rhs.converged = getMember<decltype(rhs.converged)>(node, "converged");
    rhs.initial_cost_per_obs = getMember<decltype(rhs.initial_cost_per_obs)>(node, "initial_cost_per_obs");
    rhs.final_cost_per_obs = getMember<decltype(rhs.final_cost_per_obs)>(node, "final_cost_per_obs");

    Eigen::Vector3d target_mount_to_target_pos = getMember<Eigen::Vector3d>(node, "target_mount_to_target_pos");
    Eigen::Vector3d target_mount_to_target_rpy = getMember<Eigen::Vector3d>(node, "target_mount_to_target_rpy");
    rhs.target_mount_to_target = toIsometry<double>(target_mount_to_target_pos, target_mount_to_target_rpy.reverse());

    Eigen::Vector3d camera_mount_to_camera_pos = getMember<Eigen::Vector3d>(node, "camera_mount_to_camera_pos");
    Eigen::Vector3d camera_mount_to_camera_rpy = getMember<Eigen::Vector3d>(node, "camera_mount_to_camera_rpy");
    rhs.camera_mount_to_camera = toIsometry<double>(camera_mount_to_camera_pos, camera_mount_to_camera_rpy.reverse());

    return true;
  }
};

}  // namespace YAML
