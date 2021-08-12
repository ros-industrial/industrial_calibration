#pragma once

#include <ical_core/optimizations/extrinsic_hand_eye.h>
#include <ical_core/serialization/types.h>

namespace
{
template <typename FloatT>
Eigen::Transform<FloatT, 3, Eigen::Isometry> toIsometry(const Eigen::Vector3d& trans, const Eigen::Vector3d& euler_zyx)
{
  return Eigen::Translation3d(trans) * Eigen::AngleAxisd(euler_zyx(2), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(euler_zyx(1), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(euler_zyx(0), Eigen::Vector3d::UnitX());
}
}  // namespace

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
    if (node.size() != 4) return false;

    rhs.intr = node["intr"].as<decltype(rhs.intr)>();
    rhs.camera_mount_to_camera_guess =
        node["camera_mount_to_camera_guess"].as<decltype(rhs.camera_mount_to_camera_guess)>();
    rhs.target_mount_to_target_guess =
        node["target_mount_to_target_guess"].as<decltype(rhs.target_mount_to_target_guess)>();
    rhs.observations = node["observations"].as<decltype(rhs.observations)>();

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
    if (node.size() != 3) return false;

    rhs.camera_mount_to_camera_guess =
        node["camera_mount_to_camera_guess"].as<decltype(rhs.camera_mount_to_camera_guess)>();
    rhs.target_mount_to_target_guess =
        node["target_mount_to_target_guess"].as<decltype(rhs.target_mount_to_target_guess)>();
    rhs.observations = node["observations"].as<decltype(rhs.observations)>();

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
    if (node.size() != 7) return false;

    rhs.converged = node["converged"].as<decltype(rhs.converged)>();
    rhs.initial_cost_per_obs = node["initial_cost_per_obs"].as<decltype(rhs.initial_cost_per_obs)>();
    rhs.final_cost_per_obs = node["final_cost_per_obs"].as<decltype(rhs.final_cost_per_obs)>();

    Eigen::Vector3d target_mount_to_target_pos = node["target_mount_to_target_pos"].as<Eigen::Vector3d>();
    Eigen::Vector3d target_mount_to_target_rpy = node["target_mount_to_target_rpy"].as<Eigen::Vector3d>().reverse();
    rhs.target_mount_to_target = toIsometry<double>(target_mount_to_target_pos, target_mount_to_target_rpy);

    Eigen::Vector3d camera_mount_to_camera_pos = node["camera_mount_to_camera_pos"].as<Eigen::Vector3d>();
    Eigen::Vector3d camera_mount_to_camera_rpy = node["camera_mount_to_camera_rpy"].as<Eigen::Vector3d>().reverse();
    rhs.camera_mount_to_camera = toIsometry<double>(camera_mount_to_camera_pos, camera_mount_to_camera_rpy);

    return true;
  }
};

}  // namespace YAML
