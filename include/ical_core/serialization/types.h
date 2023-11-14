#pragma once

#include <ical_core/types.h>
#include <ical_core/serialization/eigen.h>

using namespace industrial_calibration;

namespace YAML
{
template <>
struct convert<CameraIntrinsics>
{
  using T = CameraIntrinsics;

  static Node encode(const T& rhs)
  {
    YAML::Node node;
    node["cx"] = rhs.cx();
    node["cy"] = rhs.cy();
    node["fx"] = rhs.fx();
    node["fy"] = rhs.fy();
    return node;
  }

  static bool decode(const YAML::Node& node, T& rhs)
  {
    if (node.size() != 4) return false;

    rhs.cx() = node["cx"].as<double>();
    rhs.cy() = node["cy"].as<double>();
    rhs.fx() = node["fx"].as<double>();
    rhs.fy() = node["fy"].as<double>();

    return true;
  }
};

template <Eigen::Index SENSOR_DIM, Eigen::Index WORLD_DIM>
struct convert<Correspondence<SENSOR_DIM, WORLD_DIM>>
{
  using T = Correspondence<SENSOR_DIM, WORLD_DIM>;

  static Node encode(const T& corr)
  {
    YAML::Node node;
    node["in_image"] = corr.in_image;
    node["in_target"] = corr.in_target;
    return node;
  }

  static bool decode(const YAML::Node& node, T& rhs)
  {
    if (node.size() != 2) return false;

    rhs.in_image = node["in_image"].as<decltype(rhs.in_image)>();
    rhs.in_target = node["in_target"].as<decltype(rhs.in_target)>();

    return true;
  }
};

template <Eigen::Index SENSOR_DIM, Eigen::Index WORLD_DIM>
struct convert<Observation<SENSOR_DIM, WORLD_DIM>>
{
  using T = Observation<SENSOR_DIM, WORLD_DIM>;

  static Node encode(const T& obs)
  {
    YAML::Node node;
    node["correspondences"] = obs.correspondence_set;
    node["to_target_mount"] = obs.to_target_mount;
    node["to_camera_mount"] = obs.to_camera_mount;
    return node;
  }

  static bool decode(const YAML::Node& node, T& obs)
  {
    if (node.size() != 3) return false;

    obs.correspondence_set = node["correspondences"].as<decltype(obs.correspondence_set)>();
    obs.to_target_mount = node["to_target_mount"].as<decltype(obs.to_target_mount)>();
    obs.to_camera_mount = node["to_camera_mount"].as<decltype(obs.to_camera_mount)>();

    return true;
  }
};

}  // namespace YAML
