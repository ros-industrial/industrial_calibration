#pragma once

#include <industrial_calibration/dh_chain.h>
#include <industrial_calibration/types.h>
#include <industrial_calibration/serialization/eigen.h>

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
    obs.correspondence_set = node["correspondences"].as<decltype(obs.correspondence_set)>();
    obs.to_target_mount = node["to_target_mount"].as<decltype(obs.to_target_mount)>();
    obs.to_camera_mount = node["to_camera_mount"].as<decltype(obs.to_camera_mount)>();

    return true;
  }
};

template <>
struct convert<DHTransform>
{
  using T = DHTransform;
  static bool decode(const Node& n, T& val)
  {
    // Joint type
    switch (static_cast<DHJointType>(n["type"].as<unsigned>()))
    {
      case DHJointType::LINEAR:
        val.type = DHJointType::LINEAR;
        break;
      case DHJointType::REVOLUTE:
        val.type = DHJointType::REVOLUTE;
        break;
      default:
        throw std::runtime_error("Invalid joint type");
    }

    // DH offsets
    {
      YAML::Node d = n["d"];
      YAML::Node theta = n["theta"];
      YAML::Node r = n["r"];
      YAML::Node alpha = n["alpha"];

      Eigen::Vector4d params;
      params << d.as<double>(), theta.as<double>(), r.as<double>(), alpha.as<double>();

      val.params = params;
    }

    val.name = n["joint_name"].as<std::string>();

    return true;
  }
};

template <>
struct convert<DHChain>
{
  using T = DHChain;
  inline static bool decode(const Node& n, T& val)
  {
    val.base_offset_ = n["base_transform"].as<Eigen::Isometry3d>();

    const YAML::Node& transforms = n["dh_transforms"];
    val.transforms_.clear();
    val.transforms_.reserve(transforms.size());
    for (std::size_t i = 0; i < transforms.size(); i++)
    {
      val.transforms_.push_back(transforms[i].as<DHTransform>());
    }

    return true;
  }
};

}  // namespace YAML
