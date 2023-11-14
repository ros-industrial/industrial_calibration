#pragma once

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

namespace YAML
{
template <typename FloatT>
struct convert<Eigen::Matrix<FloatT, 2, 1>>
{
  using T = Eigen::Matrix<FloatT, 2, 1>;

  static Node encode(const T& val)
  {
    YAML::Node node;
    node["x"] = val.x();
    node["y"] = val.y();
    return node;
  }

  static bool decode(const YAML::Node& node, T& val)
  {
    val.x() = node["x"].as<FloatT>();
    val.y() = node["y"].as<FloatT>();

    return true;
  }
};

template <typename FloatT>
struct convert<Eigen::Matrix<FloatT, 3, 1>>
{
  using T = Eigen::Matrix<FloatT, 3, 1>;

  static Node encode(const T& val)
  {
    YAML::Node node;
    node["x"] = val.x();
    node["y"] = val.y();
    node["z"] = val.z();
    return node;
  }

  static bool decode(const YAML::Node& node, T& val)
  {
    val.x() = node["x"].as<FloatT>();
    val.y() = node["y"].as<FloatT>();
    val.z() = node["z"].as<FloatT>();

    return true;
  }
};

template <typename FloatT>
struct convert<Eigen::Transform<FloatT, 3, Eigen::Isometry>>
{
  using T = Eigen::Transform<FloatT, 3, Eigen::Isometry>;

  static Node encode(const T& val)
  {
    YAML::Node node;
    node["x"] = val.translation().x();
    node["y"] = val.translation().y();
    node["z"] = val.translation().z();

    Eigen::Quaternion<FloatT> quat(val.linear());
    node["qw"] = quat.w();
    node["qx"] = quat.x();
    node["qy"] = quat.y();
    node["qz"] = quat.z();

    return node;
  }

  static bool decode(const YAML::Node& node, T& val)
  {
    Eigen::Matrix<FloatT, 3, 1> trans;
    trans.x() = node["x"].as<FloatT>();
    trans.y() = node["y"].as<FloatT>();
    trans.z() = node["z"].as<FloatT>();

    Eigen::Quaternion<FloatT> quat;
    quat.w() = node["qw"].as<FloatT>();
    quat.x() = node["qx"].as<FloatT>();
    quat.y() = node["qy"].as<FloatT>();
    quat.z() = node["qz"].as<FloatT>();

    val = Eigen::Translation<FloatT, 3>(trans) * quat;

    return true;
  }
};

}  // namespace YAML
