#pragma once

#include <industrial_calibration/exceptions.h>

#include <cxxabi.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

template <typename FloatT>
inline Eigen::Transform<FloatT, 3, Eigen::Isometry> toIsometry(const Eigen::Vector3d& trans,
                                                               const Eigen::Vector3d& euler_zyx)
{
  return Eigen::Translation3d(trans) * Eigen::AngleAxisd(euler_zyx(2), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(euler_zyx(1), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(euler_zyx(0), Eigen::Vector3d::UnitX());
}

template <typename T>
inline T getMember(const YAML::Node& n, const std::string& key)
{
  try
  {
    T value = n[key].as<T>();
    return value;
  }
  catch (const YAML::BadConversion&)
  {
    int status;
    std::string name(abi::__cxa_demangle(typeid(T).name(), 0, 0, &status));
    throw industrial_calibration::BadFileException("Failed to convert parameter '" + key + "' to type '" + name + "'");
  }
  catch (const YAML::Exception&)
  {
    int status;
    std::string name(abi::__cxa_demangle(typeid(T).name(), 0, 0, &status));
    throw industrial_calibration::BadFileException("Failed to find '" + key + "' parameter of type '" + name + "'");
  }
}

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
    val.x() = getMember<FloatT>(node, "x");
    val.y() = getMember<FloatT>(node, "y");

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
    val.x() = getMember<FloatT>(node, "x");
    val.y() = getMember<FloatT>(node, "y");
    val.z() = getMember<FloatT>(node, "z");

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
    trans.x() = getMember<FloatT>(node, "x");
    trans.y() = getMember<FloatT>(node, "y");
    trans.z() = getMember<FloatT>(node, "z");

    Eigen::Quaternion<FloatT> quat;
    quat.w() = getMember<FloatT>(node, "qw");
    quat.x() = getMember<FloatT>(node, "qx");
    quat.y() = getMember<FloatT>(node, "qy");
    quat.z() = getMember<FloatT>(node, "qz");

    val = Eigen::Translation<FloatT, 3>(trans) * quat;

    return true;
  }
};

}  // namespace YAML
