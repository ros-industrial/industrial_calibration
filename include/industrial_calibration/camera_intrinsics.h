#pragma once

#include <industrial_calibration/serialization.h>

#include <array>

namespace industrial_calibration
{
/**
 * @brief Structure representing camera intrinsic parameters for a pin-hole model camera
 */
struct CameraIntrinsics
{
  std::array<double, 4> values{ 0.0, 0.0, 0.0, 0.0 };

  double& fx() { return values[0]; }
  double& fy() { return values[1]; }
  double& cx() { return values[2]; }
  double& cy() { return values[3]; }

  const double& fx() const { return values[0]; }
  const double& fy() const { return values[1]; }
  const double& cx() const { return values[2]; }
  const double& cy() const { return values[3]; }

  inline bool operator==(const CameraIntrinsics& rhs) const { return values == rhs.values; }
};

template <typename T>
struct CalibCameraIntrinsics
{
  const T* data;

  CalibCameraIntrinsics(const T* data) : data(data) {}

  const T& fx() const { return data[0]; }
  const T& fy() const { return data[1]; }
  const T& cx() const { return data[2]; }
  const T& cy() const { return data[3]; }

  const T& k1() const { return data[4]; }
  const T& k2() const { return data[5]; }
  const T& p1() const { return data[6]; }
  const T& p2() const { return data[7]; }
  const T& k3() const { return data[8]; }

  constexpr static std::size_t size() { return 9; }
};

template <typename T>
struct MutableCalibCameraIntrinsics
{
  T* data;

  MutableCalibCameraIntrinsics(T* data) : data(data) {}

  const T& fx() const { return data[0]; }
  const T& fy() const { return data[1]; }
  const T& cx() const { return data[2]; }
  const T& cy() const { return data[3]; }

  const T& k1() const { return data[4]; }
  const T& k2() const { return data[5]; }
  const T& p1() const { return data[6]; }
  const T& p2() const { return data[7]; }
  const T& k3() const { return data[8]; }

  T& fx() { return data[0]; }
  T& fy() { return data[1]; }
  T& cx() { return data[2]; }
  T& cy() { return data[3]; }

  T& k1() { return data[4]; }
  T& k2() { return data[5]; }
  T& p1() { return data[6]; }
  T& p2() { return data[7]; }
  T& k3() { return data[8]; }

  constexpr static std::size_t size() { return 9; }
};

}  // namespace industrial_calibration

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
    rhs.cx() = getMember<double>(node, "cx");
    rhs.cy() = getMember<double>(node, "cy");
    rhs.fx() = getMember<double>(node, "fx");
    rhs.fy() = getMember<double>(node, "fy");

    return true;
  }
};

}  // namespace YAML
