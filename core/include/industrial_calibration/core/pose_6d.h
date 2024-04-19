#pragma once

#include <array>

namespace industrial_calibration
{
/**
 * @brief Representation of an isometry homogeneous transform for better integration with Ceres
 */
struct Pose6d
{
  Pose6d() = default;
  Pose6d(std::array<double, 6> l) : values(l) {}

  std::array<double, 6> values{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  double& rx() { return values[0]; }
  double& ry() { return values[1]; }
  double& rz() { return values[2]; }
  double& x() { return values[3]; }
  double& y() { return values[4]; }
  double& z() { return values[5]; }
  const double& rx() const { return values[0]; }
  const double& ry() const { return values[1]; }
  const double& rz() const { return values[2]; }
  const double& x() const { return values[3]; }
  const double& y() const { return values[4]; }
  const double& z() const { return values[5]; }
};

}  // namespace industrial_calibration
