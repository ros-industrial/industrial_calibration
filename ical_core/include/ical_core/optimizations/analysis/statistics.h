#pragma once

#include <Eigen/Geometry>
#include <vector>
#include <tuple>

namespace industrial_calibration
{
/**
 * @brief Computes the mean and sample standard deviation statistics of a set of values
 * @return a tuple of the mean and sample standard deviation
 */
std::tuple<double, double> computeStats(const std::vector<double>& values);

/**
 * @brief Holds the mean and standard deviation of a position vector
 */
struct PositionStats
{
  Eigen::Vector3d mean;
  Eigen::Vector3d stdev;
};

/**
 * @brief Contains the mean and standard deviation of a quaternion orientation
 */
struct QuaternionStats
{
  Eigen::Quaterniond mean;
  double stdev;
};

/**
 * @brief Noise statistics for a position vector and quaternion orientation
 */
struct PnPNoiseStat
{
  PositionStats p_stat;
  QuaternionStats q_stat;
};

/**
 * @brief Computes the mean and standard deviation of a set of quaternions
 * @param quaternions
 * @return
 */
QuaternionStats computeQuaternionStats(const std::vector<Eigen::Quaterniond>& quaternions);

/**
 * @brief Computes the mean of a set of quaternions
 * @param orientations
 * @return
 */
Eigen::Quaterniond computeQuaternionMean(const std::vector<Eigen::Quaterniond>& orientations);

}  // namespace industrial_calibration
