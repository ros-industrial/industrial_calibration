#pragma once

#include <Eigen/Geometry>
#include <vector>
#include <tuple>

namespace industrial_calibration
{
/**
 * @brief Computes the mean and sample standard deviation statistics of a set of values
 * @return a tuple of the mean and sample standard deviation
 * @ingroup analysis
 */
std::tuple<double, double> computeStats(const std::vector<double>& values);

/**
 * @brief Holds the mean and standard deviation of a position vector
 * @ingroup analysis
 */
struct PositionStats
{
  Eigen::Vector3d mean;
  Eigen::Vector3d stdev;
};

/**
 * @brief Contains the mean and standard deviation of a quaternion orientation
 * @ingroup analysis
 */
struct QuaternionStats
{
  Eigen::Quaterniond mean;
  double stdev;
};

/**
 * @brief Noise statistics for a position vector and quaternion orientation
 * @ingroup analysis
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
 * @ingroup analysis
 */
QuaternionStats computeQuaternionStats(const std::vector<Eigen::Quaterniond>& quaternions);

/**
 * @brief Computes the mean of a set of quaternions
 * @details Mean quaternion is found using method described by Markley et al: Quaternion Averaging
 * https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
 *
 * Eq. 12: M = sum(w_i * q_i * q_i^T)
 *
 * Eq. 13: q_bar = argmax(q^T * M * q)
 *
 * "The solution of this maximization problem is well known. The average quaternion is
 * the eigenvector of M corresponding to the maximum eigenvalue."
 *
 * In the above equations, w_i is the weight of the ith quaternion.
 * In this case, all quaternions are equally weighted (i.e. w_i = 1)
 * @ingroup analysis
 */
Eigen::Quaterniond computeQuaternionMean(const std::vector<Eigen::Quaterniond>& orientations);

}  // namespace industrial_calibration
