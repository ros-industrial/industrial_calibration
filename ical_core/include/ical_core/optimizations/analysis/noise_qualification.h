#pragma once

#include <Eigen/Dense>
#include <ical_core/types.h>
#include <ical_core/optimizations/pnp.h>

namespace industrial_calibration
{
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

/**
 * @brief This function qualifies 2D sensor noise by
 * comparing PnP results from images taken at same pose.
 * Sensor noise can be understood by inspecting the returned standard
 * deviations.
 * @param Sets of PnP 2D problem parameters
 * @return Noise Statistics: a vector of means & std devs
 */
PnPNoiseStat qualifyNoise2D(const std::vector<PnPProblem>& params);

/**
 * @brief This function qualifies 3D sensor noise by
 * comparing PnP results from scans taken at the same pose.
 * Sensor noise can be understood by inspecting the returned standard
 * deviations.
 * @param params 3D image parameters
 * @return Noise Statiscics: a vector of standard deviations and the mean pos
 */
PnPNoiseStat qualifyNoise3D(const std::vector<PnPProblem3D>& params);

}  // namespace industrial_calibration
