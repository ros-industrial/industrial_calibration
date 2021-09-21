#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace industrial_calibration
{
/** @brief A double value identified by one or two name strings. */
struct NamedParam
{
  /** @brief Pair of names identifying this parameter. For types with just one name (e.g. standard deviation), only
   * names.first is used. */
  std::pair<std::string, std::string> names;
  /** @brief Value of this parameter. */
  std::double_t value;

  /**
   * @brief Format the NamedParam as a string.
   * @return
   */
  std::string toString() const;
};

/**
 * @brief Covariance results for optimization parameters.
 * Contains standard deviations, covariances, and correlation coefficients, as well as the original covariance and
 * correlation matrices.
 */
struct CovarianceResult
{
  /** @brief standard deviations */
  std::vector<NamedParam> standard_deviations;
  /** @brief correlation_coeffs */
  std::vector<NamedParam> correlation_coeffs;
  /** @brief covariances */
  std::vector<NamedParam> covariances;
  /** @brief Covariance matrix output from Ceres */
  Eigen::MatrixXd covariance_matrix;
  /** @brief Correlation matrix */
  Eigen::MatrixXd correlation_matrix;

  /**
   * @brief Returns named correlation coefficients that exceed @ref threshold.
   * @param Magnitude of a correlation coefficient that will result in it being returned.
   * @return Vector of NamedParams for correlation coefficients above @ref threshold.
   */
  std::vector<NamedParam> getCorrelationCoeffOutsideThreshold(const std::double_t& threshold) const;

  /**
   * @brief Format NamedParam contents as a string.
   * @return
   */
  std::string toString() const;

  /**
   * @brief Compose a string with a list of NamedParams for correlation coefficients above @ref threshold.
   * @param threshold
   * @return
   */
  std::string printCorrelationCoeffAboveThreshold(const std::double_t& threshold) const;
};

}  // namespace industrial_calibration
