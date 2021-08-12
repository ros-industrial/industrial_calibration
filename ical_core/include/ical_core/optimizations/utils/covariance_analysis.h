#pragma once

#include <ical_core/optimizations/utils/covariance_types.h>
#include <ical_core/types.h>

#include <ceres/ceres.h>
#include <Eigen/Dense>

namespace industrial_calibration
{
/**
 * @brief DefaultCovarianceOptions An instance of ceres::Covariance::Options with values better suited to the
 * optimizations in this library.
 */
struct DefaultCovarianceOptions : ceres::Covariance::Options
{
  DefaultCovarianceOptions()
  {
    this->algorithm_type = ceres::DENSE_SVD;  // DENSE_SVD can handle both full-rank and rank-deficient Jacobians
  }
};

/**
 * @brief Given a covariance matrix, calculate the matrix of standard deviations and correlation coefficients.
 * @param Covariance matrix
 * @return Matrix of standard deviations (diagonal elements) and correlation coefficents (off-diagonal elements).
 * @throw CovarianceException if @ref covariance_matrix is not square.
 */
Eigen::MatrixXd computeCorrelationsFromCovariance(const Eigen::MatrixXd& covariance_matrix);

/**
 * @brief Compute all covariance results for a Ceres optimization problem. Labels results with generic names.
 * @param problem The Ceres problem (after optimization).
 * @param options ceres::Covariance::Options to use when calculating covariance.
 * @return CovarianceResult for the problem.
 */
CovarianceResult computeCovariance(ceres::Problem& problem,
                                   const ceres::Covariance::Options& options = DefaultCovarianceOptions());

/**
 * @brief Compute covariance results for the specified parameter blocks in a Ceres optimization problem. Labels results
 * with generic names.
 * @param problem The Ceres problem (after optimization).
 * @param parameter_blocks Specific parameter blocks to compute covariance between.
 * @param options ceres::Covariance::Options to use when calculating covariance.
 * @return CovarianceResult for the problem.
 */
CovarianceResult computeCovariance(ceres::Problem& problem, const std::vector<const double*>& parameter_blocks,
                                   const ceres::Covariance::Options& options = DefaultCovarianceOptions());

/**
 * @brief Compute all covariance results for a Ceres optimization problem and label them with the provided names.
 * @param problem The Ceres problem (after optimization).
 * @param parameter_names Labels for all optimization parameters in the problem.
 * @param options ceres::Covariance::Options to use when calculating covariance.
 * @return CovarianceResult for the problem.
 */
CovarianceResult computeCovariance(ceres::Problem& problem,
                                   const std::vector<std::vector<std::string>>& parameter_names,
                                   const ceres::Covariance::Options& options = DefaultCovarianceOptions());

/**
 * @brief Compute covariance results for specified parameter blocks in a Ceres optimization problem and label them with
 * the provided names.
 * @param problem The Ceres problem (after optimization).
 * @param parameter_blocks Specific parameter blocks for which covariance will be calculated.
 * @param parameter_names Labels for optimization parameters in the specified blocks.
 * @param options ceres::Covariance::Options to use when calculating covariance.
 * @return CovarianceResult for the problem.
 * @throw CovarianceException if covariance.Compute fails.
 * @throw CovarianceException if covariance.GetCovarianceMatrix fails.
 * @throw CovarianceException if parameter_names.size() != parameter_blocks.size().
 * @throw CovarianceException if the number of parameter label strings provided for a block is different than the number
 * of parameters in that block.
 */
CovarianceResult computeCovariance(ceres::Problem& problem, const std::vector<const double*>& parameter_blocks,
                                   const std::vector<std::vector<std::string>>& parameter_names,
                                   const ceres::Covariance::Options& options = DefaultCovarianceOptions());
}  // namespace industrial_calibration
