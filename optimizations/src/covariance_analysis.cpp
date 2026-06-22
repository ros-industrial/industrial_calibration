#include <industrial_calibration/optimizations/covariance_analysis.h>
#include <industrial_calibration/core/exceptions.h>

#include <cmath>
#include <limits>
#include <sstream>

namespace industrial_calibration
{
Eigen::MatrixXd computeCorrelationsFromCovariance(const Eigen::MatrixXd& covariance_matrix)
{
  if (covariance_matrix.rows() != covariance_matrix.cols())
    throw CovarianceException("Cannot compute correlations from a non-square matrix");

  Eigen::Index num_vars = covariance_matrix.rows();

  Eigen::MatrixXd out(num_vars, num_vars);
  const double nan = std::numeric_limits<double>::quiet_NaN();

  for (Eigen::Index i = 0; i < num_vars; i++)
  {
    const double variance_i = covariance_matrix(i, i);
    const bool variance_i_is_finite = std::isfinite(variance_i);
    const bool variance_i_is_nonnegative = variance_i >= 0.0;
    const double sigma_i = (variance_i_is_finite && variance_i_is_nonnegative) ? std::sqrt(variance_i) : nan;

    for (Eigen::Index j = 0; j < num_vars; j++)
    {
      const double variance_j = covariance_matrix(j, j);
      const bool variance_j_is_finite = std::isfinite(variance_j);
      const bool variance_j_is_nonnegative = variance_j >= 0.0;
      const double sigma_j = (variance_j_is_finite && variance_j_is_nonnegative) ? std::sqrt(variance_j) : nan;

      if (i == j)  // if out(i,j) is a diagonal element
        out(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = sigma_i;
      else
      {
        const bool sigma_i_is_usable = std::isfinite(sigma_i) && sigma_i > std::numeric_limits<double>::epsilon();
        const bool sigma_j_is_usable = std::isfinite(sigma_j) && sigma_j > std::numeric_limits<double>::epsilon();

        out(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) =
            (sigma_i_is_usable && sigma_j_is_usable) ? (covariance_matrix(i, j) / (sigma_i * sigma_j)) : nan;
      }
    }
  }

  return out;
}

CovarianceResult computeCovarianceResults(const Eigen::MatrixXd& cov_matrix,
                                          const std::vector<std::string>& parameter_names)
{
  // 0. Save original covariance matrix output
  // For parameter blocks [p1, p2], the structure of this matrix will be:
  /*    |     p1    |     p2    |
   * ---|-----------|-----------|
   * p1 | C(p1, p1) | C(p1, p2) |
   * p2 | C(p2, p1) | C(p2, p2) |
   */
  CovarianceResult res;
  res.covariance_matrix = cov_matrix;

  // 1. Compute matrix of standard deviations and correlation coefficients.
  // The arrangement of elements in the correlation matrix matches the order in the covariance matrix.
  Eigen::MatrixXd correlation_matrix = computeCorrelationsFromCovariance(cov_matrix);
  res.correlation_matrix = correlation_matrix;

  // 2. Create NamedParams for covariance and correlation results, which include labels and values for the parameters.
  // Uses top-right triangular part of matrix.
  Eigen::Index col_start = 0;
  for (Eigen::Index row = 0; row < correlation_matrix.rows(); row++)
  {
    for (Eigen::Index col = col_start; col < correlation_matrix.rows(); col++)
    {
      NamedParam p_corr;
      p_corr.value = correlation_matrix(row, col);

      if (row == col)  // diagonal element, standard deviation
      {
        p_corr.names = std::make_pair(parameter_names[static_cast<std::size_t>(row)], "");
        res.standard_deviations.push_back(p_corr);
        continue;
      }

      // otherwise off-diagonal element, correlation coefficient
      p_corr.names = std::make_pair(parameter_names[static_cast<std::size_t>(row)],
                                    parameter_names[static_cast<std::size_t>(col)]);
      res.correlation_coeffs.push_back(p_corr);

      // for off-diagonal elements also get covariance
      NamedParam p_cov;
      p_cov.value = cov_matrix(row, col);
      p_cov.names = std::make_pair(parameter_names[static_cast<std::size_t>(row)],
                                   parameter_names[static_cast<std::size_t>(col)]);
      res.covariances.push_back(p_cov);
    }
    col_start++;
  }

  return res;
}

CovarianceResult computeCovariance(ceres::Problem& problem,
                                   const std::map<const double*, std::vector<int>>& param_masks,
                                   const ceres::Covariance::Options& options)
{
  std::vector<double*> blocks;
  problem.GetParameterBlocks(&blocks);

  const std::vector<const double*> blocks_const(blocks.begin(), blocks.end());

  return computeCovariance(problem, blocks_const, param_masks, options);
}

CovarianceResult computeCovariance(ceres::Problem& problem, const std::vector<const double*>& parameter_blocks,
                                   const std::map<const double*, std::vector<int>>& param_masks,
                                   const ceres::Covariance::Options& options)
{
  std::map<const double*, std::vector<std::string>> dummy_param_names;
  for (std::size_t block_index = 0; block_index < parameter_blocks.size(); block_index++)
  {
    int size = problem.ParameterBlockSize(parameter_blocks[block_index]);
    std::vector<std::string> block_names;
    for (int i = 0; i < size; i++)
    {
      // names follow format "blockN_elementM"
      block_names.emplace_back("block" + std::to_string(block_index) + "_element" + std::to_string(i));
    }
    dummy_param_names[parameter_blocks[block_index]] = block_names;
  }

  return computeCovariance(problem, parameter_blocks, dummy_param_names, param_masks, options);
}

CovarianceResult computeCovariance(ceres::Problem& problem,
                                   const std::map<const double*, std::vector<std::string>>& param_names,
                                   const std::map<const double*, std::vector<int>>& param_masks,
                                   const ceres::Covariance::Options& options)
{
  // Get all parameter blocks for the problem
  std::vector<double*> parameter_blocks(static_cast<std::size_t>(problem.NumParameterBlocks()));
  problem.GetParameterBlocks(&parameter_blocks);

  const std::vector<const double*> param_blocks_const(parameter_blocks.begin(), parameter_blocks.end());

  return computeCovariance(problem, param_blocks_const, param_names, param_masks, options);
}

CovarianceResult computeCovariance(ceres::Problem& problem, const std::vector<const double*>& parameter_blocks,
                                   const std::map<const double*, std::vector<std::string>>& param_names,
                                   const std::map<const double*, std::vector<int>>& param_masks,
                                   const ceres::Covariance::Options& options)
{
  // 0. Check user-specified arguments
  if (parameter_blocks.size() != param_names.size())
    throw CovarianceException("Provided vector parameter_names is not same length as provided number of parameter "
                              "blocks");

  Eigen::Index n_params_in_selected = 0;
  std::vector<Eigen::Index> tangent_space_indices;
  std::vector<std::string> tangent_space_labels;
  for (const double* b : parameter_blocks)
  {
    int block_size = problem.ParameterBlockSize(b);
    if (static_cast<std::size_t>(block_size) != param_names.at(b).size())
    {
      std::stringstream ss;
      ss << "Number of parameter labels provided for block does not match actual number of parameters in that block: "
         << "have " << param_names.at(b).size() << " labels and " << block_size << " parameters";
      throw CovarianceException(ss.str());
    }

    // Extract tangent space
    std::vector<int> masks;
    auto it = param_masks.find(b);
    if (it != param_masks.end())
      masks = it->second;

    const std::vector<std::string>& label = param_names.at(b);
    for (std::size_t i = 0; i < static_cast<std::size_t>(block_size); ++i)
    {
      if (std::find(masks.begin(), masks.end(), i) == masks.end())
      {
        tangent_space_indices.push_back(n_params_in_selected + static_cast<Eigen::Index>(i));
        tangent_space_labels.push_back(label.at(i));
      }
    }

    n_params_in_selected += block_size;
  }

  // 1. Compute covariance matrix
  ceres::Covariance covariance(options);

  if (!covariance.Compute(parameter_blocks, &problem))
    throw CovarianceException("Could not compute covariance in computeCovariance()");

  Eigen::MatrixXd cov_matrix(n_params_in_selected, n_params_in_selected);
  if (!covariance.GetCovarianceMatrix(parameter_blocks, cov_matrix.data()))
  {
    throw CovarianceException("GetCovarianceMatrix failed in computeCovariance()");
  }

  // 2. Extract the tangent space cov matrix
  Eigen::MatrixXd tangent_space_cov_matrix(tangent_space_labels.size(), tangent_space_labels.size());
  for (std::size_t r = 0; r < tangent_space_indices.size(); ++r)
  {
    for (std::size_t c = 0; c < tangent_space_indices.size(); ++c)
    {
      tangent_space_cov_matrix(r, c) = cov_matrix(tangent_space_indices[r], tangent_space_indices[c]);
    }
  }

  // 3. Get the covariance results
  return computeCovarianceResults(tangent_space_cov_matrix, tangent_space_labels);
}

}  // namespace industrial_calibration
