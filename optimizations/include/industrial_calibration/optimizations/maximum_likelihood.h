#pragma once

#include <industrial_calibration/core/types.h>
#include <industrial_calibration/core/exceptions.h>

#include <Eigen/Core>

namespace industrial_calibration
{
/**
 * @brief Cost function for a parameter block based on the expected means and standard deviations for the values in the
 * block. The inputs are an array of mean values and an array of standard deviations corresponding to the optimization
 * variables in the parameter block
 *
 * Ceres provides an example of a maximum likelihood constraint here:
 * https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/robot_pose_mle.cc#165
 * blob: 83c0903d11f2558a9eb48884addc44853ffb903b
 */
class MaximumLikelihood
{
public:
  MaximumLikelihood(const Eigen::ArrayXXd& mean_, const Eigen::ArrayXXd& stdev_) : mean(mean_), stdev(stdev_)
  {
    if (mean.size() != stdev.size())
      throw OptimizationException("Mean and standard deviation matrix are not the same size");
  }

  template <typename T>
  bool operator()(T const* const* parameter, T* residual) const
  {
    using ArrayXX = Eigen::Array<T, Eigen::Dynamic, Eigen::Dynamic>;
    Eigen::Map<const ArrayXX> param_array(parameter[0], mean.rows(), mean.cols());

    Eigen::Map<ArrayXX> residual_array(residual, mean.rows(), mean.cols());
    residual_array = (param_array - mean.cast<T>()) / stdev.cast<T>();

    return true;
  }

  Eigen::ArrayXXd mean;
  Eigen::ArrayXXd stdev;
};

}  // namespace industrial_calibration
