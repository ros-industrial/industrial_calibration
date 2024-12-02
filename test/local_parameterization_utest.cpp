#include <industrial_calibration/optimizations/maximum_likelihood.h>
#include <industrial_calibration/optimizations/local_parameterization.h>

#include <ceres/problem.h>
#include <ceres/dynamic_autodiff_cost_function.h>
#include <ceres/solver.h>
#include <gtest/gtest.h>

using namespace industrial_calibration;

TEST(LocalParameterizationTests, SubsetParameterization)
{
  // Create matrices of mean and standard deviation values for each parameter
  Eigen::ArrayXXd mean(Eigen::ArrayXXd::Random(6, 4));
  const double stdev_val = 0.1;
  Eigen::ArrayXXd stdev(Eigen::ArrayXXd::Constant(6, 4, stdev_val));

  // Create a matrix of random values scaled up from a range of [0, 1]
  const double scale = 10.0;
  Eigen::ArrayXXd params(Eigen::ArrayXXd::Random(6, 4) * scale);

  Eigen::IOFormat fmt(4, 0, "|", "\n", "|", "|");
  std::cout << "Mean\n" << mean.format(fmt) << std::endl;
  std::cout << "Original Parameters:\n" << params.format(fmt) << std::endl;

  auto* ml = new MaximumLikelihood(mean, stdev);

  // Run an optimization to see if we can drive the random numbers to the mean
  ceres::Problem problem;
  auto* cost_block = new ceres::DynamicAutoDiffCostFunction<MaximumLikelihood>(ml);
  cost_block->AddParameterBlock(params.size());
  cost_block->SetNumResiduals(params.size());
  problem.AddResidualBlock(cost_block, nullptr, params.data());

  // All values
  {
    std::map<const double*, std::vector<int>> mask;
    mask[params.data()] = std::vector<int>(params.size());
    std::iota(mask[params.data()].begin(), mask[params.data()].end(), 0);
    EXPECT_NO_THROW(addSubsetParameterization(problem, mask));

    // Expect there to be no parameterization, but the entire block should be constant
#if CERES_VERSION_LT_2_1
    EXPECT_EQ(problem.GetParameterization(params.data()), nullptr);
#else
    EXPECT_EQ(problem.GetManifold(params.data()), nullptr);
#endif
    EXPECT_TRUE(problem.IsParameterBlockConstant(params.data()));
  }

  // Index out of range
  {
    std::map<const double*, std::vector<int>> mask;
    mask[params.data()] = std::vector<int>(params.size());
    int bad_idx = params.size() * 2;
    mask[params.data()].insert(mask[params.data()].begin(), { bad_idx, 0, 1, 2 });
    EXPECT_THROW(addSubsetParameterization(problem, mask), OptimizationException);
#if CERES_VERSION_LT_2_1
    EXPECT_EQ(problem.GetParameterization(params.data()), nullptr);
#else
    EXPECT_EQ(problem.GetManifold(params.data()), nullptr);
#endif
  }

  // Empty mask
  {
    std::map<const double*, std::vector<int>> mask;
    EXPECT_NO_THROW(addSubsetParameterization(problem, mask));
    // An empty mask should not have added any local parameterization
#if CERES_VERSION_LT_2_1
    EXPECT_EQ(problem.GetParameterization(params.data()), nullptr);
#else
    EXPECT_EQ(problem.GetManifold(params.data()), nullptr);
#endif
  }

  // Hold the zero-th row constant
  Eigen::ArrayXXd original_params = params;
  {
    std::map<const double*, std::vector<int>> mask;
    mask[params.data()] = std::vector<int>();
    // Remember, Eigen stores values internally in column-wise order
    for (Eigen::Index i = 0; i < params.cols(); ++i)
    {
      mask[params.data()].push_back(i * params.rows());
    }
    EXPECT_NO_THROW(addSubsetParameterization(problem, mask));
#if CERES_VERSION_LT_2_1
    EXPECT_NE(problem.GetParameterization(params.data()), nullptr);
#else
    EXPECT_NE(problem.GetManifold(params.data()), nullptr);
#endif
  }

  // Solve the optimization
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  EXPECT_TRUE(summary.termination_type == ceres::CONVERGENCE);

  std::cout << "Optimized Parameters\n" << params.format(fmt) << std::endl;

  // Calculate the difference between the parameters
  Eigen::ArrayXXd diff = original_params - params;
  EXPECT_LE(diff.row(0).abs().sum(), std::numeric_limits<double>::epsilon());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
