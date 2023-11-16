#include <industrial_calibration/optimizations/analysis/covariance_analysis.h>
#include <industrial_calibration_tests/circle_fit.h>
#include <industrial_calibration/exceptions.h>

#include <ceres/ceres.h>
#include <cmath>
#include <functional>
#include <gtest/gtest.h>
#include <random>

using namespace industrial_calibration;

static std::mt19937 RAND_GEN(RANDOM_SEED);

template <typename T>
T perturb_normal(T val, T std_dev)
{
  return std::bind(std::normal_distribution<T>{ val, std_dev }, RAND_GEN)();
}

template <typename T>
T perturb_random(T mean, T offset)
{
  return std::bind(std::uniform_real_distribution<T>{ mean - offset, mean + offset }, RAND_GEN)();
}

class CircleFitUnit : public ::testing::Test
{
protected:
  void SetUp() override
  {
    setActualData();
    setInitialGuess();
    setObservations();

    problem.x_center_initial = center_x_initial;
    problem.y_center_initial = center_y_initial;
    problem.radius_initial = radius_initial;
    problem.observations = observations;
  }

  virtual void setActualData() = 0;

  virtual void setInitialGuess() = 0;

  virtual void setObservations() = 0;

  static void printResults(const CircleFitResult& r)
  {
    std::cout << "X: " << r.x_center << std::endl;
    std::cout << "Y: " << r.y_center << std::endl;
    std::cout << "R: " << r.radius << std::endl;
    std::cout << "Covariance Matrix:\n" << r.covariance.covariance_matrix.matrix() << std::endl;
    std::cout << r.covariance.toString() << std::endl;
  }

  std::double_t center_x_actual, center_y_actual, radius_actual;

  CircleFitProblem problem;
  std::double_t center_x_initial, center_y_initial, radius_initial;
  std::vector<Eigen::Vector2d> observations;
};

class CircleFitUnit_PerfectObservations_RandomGuess : public CircleFitUnit
{
protected:
  void setActualData()
  {
    center_x_actual = 0.0;
    center_y_actual = 0.0;
    radius_actual = 1.0;
  }

  void setInitialGuess()
  {
    center_x_initial = perturb_random<std::double_t>(0.0, 1.0);
    center_y_initial = perturb_random<std::double_t>(0.0, 1.0);
    radius_initial = perturb_random<std::double_t>(1.0, 0.9);
  }

  void setObservations()
  {
    std::size_t n_obs_target = 50;
    std::double_t obs_step = 2 * M_PI / n_obs_target;

    for (std::double_t angle = 0; angle < 2 * M_PI; angle += obs_step)
    {
      std::double_t point_x = radius_actual * cos(angle) + center_x_actual;
      std::double_t point_y = radius_actual * sin(angle) + center_y_actual;
      observations.emplace_back(Eigen::Vector2d(point_x, point_y));
    }
  }
};

class CircleFitUnit_SlightlyPerturbed : public CircleFitUnit_PerfectObservations_RandomGuess
{
protected:
  void setObservations() override
  {
    std::size_t n_obs_target = 50;
    std::double_t obs_step = 2 * M_PI / n_obs_target;
    std::double_t std_dev = radius_actual * 0.01;

    for (std::double_t angle = 0; angle < 2 * M_PI; angle += obs_step)
    {
      std::double_t point_x = perturb_normal<std::double_t>(radius_actual * cos(angle) + center_x_actual, std_dev);
      std::double_t point_y = perturb_normal<std::double_t>(radius_actual * sin(angle) + center_y_actual, std_dev);
      observations.emplace_back(Eigen::Vector2d(point_x, point_y));
    }
  }
};

class CircleFitUnit_VeryPerturbed : public CircleFitUnit_PerfectObservations_RandomGuess
{
protected:
  void setObservations() override
  {
    std::size_t n_obs_target = 50;
    std::double_t obs_step = 2 * M_PI / n_obs_target;
    std::double_t std_dev = radius_actual * 0.1;

    for (std::double_t angle = 0; angle < 2 * M_PI; angle += obs_step)
    {
      std::double_t point_x = perturb_normal<std::double_t>(radius_actual * cos(angle) + center_x_actual, std_dev);
      std::double_t point_y = perturb_normal<std::double_t>(radius_actual * sin(angle) + center_y_actual, std_dev);
      observations.emplace_back(Eigen::Vector2d(point_x, point_y));
    }
  }
};

class CircleFitUnit_AnglePerturbed : public CircleFitUnit_PerfectObservations_RandomGuess
{
protected:
  void setObservations() override
  {
    std::size_t n_obs_target = 50;
    std::double_t obs_step = 2 * M_PI / n_obs_target;
    std::double_t std_dev = 0.0;

    for (std::double_t angle = 0; angle < 2 * M_PI; angle += perturb_normal<std::double_t>(obs_step, 0.3))
    {
      std::double_t point_x = perturb_normal<std::double_t>(radius_actual * cos(angle) + center_x_actual, std_dev);
      std::double_t point_y = perturb_normal<std::double_t>(radius_actual * sin(angle) + center_y_actual, std_dev);
      observations.emplace_back(Eigen::Vector2d(point_x, point_y));
    }
  }
};

class CircleFitUnit_ClusteredObservations : public CircleFitUnit_PerfectObservations_RandomGuess
{
protected:
  void setObservations() override
  {
    std::vector<std::double_t> angles{ -0.02,       -0.01,       0.0,  0.01,        0.02,
                                       M_PI - 0.02, M_PI - 0.01, M_PI, M_PI + 0.01, M_PI + 0.02 };

    for (auto angle : angles)
    {
      std::double_t point_x = radius_actual * cos(angle) + center_x_actual;
      std::double_t point_y = radius_actual * sin(angle) + center_y_actual;
      observations.emplace_back(Eigen::Vector2d(point_x, point_y));
    }
  }
};

class CircleFitUnit_ClusteredObservationsPerturbed : public CircleFitUnit_PerfectObservations_RandomGuess
{
protected:
  void setObservations() override
  {
    std::vector<std::double_t> angles{ -0.02,       -0.01,       0.0,  0.01,        0.02,
                                       M_PI - 0.02, M_PI - 0.01, M_PI, M_PI + 0.01, M_PI + 0.02 };

    for (auto angle : angles)
    {
      std::double_t point_x = perturb_normal<std::double_t>(radius_actual * cos(angle) + center_x_actual, 0.05);
      std::double_t point_y = perturb_normal<std::double_t>(radius_actual * sin(angle) + center_y_actual, 0.05);
      observations.emplace_back(Eigen::Vector2d(point_x, point_y));
    }
  }
};

class CircleFitUnit_TwoObsX : public CircleFitUnit_PerfectObservations_RandomGuess
{
protected:
  void setObservations() override
  {
    observations.emplace_back(Eigen::Vector2d(-1.0, 0.0));
    observations.emplace_back(Eigen::Vector2d(1.0, 0.0));
  }
};

class CircleFitUnit_TwoObsY : public CircleFitUnit_PerfectObservations_RandomGuess
{
protected:
  void setObservations() override
  {
    observations.emplace_back(Eigen::Vector2d(0.0, -1.0));
    observations.emplace_back(Eigen::Vector2d(0.0, 1.0));
  }
};

class CircleFitUnit_ThreeObs : public CircleFitUnit_PerfectObservations_RandomGuess
{
protected:
  void setObservations() override
  {
    std::size_t n_obs_target = 3;
    std::double_t obs_step = 2 * M_PI / n_obs_target;

    for (std::double_t angle = 0; angle < 2 * M_PI; angle += obs_step)
    {
      std::double_t point_x = radius_actual * cos(angle) + center_x_actual;
      std::double_t point_y = radius_actual * sin(angle) + center_y_actual;
      observations.emplace_back(Eigen::Vector2d(point_x, point_y));
    }
  }
};

class CircleFitUnit_OneObs : public CircleFitUnit_PerfectObservations_RandomGuess
{
protected:
  void setObservations() override { observations.emplace_back(Eigen::Vector2d(0.0, 0.0)); }
};

TEST_F(CircleFitUnit_PerfectObservations_RandomGuess, FitCircleToPerfectObs)
{
  CircleFitResult result;
  EXPECT_NO_THROW(result = optimize(problem));
  printResults(result);

  // assert that the optimization converged and expect that the results are close to the real values
  ASSERT_TRUE(result.converged);
  EXPECT_NEAR(center_x_actual, result.x_center, 1e-5);
  EXPECT_NEAR(center_y_actual, result.y_center, 1e-5);
  EXPECT_NEAR(radius_actual, result.radius, 1e-5);

  // assert 3x3 covariance matrix
  ASSERT_EQ(result.covariance.covariance_matrix.rows(), 3);
  ASSERT_EQ(result.covariance.covariance_matrix.cols(), 3);

  // assert 3x3 correlation matrix
  ASSERT_EQ(result.covariance.correlation_matrix.rows(), 3);
  ASSERT_EQ(result.covariance.correlation_matrix.cols(), 3);

  // expect both matrices to be symmetric
  EXPECT_NEAR(result.covariance.covariance_matrix(0, 1), result.covariance.covariance_matrix(1, 0),
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance.covariance_matrix(0, 2), result.covariance.covariance_matrix(2, 0),
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance.covariance_matrix(1, 2), result.covariance.covariance_matrix(2, 1),
              std::numeric_limits<double>::epsilon());
  EXPECT_TRUE(result.covariance.covariance_matrix.isApprox(
      result.covariance.covariance_matrix.transpose()));  // TODO: use this instead in future

  EXPECT_TRUE(result.covariance.correlation_matrix.isApprox(result.covariance.correlation_matrix.transpose()));

  // expect diagonal elements to be positive
  EXPECT_GE(result.covariance.covariance_matrix(0, 0), 0.0);
  EXPECT_GE(result.covariance.covariance_matrix(1, 1), 0.0);
  EXPECT_GE(result.covariance.covariance_matrix(2, 2), 0.0);
  EXPECT_GE(result.covariance.correlation_matrix(0, 0), 0.0);
  EXPECT_GE(result.covariance.correlation_matrix(1, 1), 0.0);
  EXPECT_GE(result.covariance.correlation_matrix(2, 2), 0.0);

  // expect off-diagonal elements to be near-zero
  EXPECT_NEAR(result.covariance.covariance_matrix(0, 1), 0.0, 1e-5);
  EXPECT_NEAR(result.covariance.covariance_matrix(0, 2), 0.0, 1e-5);
  EXPECT_NEAR(result.covariance.covariance_matrix(1, 2), 0.0, 1e-5);

  // with perfect evenly-spaced observations we expect the standard deviations of X and Y to be near-identical
  EXPECT_NEAR(result.covariance.covariance_matrix(0, 0), result.covariance.covariance_matrix(1, 1), 1e-5);

  // checks for NamedParam output
  // expect three parameters for standard deviations
  ASSERT_EQ(result.covariance.standard_deviations.size(), 3);

  // expect three parameters for off-diagonal covariance
  ASSERT_EQ(result.covariance.covariances.size(), 3);

  // expect three parameters for off-diagonal correlation coefficients
  ASSERT_EQ(result.covariance.correlation_coeffs.size(), 3);

  // expect names to match what was set in the problem
  EXPECT_EQ(result.covariance.standard_deviations[0].names.first, problem.labels[0]);
  EXPECT_EQ(result.covariance.standard_deviations[0].names.second, "");
  EXPECT_EQ(result.covariance.standard_deviations[1].names.first, problem.labels[1]);
  EXPECT_EQ(result.covariance.standard_deviations[1].names.second, "");
  EXPECT_EQ(result.covariance.standard_deviations[2].names.first, problem.labels[2]);
  EXPECT_EQ(result.covariance.standard_deviations[2].names.second, "");

  EXPECT_EQ(result.covariance.covariances[0].names.first, problem.labels[0]);
  EXPECT_EQ(result.covariance.covariances[0].names.second, problem.labels[1]);
  EXPECT_EQ(result.covariance.covariances[1].names.first, problem.labels[0]);
  EXPECT_EQ(result.covariance.covariances[1].names.second, problem.labels[2]);
  EXPECT_EQ(result.covariance.covariances[2].names.first, problem.labels[1]);
  EXPECT_EQ(result.covariance.covariances[2].names.second, problem.labels[2]);

  EXPECT_EQ(result.covariance.correlation_coeffs[0].names.first, problem.labels[0]);
  EXPECT_EQ(result.covariance.correlation_coeffs[0].names.second, problem.labels[1]);
  EXPECT_EQ(result.covariance.correlation_coeffs[1].names.first, problem.labels[0]);
  EXPECT_EQ(result.covariance.correlation_coeffs[1].names.second, problem.labels[2]);
  EXPECT_EQ(result.covariance.correlation_coeffs[2].names.first, problem.labels[1]);
  EXPECT_EQ(result.covariance.correlation_coeffs[2].names.second, problem.labels[2]);

  // expect values to match contents of matrices
  EXPECT_EQ(result.covariance.standard_deviations[0].value, result.covariance.correlation_matrix(0, 0));
  EXPECT_EQ(result.covariance.standard_deviations[1].value, result.covariance.correlation_matrix(1, 1));
  EXPECT_EQ(result.covariance.standard_deviations[2].value, result.covariance.correlation_matrix(2, 2));

  EXPECT_EQ(result.covariance.covariances[0].value, result.covariance.covariance_matrix(0, 1));
  EXPECT_EQ(result.covariance.covariances[1].value, result.covariance.covariance_matrix(0, 2));
  EXPECT_EQ(result.covariance.covariances[2].value, result.covariance.covariance_matrix(1, 2));

  EXPECT_EQ(result.covariance.correlation_coeffs[0].value, result.covariance.correlation_matrix(0, 1));
  EXPECT_EQ(result.covariance.correlation_coeffs[1].value, result.covariance.correlation_matrix(0, 2));
  EXPECT_EQ(result.covariance.correlation_coeffs[2].value, result.covariance.correlation_matrix(1, 2));
}

TEST_F(CircleFitUnit_ClusteredObservations, FitCircleToClusteredObs)
{
  CircleFitResult result;
  EXPECT_NO_THROW(result = optimize(problem));
  printResults(result);

  // assert 3x3 covariance matrix
  ASSERT_EQ(result.covariance.covariance_matrix.rows(), 3);
  ASSERT_EQ(result.covariance.covariance_matrix.cols(), 3);

  // expect matrix to be symmetric
  EXPECT_NEAR(result.covariance.covariance_matrix(0, 1), result.covariance.covariance_matrix(1, 0),
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance.covariance_matrix(0, 2), result.covariance.covariance_matrix(2, 0),
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance.covariance_matrix(1, 2), result.covariance.covariance_matrix(2, 1),
              std::numeric_limits<double>::epsilon());

  // expect diagonal elements to be positive
  EXPECT_GE(result.covariance.covariance_matrix(0, 0), 0.0);
  EXPECT_GE(result.covariance.covariance_matrix(1, 1), 0.0);
  EXPECT_GE(result.covariance.covariance_matrix(2, 2), 0.0);

  // std dev of Y-coord should be greater than std dev of X-coord
  EXPECT_GT(result.covariance.covariance_matrix(1, 1), result.covariance.covariance_matrix(0, 0));
}

TEST_F(CircleFitUnit_ClusteredObservationsPerturbed, FitCircleToClusteredObsPerturbed)
{
  CircleFitResult result;
  EXPECT_NO_THROW(result = optimize(problem));
  printResults(result);

  // assert 3x3 covariance matrix
  ASSERT_EQ(result.covariance.covariance_matrix.rows(), 3);
  ASSERT_EQ(result.covariance.covariance_matrix.cols(), 3);

  // expect matrix to be symmetric
  EXPECT_NEAR(result.covariance.covariance_matrix(0, 1), result.covariance.covariance_matrix(1, 0),
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance.covariance_matrix(0, 2), result.covariance.covariance_matrix(2, 0),
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance.covariance_matrix(1, 2), result.covariance.covariance_matrix(2, 1),
              std::numeric_limits<double>::epsilon());

  // expect diagonal elements to be positive
  EXPECT_GE(result.covariance.covariance_matrix(0, 0), 0.0);
  EXPECT_GE(result.covariance.covariance_matrix(1, 1), 0.0);
  EXPECT_GE(result.covariance.covariance_matrix(2, 2), 0.0);

  // std dev of Y-coord should be greater than std dev of X-coord
  EXPECT_GT(result.covariance.covariance_matrix(1, 1), result.covariance.covariance_matrix(0, 0));
}

// Do optimization to fit a circle to two observed points.
TEST_F(CircleFitUnit_TwoObsX, FitCircleToTwoPoints)
{
  CircleFitResult result;
  EXPECT_NO_THROW(result = optimize(problem));
  printResults(result);

  // assert 3x3 covariance matrix
  ASSERT_EQ(result.covariance.covariance_matrix.rows(), 3);
  ASSERT_EQ(result.covariance.covariance_matrix.cols(), 3);

  // expect matrix to be symmetric
  // BUG: flaky way to evaluate, since difference is sometimes (but rarely) greater than epsilon
  EXPECT_NEAR(result.covariance.covariance_matrix(0, 1), result.covariance.covariance_matrix(1, 0),
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance.covariance_matrix(0, 2), result.covariance.covariance_matrix(2, 0),
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance.covariance_matrix(1, 2), result.covariance.covariance_matrix(2, 1),
              std::numeric_limits<double>::epsilon());

  // expect diagonal elements to be positive
  EXPECT_GE(result.covariance.covariance_matrix(0, 0), 0.0);
  EXPECT_GE(result.covariance.covariance_matrix(1, 1), 0.0);
  EXPECT_GE(result.covariance.covariance_matrix(2, 2), 0.0);

  // expect covariance.covariance_matrix between Y and R to be close to 1
  EXPECT_NEAR(fabs(result.covariance.correlation_matrix(1, 2)), 1.0, 1e-5);

  // expect covariance between X and R to be somewhat close to 0
  // TODO: fix, not always close to zero due to random initial conditions of problem
  //  EXPECT_NEAR(result.covariance(0, 2), 0.0, 1e-2);

  // expect correlation coefficient between y and r to be comparatively large, and all others to be small
  std::vector<NamedParam> outside_thresh = result.covariance.getCorrelationCoeffOutsideThreshold(0.1);
  EXPECT_EQ(outside_thresh.size(), 1);
  EXPECT_EQ(outside_thresh.front().names, std::make_pair(std::string("y"), std::string("r")));
}

// Do optimization to fit a circle to two observed points.
TEST_F(CircleFitUnit_TwoObsY, FitCircleToTwoPoints)
{
  CircleFitResult result;
  EXPECT_NO_THROW(result = optimize(problem));
  printResults(result);

  // assert 3x3 covariance matrix
  ASSERT_EQ(result.covariance.covariance_matrix.rows(), 3);
  ASSERT_EQ(result.covariance.covariance_matrix.cols(), 3);

  // expect matrix to be symmetric
  EXPECT_NEAR(result.covariance.covariance_matrix(0, 1), result.covariance.covariance_matrix(1, 0),
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance.covariance_matrix(0, 2), result.covariance.covariance_matrix(2, 0),
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance.covariance_matrix(1, 2), result.covariance.covariance_matrix(2, 1),
              std::numeric_limits<double>::epsilon());

  // expect diagonal elements to be positive
  EXPECT_GE(result.covariance.covariance_matrix(0, 0), 0.0);
  EXPECT_GE(result.covariance.covariance_matrix(1, 1), 0.0);
  EXPECT_GE(result.covariance.covariance_matrix(2, 2), 0.0);

  // expect covariance between X and R to be close to 1
  EXPECT_NEAR(fabs(result.covariance.correlation_matrix(0, 2)), 1.0, 1e-5);

  // expect covariance between Y and R to be somewhat close to 0
  // TODO: fix, not always close to zero due to random initial conditions of problem
  //  EXPECT_NEAR(result.covariance(1, 2), 0.0, 1e-2);

  // expect correlation coefficient between x and r to be comparatively large, and all others to be small
  std::vector<NamedParam> outside_thresh = result.covariance.getCorrelationCoeffOutsideThreshold(0.1);
  EXPECT_EQ(outside_thresh.size(), 1);
  EXPECT_EQ(outside_thresh.front().names, std::make_pair(std::string("x"), std::string("r")));
}

// Do optimization to fit a circle to just one observed point.
TEST_F(CircleFitUnit_OneObs, FitCircleToOnePoint)
{
  CircleFitResult result;
  EXPECT_NO_THROW(result = optimize(problem));
  printResults(result);

  // assert 3x3 covariance matrix
  ASSERT_EQ(result.covariance.covariance_matrix.rows(), 3);
  ASSERT_EQ(result.covariance.covariance_matrix.cols(), 3);

  // expect matrix to be symmetric
  EXPECT_NEAR(result.covariance.covariance_matrix(0, 1), result.covariance.covariance_matrix(1, 0),
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance.covariance_matrix(0, 2), result.covariance.covariance_matrix(2, 0),
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance.covariance_matrix(1, 2), result.covariance.covariance_matrix(2, 1),
              std::numeric_limits<double>::epsilon());

  // expect diagonal elements to be positive
  EXPECT_GE(result.covariance.covariance_matrix(0, 0), 0.0);
  EXPECT_GE(result.covariance.covariance_matrix(1, 1), 0.0);
  EXPECT_GE(result.covariance.covariance_matrix(2, 2), 0.0);

  // expect magnitudes of all off-diagonal elements to be close to 1
  EXPECT_NEAR(fabs(result.covariance.correlation_matrix(0, 1)), 1.0, 1e-5);
  EXPECT_NEAR(fabs(result.covariance.correlation_matrix(0, 2)), 1.0, 1e-5);
  EXPECT_NEAR(fabs(result.covariance.correlation_matrix(1, 2)), 1.0, 1e-5);

  // expect all correlation coefficients to be greater than 0.1
  std::vector<NamedParam> outside_thresh = result.covariance.getCorrelationCoeffOutsideThreshold(0.1);
  EXPECT_EQ(outside_thresh.size(), 3);
}

TEST_F(CircleFitUnit_ThreeObs, FitCircleToThreePoints)
{
  CircleFitResult result;
  EXPECT_NO_THROW(result = optimize(problem));
  printResults(result);

  // assert 3x3 covariance matrix
  ASSERT_EQ(result.covariance.covariance_matrix.rows(), 3);
  ASSERT_EQ(result.covariance.covariance_matrix.cols(), 3);

  // expect matrix to be symmetric
  EXPECT_NEAR(result.covariance.covariance_matrix(0, 1), result.covariance.covariance_matrix(1, 0),
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance.covariance_matrix(0, 2), result.covariance.covariance_matrix(2, 0),
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance.covariance_matrix(1, 2), result.covariance.covariance_matrix(2, 1),
              std::numeric_limits<double>::epsilon());

  // expect diagonal elements to be positive
  EXPECT_GE(result.covariance.covariance_matrix(0, 0), 0.0);
  EXPECT_GE(result.covariance.covariance_matrix(1, 1), 0.0);
  EXPECT_GE(result.covariance.covariance_matrix(2, 2), 0.0);

  // expect off-diagonal elements to be close to 0
  EXPECT_NEAR(result.covariance.covariance_matrix(0, 1), 0.0, 1e-5);
  EXPECT_NEAR(result.covariance.covariance_matrix(0, 2), 0.0, 1e-5);
  EXPECT_NEAR(result.covariance.covariance_matrix(1, 2), 0.0, 1e-5);

  // expect all correlation coefficients to be less than 0.1
  std::vector<NamedParam> outside_thresh = result.covariance.getCorrelationCoeffOutsideThreshold(0.1);
  EXPECT_EQ(outside_thresh.size(), 0);
}

// This test exercises the the variations of the computeCovariance function.
// Since these functions are usually set up inside an optimize() function, the CircleFit problem's setup and
// optimization is duplicated here.
TEST(CovarianceAnalysis, CovarianceAnalysisFunctions)
{
  double x = perturb_random<std::double_t>(0.0, 1.0);
  double y = perturb_random<std::double_t>(0.0, 1.0);
  double r = perturb_random<std::double_t>(1.0, 0.9);

  double r_sqrt = sqrt(r);

  std::vector<double> x_internal(1, x);
  std::vector<double> y_internal(1, y);
  std::vector<double> r_sqrt_internal(1, r_sqrt);

  std::vector<double> params_internal(3);
  params_internal[0] = x;
  params_internal[1] = y;
  params_internal[2] = r_sqrt;

  CircleFitResult result;

  ceres::Problem problem;

  ceres::LossFunction* loss_fn = nullptr;

  std::vector<Eigen::Vector2d> observations;
  std::size_t n_obs_target = 50;
  std::double_t obs_step = 2 * M_PI / n_obs_target;
  for (std::double_t angle = 0; angle < 2 * M_PI; angle += obs_step)
  {
    std::double_t point_x = 1.0 * cos(angle);
    std::double_t point_y = 1.0 * sin(angle);
    observations.emplace_back(Eigen::Vector2d(point_x, point_y));
  }

  for (auto obs : observations)
  {
    auto* cost_fn = new CircleDistCost(obs.x(), obs.y());

    // This uses the alternative CircleDist cost function where the three optimization variables are separate parameter
    // blocks
    auto* cost_block = new ceres::AutoDiffCostFunction<CircleDistCost, 1, 1, 1, 1>(cost_fn);
    problem.AddResidualBlock(cost_block, loss_fn, params_internal.data(), params_internal.data() + 1,
                             params_internal.data() + 2);
  }

  ceres::Solver::Options options;
  options.max_num_iterations = 500;
  options.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << std::endl;

  result.converged = summary.termination_type == ceres::TerminationType::CONVERGENCE;
  result.x_center = params_internal[0];
  result.y_center = params_internal[1];
  result.radius = pow(params_internal[2], 2);
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;

  // sanity-check optimization results
  ASSERT_TRUE(result.converged);
  EXPECT_NEAR(0.0, result.x_center, 1e-5);
  EXPECT_NEAR(0.0, result.y_center, 1e-5);
  EXPECT_NEAR(1.0, result.radius, 1e-5);

  std::map<const double*, std::vector<std::string>> labels_all;
  labels_all[params_internal.data()] = { "circle_x" };
  labels_all[params_internal.data() + 1] = { "circle_y" };
  labels_all[params_internal.data() + 2] = { "circle_r" };

  std::map<const double*, std::vector<std::string>> labels_x_r;
  labels_x_r[params_internal.data()] = { "circle_x" };
  labels_x_r[params_internal.data() + 2] = { "circle_r" };

  std::vector<const double*> param_blocks_x_r(2);
  param_blocks_x_r[0] = params_internal.data();
  param_blocks_x_r[1] = params_internal.data() + 2;

  CovarianceResult covariance_labeled = computeCovariance(problem, labels_all);

  EXPECT_EQ(covariance_labeled.standard_deviations.size(), 3);
  EXPECT_EQ(covariance_labeled.covariances.size(), 3);
  EXPECT_EQ(covariance_labeled.correlation_coeffs.size(), 3);

  CovarianceResult covariance_generic_names = computeCovariance(problem);

  EXPECT_EQ(covariance_generic_names.standard_deviations.size(), 3);
  EXPECT_EQ(covariance_generic_names.covariances.size(), 3);
  EXPECT_EQ(covariance_generic_names.correlation_coeffs.size(), 3);

  CovarianceResult cov_x_r_generic_names = computeCovariance(problem, param_blocks_x_r);

  EXPECT_EQ(cov_x_r_generic_names.standard_deviations.size(), 2);
  EXPECT_EQ(cov_x_r_generic_names.covariances.size(), 1);
  EXPECT_EQ(cov_x_r_generic_names.correlation_coeffs.size(), 1);

  EXPECT_EQ(cov_x_r_generic_names.standard_deviations[0].names.first, "block0_element0");
  EXPECT_EQ(cov_x_r_generic_names.standard_deviations[0].value, covariance_labeled.standard_deviations[0].value);

  EXPECT_EQ(cov_x_r_generic_names.standard_deviations[1].names.first, "block1_element0");
  EXPECT_EQ(cov_x_r_generic_names.standard_deviations[1].value, covariance_labeled.standard_deviations[2].value);

  EXPECT_EQ(cov_x_r_generic_names.covariances[0].names.first, "block0_element0");
  EXPECT_EQ(cov_x_r_generic_names.covariances[0].names.second, "block1_element0");
  EXPECT_EQ(cov_x_r_generic_names.covariances[0].value, covariance_labeled.covariances[1].value);

  EXPECT_EQ(cov_x_r_generic_names.correlation_coeffs[0].names.first, "block0_element0");
  EXPECT_EQ(cov_x_r_generic_names.correlation_coeffs[0].names.second, "block1_element0");
  EXPECT_EQ(cov_x_r_generic_names.correlation_coeffs[0].value, covariance_labeled.correlation_coeffs[1].value);

  CovarianceResult cov_x_r_labeled = computeCovariance(problem, param_blocks_x_r, labels_x_r);

  EXPECT_EQ(cov_x_r_labeled.standard_deviations.size(), 2);
  EXPECT_EQ(cov_x_r_labeled.covariances.size(), 1);
  EXPECT_EQ(cov_x_r_labeled.correlation_coeffs.size(), 1);

  EXPECT_EQ(cov_x_r_labeled.standard_deviations[0].names.first, covariance_labeled.standard_deviations[0].names.first);
  EXPECT_EQ(cov_x_r_labeled.standard_deviations[0].value, covariance_labeled.standard_deviations[0].value);

  EXPECT_EQ(cov_x_r_labeled.standard_deviations[1].names.first, covariance_labeled.standard_deviations[2].names.first);
  EXPECT_EQ(cov_x_r_labeled.standard_deviations[1].value, covariance_labeled.standard_deviations[2].value);

  EXPECT_EQ(cov_x_r_labeled.covariances[0].names.first, covariance_labeled.covariances[1].names.first);
  EXPECT_EQ(cov_x_r_labeled.covariances[0].names.second, covariance_labeled.covariances[1].names.second);
  EXPECT_EQ(cov_x_r_labeled.covariances[0].value, covariance_labeled.covariances[1].value);

  EXPECT_EQ(cov_x_r_labeled.correlation_coeffs[0].names.first, covariance_labeled.correlation_coeffs[1].names.first);
  EXPECT_EQ(cov_x_r_labeled.correlation_coeffs[0].names.second, covariance_labeled.correlation_coeffs[1].names.second);
  EXPECT_EQ(cov_x_r_labeled.correlation_coeffs[0].value, covariance_labeled.correlation_coeffs[1].value);

  std::cout << "covariance_labeled\n" << covariance_labeled.toString() << std::endl;
  std::cout << "covariance_generic_names\n" << covariance_generic_names.toString() << std::endl;
  std::cout << "cov_x_r_generic_names\n" << cov_x_r_generic_names.toString() << std::endl;
  std::cout << "cov_x_r_labeled\n" << cov_x_r_labeled.toString() << std::endl;

  // expect exception if number of label vectors is different from number of parameter blocks
  EXPECT_THROW(computeCovariance(problem, param_blocks_x_r, labels_all), CovarianceException);

  // expect exception if number of labels for a parameter block is different from the number of parameters in that block
  // in the problem
  std::map<const double*, std::vector<std::string>> labels_x_r_wrong_size;
  labels_x_r_wrong_size[params_internal.data()] = { "circle_x", "circle_x_extra_entry" };
  labels_x_r_wrong_size[params_internal.data() + 2] = { "circle_r" };
  EXPECT_THROW(computeCovariance(problem, param_blocks_x_r, labels_x_r_wrong_size), CovarianceException);

  // expect exception with empty parameter block vector
  EXPECT_THROW(computeCovariance(problem, std::vector<const double*>(), labels_all), CovarianceException);

  // expect exception with empty labels vector
  EXPECT_THROW(computeCovariance(problem, param_blocks_x_r, std::map<const double*, std::vector<std::string>>()),
               CovarianceException);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
