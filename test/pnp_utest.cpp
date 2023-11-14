#include <industrial_calibration/optimizations/pnp.h>
#include <industrial_calibration/optimizations/analysis/statistics.h>
#include <industrial_calibration_tests/observation_creator.h>
#include <industrial_calibration_tests/utilities.h>

#include <gtest/gtest.h>

using namespace industrial_calibration;

static const unsigned TARGET_ROWS = 10;
static const unsigned TARGET_COLS = 14;
static const double SPACING = 0.025;
static const double CORRELATION_COEFFICIENT_THRESHOLD = 0.8;
static const std::size_t N_RANDOM_SAMPLES = 30;

void checkCorrelation(const Eigen::MatrixXd& cov)
{
  for (Eigen::Index row = 0; row < cov.rows(); ++row)
  {
    for (Eigen::Index col = 0; col < cov.cols(); ++col)
    {
      // Since the covariance matrix is symmetric, just check the values in the top triangle
      if (row < col) EXPECT_LT(std::abs(cov(row, col)), CORRELATION_COEFFICIENT_THRESHOLD);
    }
  }
}

void printMatrix(const Eigen::MatrixXd& mat, const std::string& title)
{
  Eigen::IOFormat fmt(4, 0, " | ", "\n", "|", "|");
  std::cout << title << ":\n" << mat.format(fmt) << std::endl;
}

class PnP2DTest : public ::testing::Test
{
public:
  PnP2DTest()
    : camera(test::makeKinectCamera())
    , target(TARGET_ROWS, TARGET_COLS, SPACING)
    , target_to_camera(Eigen::Isometry3d::Identity())
  {
    double x = static_cast<double>(TARGET_ROWS - 1) * SPACING / 2.0;
    double y = static_cast<double>(TARGET_COLS - 1) * SPACING / 2.0;
    target_to_camera.translate(Eigen::Vector3d(x, y, 0.4));
    target_to_camera.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  }

  test::Camera camera;
  test::Target target;
  Eigen::Isometry3d target_to_camera;
};

TEST_F(PnP2DTest, PerfectInitialConditions)
{
  PnPProblem problem;
  problem.camera_to_target_guess = target_to_camera.inverse();
  problem.intr = camera.intr;
  EXPECT_NO_THROW(problem.correspondences =
                      test::getCorrespondences(target_to_camera, Eigen::Isometry3d::Identity(), camera, target, true));

  PnPResult result = optimize(problem);
  EXPECT_TRUE(result.converged);
  EXPECT_TRUE(result.camera_to_target.isApprox(target_to_camera.inverse()));
  EXPECT_LT(result.initial_cost_per_obs, 1.0e-15);
  EXPECT_LT(result.final_cost_per_obs, 1.0e-15);

  checkCorrelation(result.covariance.correlation_matrix);
  printMatrix(result.covariance.correlation_matrix, "Correlation");

  std::cout << result.covariance.toString() << std::endl;
}

TEST_F(PnP2DTest, PerturbedInitialCondition)
{
  PnPProblem problem;
  problem.intr = camera.intr;
  problem.correspondences =
      test::getCorrespondences(target_to_camera, Eigen::Isometry3d::Identity(), camera, target, true);

  std::vector<double> residual_acc, pos_acc, ori_acc;
  residual_acc.reserve(N_RANDOM_SAMPLES);
  pos_acc.reserve(N_RANDOM_SAMPLES);
  ori_acc.reserve(N_RANDOM_SAMPLES);

  for (std::size_t i = 0; i < N_RANDOM_SAMPLES; ++i)
  {
    problem.camera_to_target_guess = test::perturbPose(target_to_camera.inverse(), 0.05, 0.05);

    PnPResult result = optimize(problem);

    EXPECT_TRUE(result.converged);
    checkCorrelation(result.covariance.correlation_matrix);

    // Calculate the difference between the transforms (ideally, an identity matrix)
    Eigen::Isometry3d diff = result.camera_to_target * target_to_camera;

    // Accumulate the positional, rotational, and residual errors
    pos_acc.push_back(diff.translation().norm());
    ori_acc.push_back(Eigen::Quaterniond::Identity().angularDistance(Eigen::Quaterniond(diff.linear())));
    residual_acc.push_back(result.final_cost_per_obs);
  }

  // Expect 99% of the outputs (i.e. 3 standard deviations) to be within the corresponding threshold
  double pos_mean, pos_stdev;
  std::tie(pos_mean, pos_stdev) = computeStats(pos_acc);
  EXPECT_LT(pos_mean + 3 * pos_stdev, 1.0e-7);

  double ori_mean, ori_stdev;
  std::tie(ori_mean, ori_stdev) = computeStats(ori_acc);
  EXPECT_LT(ori_mean + 3 * ori_stdev, 1.0e-6);

  double residual_mean, residual_stdev;
  std::tie(residual_mean, residual_stdev) = computeStats(residual_acc);
  EXPECT_LT(residual_mean + 3 * residual_stdev, 1.0e-10);
}

TEST_F(PnP2DTest, BadIntrinsicParameters)
{
  PnPProblem problem;
  problem.camera_to_target_guess = target_to_camera.inverse();
  problem.correspondences =
      test::getCorrespondences(target_to_camera, Eigen::Isometry3d::Identity(), camera, target, true);

  // Tweak the camera intrinsics such that we are optimizing with different values (+/- 1%)
  // than were used to generate the observations
  problem.intr = camera.intr;
  problem.intr.fx() *= 1.01;
  problem.intr.fy() *= 0.99;
  problem.intr.cx() *= 1.01;
  problem.intr.cy() *= 0.99;

  PnPResult result = optimize(problem);

  // The optimization should still converge by moving the camera further away from the target,
  // but the residual error and error in the resulting transform should be much higher
  EXPECT_TRUE(result.converged);
  EXPECT_FALSE(result.camera_to_target.isApprox(target_to_camera.inverse(), 1.0e-3));
  EXPECT_GT(result.final_cost_per_obs, 1.0e-3);

  checkCorrelation(result.covariance.correlation_matrix);
  printMatrix(result.covariance.correlation_matrix, "Correlation");
  std::cout << result.covariance.toString() << std::endl;
}

class PnP3DTest : public ::testing::Test
{
public:
  PnP3DTest() : target(TARGET_ROWS, TARGET_COLS, SPACING), target_to_camera(Eigen::Isometry3d::Identity())
  {
    double x = static_cast<double>(TARGET_ROWS - 1) * SPACING / 2.0;
    double y = static_cast<double>(TARGET_COLS - 1) * SPACING / 2.0;
    target_to_camera.translate(Eigen::Vector3d(x, y, 0.4));
    target_to_camera.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  }

  test::Target target;
  Eigen::Isometry3d target_to_camera;
};

TEST_F(PnP3DTest, PerfectInitialConditions)
{
  PnPProblem3D problem;
  problem.camera_to_target_guess = target_to_camera.inverse();
  EXPECT_NO_THROW(problem.correspondences =
                      test::getCorrespondences(target_to_camera, Eigen::Isometry3d::Identity(), target));

  PnPResult result = optimize(problem);
  EXPECT_TRUE(result.converged);
  EXPECT_TRUE(result.camera_to_target.isApprox(target_to_camera.inverse()));
  EXPECT_LT(result.initial_cost_per_obs, 1.0e-15);
  EXPECT_LT(result.final_cost_per_obs, 1.0e-15);

  checkCorrelation(result.covariance.correlation_matrix);

  // BUG: tests pass but nothing is printed
  //  printMatrix(result.covariance.correlation_matrix, "Correlation");
  //  std::cout << result.covariance.toString() << std::endl;
}

TEST_F(PnP3DTest, PerturbedInitialCondition)
{
  PnPProblem3D problem;
  problem.correspondences = test::getCorrespondences(target_to_camera, Eigen::Isometry3d::Identity(), target);

  std::vector<double> residual_acc, pos_acc, ori_acc;
  residual_acc.reserve(N_RANDOM_SAMPLES);
  pos_acc.reserve(N_RANDOM_SAMPLES);
  ori_acc.reserve(N_RANDOM_SAMPLES);

  for (std::size_t i = 0; i < N_RANDOM_SAMPLES; ++i)
  {
    problem.camera_to_target_guess = test::perturbPose(target_to_camera.inverse(), 0.05, 0.05);

    PnPResult result = optimize(problem);
    EXPECT_TRUE(result.converged);
    checkCorrelation(result.covariance.correlation_matrix);

    // Calculate the difference between the transforms (ideally, an identity matrix)
    Eigen::Isometry3d diff = result.camera_to_target * target_to_camera;

    // Accumulate the positional, rotational, and residual errors
    pos_acc.push_back(diff.translation().norm());
    ori_acc.push_back(Eigen::Quaterniond::Identity().angularDistance(Eigen::Quaterniond(diff.linear())));
    residual_acc.push_back(result.final_cost_per_obs);
  }

  // Expect 99% of the outputs (i.e. 3 standard deviations) to be within the corresponding threshold
  double pos_mean, pos_stdev;
  std::tie(pos_mean, pos_stdev) = computeStats(pos_acc);
  EXPECT_LT(pos_mean + 3 * pos_stdev, 1.0e-7);

  double ori_mean, ori_stdev;
  std::tie(ori_mean, ori_stdev) = computeStats(ori_acc);
  EXPECT_LT(ori_mean + 3 * ori_stdev, 1.0e-6);

  double residual_mean, residual_stdev;
  std::tie(residual_mean, residual_stdev) = computeStats(residual_acc);
  EXPECT_LT(residual_mean + 3 * residual_stdev, 1.0e-10);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
