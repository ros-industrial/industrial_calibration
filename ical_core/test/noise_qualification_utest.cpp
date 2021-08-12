#include <ical_core/optimizations/analysis/noise_qualification.h>
#include <ical_core/optimizations/pnp.h>
#include <ical_core/optimizations/utils/ceres_math_utilities.h>
#include <ical_core_tests/utilities.h>
#include <ical_core_tests/observation_creator.h>

#include <random>
#include <gtest/gtest.h>

using namespace industrial_calibration;

static const unsigned TARGET_ROWS = 10;
static const unsigned TARGET_COLS = 14;
static const double SPACING = 0.025;
static const double CAMERA_STANDOFF = 0.5;

TEST(NoiseTest, QuatMeanTest)
{
  // base quaternion
  Eigen::Quaterniond q_mean(1, 0, 0, 0);

  // Make a lot of quaternions randomly oriented about the x-axis
  std::mt19937 mt_rand(RCT_RANDOM_SEED);
  double stdev = M_PI / 8.0;
  std::normal_distribution<double> dist(0.0, stdev);

  std::size_t n = 10000;
  std::vector<Eigen::Quaterniond> random_q;
  random_q.reserve(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    random_q.push_back(q_mean * Eigen::AngleAxisd(dist(mt_rand), Eigen::Vector3d::UnitX()));
  }

  // average new quats
  QuaternionStats q_stats = computeQuaternionStats(random_q);

  // The two quaternions are 2 pi rad apart, so the mean should be ~PI away from both
  EXPECT_LT(q_mean.angularDistance(q_stats.mean), 1.0 * M_PI / 180.0);
  EXPECT_LT(std::abs(stdev - q_stats.stdev), 1.0 * M_PI / 180.0);
}

class NoiseQualification2D : public ::testing::Test
{
public:
  NoiseQualification2D()
    : target(TARGET_ROWS, TARGET_COLS, SPACING)
    , camera(test::makeKinectCamera())
    , target_to_camera(Eigen::Isometry3d::Identity())
    , mt_rand(RCT_RANDOM_SEED)
  {
    // Put the camera over the center of the target facing normal to the target
    double x = static_cast<double>(TARGET_ROWS - 1) * SPACING / 2.0;
    double y = static_cast<double>(TARGET_COLS - 1) * SPACING / 2.0;
    target_to_camera.translate(Eigen::Vector3d(x, y, CAMERA_STANDOFF));
    target_to_camera.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  }

  Correspondence2D3D::Set createCorrespondences()
  {
    Correspondence2D3D::Set out;
    EXPECT_NO_THROW(
        out = test::getCorrespondences(target_to_camera, Eigen::Isometry3d::Identity(), camera, target, true););
    return out;
  }

  Correspondence2D3D::Set createNoisyCorrespondences(double mean, double stdev)
  {
    std::normal_distribution<double> dist(mean, stdev);
    Correspondence2D3D::Set out = createCorrespondences();
    for (auto& correspondence : out)
    {
      Eigen::Vector2d noise;
      noise << dist(mt_rand), dist(mt_rand);
      correspondence.in_image += noise;
    }
    return out;
  }

  test::Target target;
  test::Camera camera;
  Eigen::Isometry3d target_to_camera;
  Correspondence2D3D::Set correspondences;
  std::mt19937 mt_rand;
};

TEST_F(NoiseQualification2D, PerfectData)
{
  // reserve observations
  const std::size_t obs_cnt = 35;
  std::vector<PnPProblem> ideal_problem_set;
  ideal_problem_set.reserve(obs_cnt);

  // create observations
  Eigen::Isometry3d expected_pose = target_to_camera.inverse();
  for (std::size_t i = 0; i < obs_cnt; ++i)
  {
    PnPProblem problem;
    problem.camera_to_target_guess = expected_pose;
    problem.intr = camera.intr;
    problem.correspondences = createCorrespondences();

    ideal_problem_set.push_back(problem);
  }

  PnPNoiseStat results = qualifyNoise2D(ideal_problem_set);

  EXPECT_TRUE(results.p_stat.mean.isApprox(expected_pose.translation()));
  EXPECT_LT(results.p_stat.stdev.norm(), 1.0e-15);
  EXPECT_LT(results.q_stat.mean.angularDistance(Eigen::Quaterniond(expected_pose.linear())), 1.0e-15);
  EXPECT_LT(results.q_stat.stdev, 1.0e-15);
}

TEST_F(NoiseQualification2D, NoisyData)
{
  // reserve observations
  std::size_t obs_cnt = 35;
  std::vector<PnPProblem> perturbed_problem_set;
  perturbed_problem_set.reserve(obs_cnt);

  // add noise boilerplate
  const double mean = 0.0;
  const double stdev = 2.0;

  // create observations
  Eigen::Isometry3d expected_pose = target_to_camera.inverse();
  for (std::size_t i = 0; i < obs_cnt; ++i)
  {
    PnPProblem problem;
    problem.camera_to_target_guess = expected_pose;
    problem.intr = camera.intr;
    problem.correspondences = createNoisyCorrespondences(mean, stdev);

    perturbed_problem_set.push_back(problem);
  }

  PnPNoiseStat results = qualifyNoise2D(perturbed_problem_set);

  // Project the mean camera to target translation into the camera
  Eigen::Vector2d uv = projectPoint(camera.intr, results.p_stat.mean);

  // Take the differential of the camera projection equations to estimate the change in [x,y,z] that is expected from
  // applying camera image noise of [stdev, stdev] u = fx * (x / z) + cx --> dx = du * (z / fx) v = fy * (y / z) + cy
  // --> dy = du * (z / fy) z = fx * x / (u - cx) --> dz = du * -1.0 * (fx * x) / (u - cx)^2
  Eigen::Vector3d delta;
  delta.x() = stdev * results.p_stat.mean.z() / camera.intr.fx();
  delta.y() = stdev * results.p_stat.mean.z() / camera.intr.fy();
  delta.z() = stdev * -1.0 * camera.intr.fx() * results.p_stat.mean.x() / std::pow((uv(0) - camera.intr.cx()), 2.0);

  EXPECT_LT(results.p_stat.stdev.norm(), delta.norm());
  EXPECT_TRUE(results.p_stat.mean.isApprox(expected_pose.translation(), delta.norm()));

  // Expect the mean quaternion to be within 1 sigma of the expected orientation
  EXPECT_LT(results.q_stat.mean.angularDistance(Eigen::Quaterniond(expected_pose.linear())), results.q_stat.stdev);
}

TEST_F(NoiseQualification2D, NoisyDataPerturbedGuess)
{
  // reserve observations
  std::size_t obs_cnt = 35;
  std::vector<PnPProblem> perturbed_problem_set;
  perturbed_problem_set.reserve(obs_cnt);

  // add noise boilerplate
  const double mean = 0.0;
  const double stdev = 2.0;

  // create observations
  Eigen::Isometry3d expected_pose = target_to_camera.inverse();
  for (std::size_t i = 0; i < obs_cnt; ++i)
  {
    PnPProblem problem;
    problem.camera_to_target_guess = test::perturbPose(expected_pose, 0.01, 0.01);
    problem.intr = camera.intr;
    problem.correspondences = createNoisyCorrespondences(mean, stdev);

    perturbed_problem_set.push_back(problem);
  }

  PnPNoiseStat results = qualifyNoise2D(perturbed_problem_set);

  // Project the mean camera to target translation into the camera
  Eigen::Vector2d uv = projectPoint(camera.intr, results.p_stat.mean);

  // Take the differential of the camera projection equations to estimate the change in [x,y,z] that is expected from
  // applying camera image noise of [stdev, stdev] u = fx * (x / z) + cx --> dx = du * (z / fx) v = fy * (y / z) + cy
  // --> dy = du * (z / fy) z = fx * x / (u - cx) --> dz = du * -1.0 * (fx * x) / (u - cx)^2
  Eigen::Vector3d delta;
  delta.x() = stdev * results.p_stat.mean.z() / camera.intr.fx();
  delta.y() = stdev * results.p_stat.mean.z() / camera.intr.fy();
  delta.z() = stdev * -1.0 * camera.intr.fx() * results.p_stat.mean.x() / std::pow((uv(0) - camera.intr.cx()), 2.0);

  EXPECT_LT(results.p_stat.stdev.norm(), delta.norm());
  EXPECT_TRUE(results.p_stat.mean.isApprox(expected_pose.translation(), delta.norm()));

  // Expect the mean quaternion to be within 1 sigma of the expected orientation
  EXPECT_LT(results.q_stat.mean.angularDistance(Eigen::Quaterniond(expected_pose.linear())), results.q_stat.stdev);
}

class NoiseQualification3D : public ::testing::Test
{
public:
  NoiseQualification3D()
    : target(test::Target(TARGET_ROWS, TARGET_COLS, SPACING))
    , target_to_camera(Eigen::Isometry3d::Identity())
    , mt_rand(RCT_RANDOM_SEED)
  {
    // Put the camera over the center of the target facing normal to the target
    double x = static_cast<double>(TARGET_ROWS - 1) * SPACING / 2.0;
    double y = static_cast<double>(TARGET_COLS - 1) * SPACING / 2.0;
    target_to_camera.translate(Eigen::Vector3d(x, y, CAMERA_STANDOFF));
    target_to_camera.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  }

  Correspondence3D3D::Set createCorrespondences()
  {
    Correspondence3D3D::Set out;
    EXPECT_NO_THROW(out = test::getCorrespondences(target_to_camera, Eigen::Isometry3d::Identity(), target););

    return out;
  }

  Correspondence3D3D::Set createNoisyCorrespondences(const double mean, const double stdev)
  {
    Correspondence3D3D::Set out = createCorrespondences();
    std::normal_distribution<double> dist(mean, stdev);

    for (auto& corr : out)
    {
      Eigen::Vector3d noise;
      noise << dist(mt_rand), dist(mt_rand), dist(mt_rand);
      corr.in_image += noise;
    }

    return out;
  }

  test::Target target;
  Eigen::Isometry3d target_to_camera;
  std::mt19937 mt_rand;
};

TEST_F(NoiseQualification3D, PerfectData)
{
  // reserve observations
  std::size_t obs_cnt = 35;
  std::vector<PnPProblem3D> ideal_problem_set;
  ideal_problem_set.reserve(obs_cnt);

  // create observations
  Eigen::Isometry3d expected_pose = target_to_camera.inverse();
  for (std::size_t i = 0; i < obs_cnt; ++i)
  {
    PnPProblem3D problem;
    problem.camera_to_target_guess = expected_pose;
    problem.correspondences = createCorrespondences();

    ideal_problem_set.push_back(problem);
  }

  PnPNoiseStat result = qualifyNoise3D(ideal_problem_set);

  EXPECT_TRUE(result.p_stat.mean.isApprox(expected_pose.translation()));
  EXPECT_LT(result.p_stat.stdev.norm(), 1.0e-15);
  EXPECT_LT(result.q_stat.mean.angularDistance(Eigen::Quaterniond(expected_pose.linear())), 1.0e-15);
  EXPECT_LT(result.q_stat.stdev, 1.0e-15);
}

TEST_F(NoiseQualification3D, NoisyData)
{
  // reserve observations
  std::size_t obs_cnt = 35;
  std::vector<PnPProblem3D> ideal_problem_set;
  ideal_problem_set.reserve(obs_cnt);

  double mean = 0.0;
  double stdev = 0.005;

  // create observations
  Eigen::Isometry3d expected_pose = target_to_camera.inverse();
  for (std::size_t i = 0; i < obs_cnt; ++i)
  {
    PnPProblem3D problem;
    problem.camera_to_target_guess = expected_pose;
    problem.correspondences = createNoisyCorrespondences(mean, stdev);

    ideal_problem_set.push_back(problem);
  }

  PnPNoiseStat result = qualifyNoise3D(ideal_problem_set);

  // Create a variable for the magnitude of the standard deviation since it was applied in all three directions
  double stdev_norm = std::sqrt(3) * stdev;

  EXPECT_TRUE(result.p_stat.mean.isApprox(expected_pose.translation(), stdev_norm));
  EXPECT_LT(result.p_stat.stdev.norm(), stdev_norm);
  // Expect that the average quaternion is within one standard deviation of the expected orientation
  EXPECT_LT(result.q_stat.mean.angularDistance(Eigen::Quaterniond(expected_pose.linear())), result.q_stat.stdev);
}

TEST_F(NoiseQualification3D, NoisyDataPerturbedGuess)
{
  // reserve observations
  std::size_t obs_cnt = 35;
  std::vector<PnPProblem3D> ideal_problem_set;
  ideal_problem_set.reserve(obs_cnt);

  double mean = 0.0;
  double stdev = 0.005;

  // create observations
  Eigen::Isometry3d expected_pose = target_to_camera.inverse();
  for (std::size_t i = 0; i < obs_cnt; ++i)
  {
    PnPProblem3D problem;
    problem.camera_to_target_guess = test::perturbPose(expected_pose, 0.01, 0.01);
    problem.correspondences = createNoisyCorrespondences(mean, stdev);

    ideal_problem_set.push_back(problem);
  }

  PnPNoiseStat result = qualifyNoise3D(ideal_problem_set);

  // Create a variable for the magnitude of the standard deviation since it was applied in all three directions
  double stdev_norm = std::sqrt(3) * stdev;

  EXPECT_TRUE(result.p_stat.mean.isApprox(expected_pose.translation(), stdev_norm));
  EXPECT_LT(result.p_stat.stdev.norm(), stdev_norm);
  // Expect that the average quaternion is within one standard deviation of the expected orientation
  EXPECT_LT(result.q_stat.mean.angularDistance(Eigen::Quaterniond(expected_pose.linear())), result.q_stat.stdev);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
