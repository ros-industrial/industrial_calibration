#include <gtest/gtest.h>
#include <industrial_calibration/optimizations/extrinsic_multi_static_camera.h>
#include <industrial_calibration/optimizations/ceres_math_utilities.h>
#include <industrial_calibration_tests/utilities.h>
#include <industrial_calibration_tests/observation_creator.h>

using namespace industrial_calibration;

void printResults(const ExtrinsicMultiStaticCameraMovingTargetResult& opt_result)
{
  // Report results
  std::cout << "Did converge?: " << opt_result.converged << "\n";
  std::cout << "Initial cost?: " << opt_result.initial_cost_per_obs << "\n";
  std::cout << "Final cost?: " << opt_result.final_cost_per_obs << "\n";

  Eigen::Isometry3d t = opt_result.wrist_to_target;

  std::cout << "Wrist to Target:\n";
  std::cout << t.matrix() << "\n";

  for (std::size_t c = 0; c < opt_result.base_to_camera.size(); ++c)
  {
    // Load the data set path from ROS param
    std::string param_base = "camera_" + std::to_string(c);
    Eigen::Isometry3d t = opt_result.base_to_camera[c];

    std::cout << "Base to Camera (" + param_base + "):\n";
    std::cout << t.matrix() << "\n";

    std::cout << "--- URDF Format Base to Camera (" + param_base + ") ---\n";
    Eigen::Vector3d rpy = t.rotation().eulerAngles(2, 1, 0);
    std::cout << "xyz=\"" << t.translation()(0) << " " << t.translation()(1) << " " << t.translation()(2) << "\"\n";
    std::cout << "rpy=\"" << rpy(2) << "(" << rpy(2) * 180 / M_PI << " deg) " << rpy(1) << "(" << rpy(1) * 180 / M_PI
              << " deg) " << rpy(0) << "(" << rpy(0) * 180 / M_PI << " deg)\"\n";
  }

  std::cout << opt_result.covariance.toString() << std::endl;
}

struct Observations
{
  std::vector<Eigen::Isometry3d> wrist_poses;
  std::vector<Correspondence2D3D::Set> correspondences;
};

Observations addObservations(const test::Target& target, const test::Camera& camera,
                             const Eigen::Isometry3d& base_to_camera, const Eigen::Isometry3d& wrist_to_target,
                             const Eigen::Vector3d& axis)
{
  const int n = 20;

  Observations obs;
  obs.wrist_poses.reserve(n + 1);
  obs.correspondences.reserve(n + 1);

  // Add a grid of robot poses and associated target observations
  for (int i = -(n / 2); i <= (n / 2); ++i)
  {
    Eigen::Isometry3d link_1 = Eigen::Isometry3d::Identity();
    link_1.translation() = Eigen::Vector3d(0, 0, 0.25);

    Eigen::Isometry3d link_2(Eigen::AngleAxisd(i * M_PI / 180.0, axis));
    link_2.translation() = Eigen::Vector3d(0, 0, 0.25);

    Eigen::Isometry3d base_to_wrist = link_1 * link_2;
    Eigen::Isometry3d base_to_target = base_to_wrist * wrist_to_target;

    // Get visible observations
    Correspondence2D3D::Set obs_set = test::getCorrespondences(base_to_camera, base_to_target, camera, target, false);

    if (obs_set.size() > 0)
    {
      obs.wrist_poses.push_back(base_to_wrist);
      obs.correspondences.push_back(obs_set);
    }
  }

  return obs;
}

TEST(ExtrinsicMultiStaticCamera, single_camera)
{
  ExtrinsicMultiStaticCameraMovingTargetProblem problem_def;

  // Create a target
  test::Target target(5, 5, 0.015);

  // Create the base to camera transform
  std::vector<Eigen::Isometry3d> base_to_camera;
  base_to_camera.push_back(Eigen::Isometry3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())));
  base_to_camera[0].translation() = Eigen::Vector3d(0, 0, 2.0);

  // Create the wrist to target transform
  Eigen::Isometry3d wrist_to_target = Eigen::Isometry3d::Identity();
  wrist_to_target.translation() = Eigen::Vector3d(0, 0, 0.25);

  // Set up the problem
  problem_def.intr.resize(1);
  problem_def.intr.at(0).fx() = 1411.0;
  problem_def.intr.at(0).fy() = 1408.0;
  problem_def.intr.at(0).cx() = 807.2;
  problem_def.intr.at(0).cy() = 615.0;

  problem_def.base_to_camera_guess.resize(problem_def.intr.size());
  problem_def.base_to_camera_guess[0] = Eigen::Isometry3d(Eigen::AngleAxisd(M_PI + 0.01, Eigen::Vector3d::UnitX()));
  problem_def.base_to_camera_guess[0].translation() = Eigen::Vector3d(0.01, 0.001, 1.99);

  problem_def.wrist_to_target_guess = Eigen::Isometry3d::Identity();
  problem_def.wrist_to_target_guess.translation() = Eigen::Vector3d(0.001, 0.001, 0.26);

  // Add the observations
  problem_def.wrist_poses.resize(problem_def.intr.size());
  problem_def.image_observations.resize(problem_def.intr.size());

  // Set up variables for observation generation
  std::vector<Eigen::Vector3d> axes = { Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitX() };

  // Generate observations for each camera
  for (std::size_t i = 0; i < problem_def.intr.size(); ++i)
  {
    // Create the camera
    test::Camera camera;
    camera.intr = problem_def.intr.at(i);
    camera.width = 1600;
    camera.height = 1200;

    // Generate various data by rotating about different axes
    for (const auto& axis : axes)
    {
      auto tmp = addObservations(target, camera, base_to_camera.at(i), wrist_to_target, axis);

      // Add the wrist poses to the problem
      auto& poses = problem_def.wrist_poses.at(i);
      poses.insert(poses.end(), tmp.wrist_poses.begin(), tmp.wrist_poses.end());

      // Add the image_observations to the problem
      auto& obs = problem_def.image_observations.at(i);
      obs.insert(obs.end(), tmp.correspondences.begin(), tmp.correspondences.end());
    }
  }

  // Run optimization
  ExtrinsicMultiStaticCameraMovingTargetResult opt_result = optimize(problem_def);

  EXPECT_TRUE(opt_result.converged);
  EXPECT_TRUE(opt_result.final_cost_per_obs < 1e-15);
  EXPECT_TRUE(opt_result.wrist_to_target.isApprox(wrist_to_target, 1e-8));
  for (std::size_t c = 0; c < opt_result.base_to_camera.size(); ++c)
  {
    EXPECT_TRUE(opt_result.base_to_camera[c].isApprox(base_to_camera[c], 1e-8));
  }

  EXPECT_EQ(opt_result.covariance.covariance_matrix.rows(), 12);
  EXPECT_EQ(opt_result.covariance.covariance_matrix.cols(), 12);
  EXPECT_EQ(opt_result.covariance.correlation_matrix.rows(), 12);
  EXPECT_EQ(opt_result.covariance.correlation_matrix.cols(), 12);
  EXPECT_EQ(opt_result.covariance.standard_deviations.size(), 12);
  EXPECT_EQ(opt_result.covariance.covariances.size(), 66);
  EXPECT_EQ(opt_result.covariance.correlation_coeffs.size(), 66);

  printResults(opt_result);
}

TEST(ExtrinsicMultiStaticCamera, two_cameras)
{
  ExtrinsicMultiStaticCameraMovingTargetProblem problem_def;

  // Create a target
  test::Target target(5, 5, 0.015);

  // Create the wrist to target transform
  Eigen::Isometry3d wrist_to_target = Eigen::Isometry3d::Identity();
  wrist_to_target.translation() = Eigen::Vector3d(0, 0, 0.25);

  // Create the base to camera transforms
  std::vector<Eigen::Isometry3d> base_to_camera;
  base_to_camera.resize(2);
  base_to_camera[0] = Eigen::Isometry3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  base_to_camera[0].translation() = Eigen::Vector3d(-0.1, 0, 2.0);
  base_to_camera[1] = Eigen::Isometry3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  base_to_camera[1].translation() = Eigen::Vector3d(0.1, 0, 2.0);

  // Set up the problem
  problem_def.intr.resize(2);
  problem_def.intr.at(0).fx() = 1411.0;
  problem_def.intr.at(0).fy() = 1408.0;
  problem_def.intr.at(0).cx() = 807.2;
  problem_def.intr.at(0).cy() = 615.0;
  problem_def.intr.at(1) = problem_def.intr.at(0);

  problem_def.base_to_camera_guess.resize(problem_def.intr.size());
  problem_def.base_to_camera_guess[0] = Eigen::Isometry3d(Eigen::AngleAxisd(M_PI + 0.01, Eigen::Vector3d::UnitX()));
  problem_def.base_to_camera_guess[0].translation() = Eigen::Vector3d(-0.101, 0.001, 1.99);
  problem_def.base_to_camera_guess[1] = Eigen::Isometry3d(Eigen::AngleAxisd(M_PI + 0.02, Eigen::Vector3d::UnitX()));
  problem_def.base_to_camera_guess[1].translation() = Eigen::Vector3d(0.101, 0.001, 1.99);

  problem_def.wrist_to_target_guess = Eigen::Isometry3d::Identity();
  problem_def.wrist_to_target_guess.translation() = Eigen::Vector3d(0.001, 0.001, 0.26);
  problem_def.intr[1] = problem_def.intr[0];

  // Add the observations
  problem_def.wrist_poses.resize(problem_def.intr.size());
  problem_def.image_observations.resize(problem_def.intr.size());

  // Set up variables for observation generation
  std::vector<Eigen::Vector3d> axes = { Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitX() };

  // Generate observations for each camera
  for (std::size_t i = 0; i < problem_def.intr.size(); ++i)
  {
    // Create the camera
    test::Camera camera;
    camera.intr = problem_def.intr.at(i);
    camera.width = 1600;
    camera.height = 1200;

    // Generate various data by rotating about different axes
    for (const auto& axis : axes)
    {
      auto tmp = addObservations(target, camera, base_to_camera.at(i), wrist_to_target, axis);

      // Add the wrist poses to the problem
      auto& poses = problem_def.wrist_poses.at(i);
      poses.insert(poses.end(), tmp.wrist_poses.begin(), tmp.wrist_poses.end());

      // Add the image_observations to the problem
      auto& obs = problem_def.image_observations.at(i);
      obs.insert(obs.end(), tmp.correspondences.begin(), tmp.correspondences.end());
    }
  }

  // Run optimization
  ExtrinsicMultiStaticCameraMovingTargetResult opt_result = optimize(problem_def);

  EXPECT_TRUE(opt_result.converged);
  EXPECT_LT(opt_result.final_cost_per_obs, 1e-15);
  EXPECT_TRUE(opt_result.wrist_to_target.isApprox(wrist_to_target, 1e-8));
  for (std::size_t c = 0; c < opt_result.base_to_camera.size(); ++c)
  {
    EXPECT_TRUE(opt_result.base_to_camera[c].isApprox(base_to_camera[c], 1e-8));
  }

  EXPECT_EQ(opt_result.covariance.covariance_matrix.rows(), 18);
  EXPECT_EQ(opt_result.covariance.covariance_matrix.cols(), 18);
  EXPECT_EQ(opt_result.covariance.correlation_matrix.rows(), 18);
  EXPECT_EQ(opt_result.covariance.correlation_matrix.cols(), 18);
  EXPECT_EQ(opt_result.covariance.standard_deviations.size(), 18);
  EXPECT_EQ(opt_result.covariance.covariances.size(), 153);
  EXPECT_EQ(opt_result.covariance.correlation_coeffs.size(), 153);

  printResults(opt_result);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
