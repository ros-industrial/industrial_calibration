#include <industrial_calibration/optimizations/analysis/statistics.h>
#include <industrial_calibration/optimizations/dh_chain_kinematic_calibration.h>
#include <industrial_calibration/cost_functions/dh_chain_kinematic_calibration.h>
#include <industrial_calibration/optimizations/utils/local_parameterization.h>
#include <industrial_calibration_tests/dh_chain_observation_creator.h>
#include <industrial_calibration_tests/utilities.h>
#include <industrial_calibration_tests/observation_creator.h>

#include <gtest/gtest.h>

using namespace industrial_calibration;

class DHChainMeasurementTest : public ::testing::Test
{
public:
  /**
   * @brief Constructor
   * Note: the calibration problem is initialized with a DH chain with modified values to be
   * more representative of a "real-life" use-case
   */
  DHChainMeasurementTest()
    : camera_chain_truth(test::perturbDHChain(test::createABBIRB2400(), 1.0e-3))
    , target_chain_truth(std::vector<DHTransform>{})
    , problem(camera_chain_truth, target_chain_truth)
    , orientation_weight(100.0)
  {
    // Set a few specific Ceres solver parameters
    options.max_num_iterations = 500;
    options.num_threads = 4;
    options.minimizer_progress_to_stdout = true;
    options.use_nonmonotonic_steps = true;
  }

  void SetUp() override
  {
    setActualData();
    setInitialGuess();
    setObservations();
    applyMasks();
  }

  virtual void setActualData()
  {
    n_observations = 150;

    position_diff_mean_threshold = 0.005;
    position_diff_std_dev_threshold = 0.003;
    orientation_diff_mean_threshold = 0.008;
    orientation_diff_std_dev_threshold = 0.004;

    // Create the transform from the camera mount (i.e. robot wrist) to the camera
    camera_mount_to_camera_truth = Eigen::Isometry3d::Identity();
    camera_mount_to_camera_truth.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));

    // Create the transform from the target mount (in this case an empty kinematic chain) to the target
    target_mount_to_target_truth = Eigen::Isometry3d::Identity();

    // Create the transform from the base of the camera kinematic chain to the base of the target kinematic chain
    camera_base_to_target_base_truth = Eigen::Isometry3d::Identity();
    camera_base_to_target_base_truth.translate(Eigen::Vector3d(0.940, 0.0, 0.0));
  }

  virtual void setInitialGuess()
  {
    // set initial guess to true values
    problem.camera_mount_to_camera_guess = camera_mount_to_camera_truth;
    problem.target_mount_to_target_guess = target_mount_to_target_truth;
    problem.camera_base_to_target_base_guess = camera_base_to_target_base_truth;
  }

  virtual void setObservations()
  {
    // generate observations using true kinematics
    EXPECT_NO_THROW(problem.observations = test::createKinematicMeasurements(
                        camera_chain_truth, target_chain_truth, camera_mount_to_camera_truth,
                        target_mount_to_target_truth, camera_base_to_target_base_truth, n_observations));
  }

  virtual void applyMasks() {}

  virtual void analyzeResults(const KinematicCalibrationResult& result)
  {
    EXPECT_TRUE(result.converged);

    // Expect the resulting DH parameter offsets to be the same shape as the input
    // This is an important check of the case where the cost function modifies for 0 DoF chains
    EXPECT_EQ(problem.camera_chain.dof(), result.camera_chain_dh_offsets.rows());
    EXPECT_EQ(problem.target_chain.dof(), result.target_chain_dh_offsets.rows());

    // Test the result by moving the robot around to a lot of positions and seeing of the results match
    DHChain optimized_chain(problem.camera_chain, result.camera_chain_dh_offsets);

    std::cout << "true chain:\n" << camera_chain_truth.getDHTable().matrix() << std::endl << std::endl;
    std::cout << "optimized chain:\n" << optimized_chain.getDHTable().matrix() << std::endl << std::endl;

    const std::size_t n = 1000;

    std::vector<double> pos_acc, ori_acc;
    pos_acc.reserve(n);
    ori_acc.reserve(n);

    for (std::size_t i = 0; i < n; ++i)
    {
      Eigen::VectorXd random_pose = camera_chain_truth.createUniformlyRandomPose();

      // True chain
      Eigen::Isometry3d true_fk = camera_chain_truth.getFK(random_pose) * camera_mount_to_camera_truth;

      // Optimized chain
      Eigen::Isometry3d optimized_fk = optimized_chain.getFK(random_pose) * result.camera_mount_to_camera;

      // Compare
      Eigen::Isometry3d diff = true_fk.inverse() * optimized_fk;
      pos_acc.push_back(diff.translation().norm());
      ori_acc.push_back(
          Eigen::Quaterniond(true_fk.linear()).angularDistance(Eigen::Quaterniond(optimized_fk.linear())));
    }

    double pos_mean, pos_stdev;
    std::tie(pos_mean, pos_stdev) = computeStats(pos_acc);

    std::cout << "Position Difference Mean: " << pos_mean << std::endl;
    std::cout << "Position Difference Std. Dev.: " << pos_stdev << std::endl;

    double ori_mean, ori_stdev;
    std::tie(ori_mean, ori_stdev) = computeStats(ori_acc);

    std::cout << "Orientation Difference Mean: " << ori_mean << std::endl;
    std::cout << "Orientation difference Std. Dev.: " << ori_stdev << std::endl;

    EXPECT_LT(pos_mean, position_diff_mean_threshold);
    EXPECT_LT(pos_stdev, position_diff_std_dev_threshold);
    EXPECT_LT(ori_mean, orientation_diff_mean_threshold);
    EXPECT_LT(ori_stdev, orientation_diff_std_dev_threshold);
  }

  double position_diff_mean_threshold;
  double position_diff_std_dev_threshold;
  double orientation_diff_mean_threshold;
  double orientation_diff_std_dev_threshold;

  std::size_t n_observations;

  KinematicMeasurement::Set observations;

  const DHChain camera_chain_truth;
  const DHChain target_chain_truth;
  Eigen::Isometry3d camera_mount_to_camera_truth;
  Eigen::Isometry3d target_mount_to_target_truth;
  Eigen::Isometry3d camera_base_to_target_base_truth;

  KinematicCalibrationProblemPose6D problem;
  ceres::Solver::Options options;
  double orientation_weight;
};

/**
 * @brief Tests the Dual DH Chain kinematic calibration algorithm with perfect initial conditions
 */
class DHChainMeasurementTest_PerfectInitial : public DHChainMeasurementTest
{
public:
  using DHChainMeasurementTest::DHChainMeasurementTest;

  virtual void analyzeResults(const KinematicCalibrationResult& result) override
  {
    // Check the nominal expectations
    DHChainMeasurementTest::analyzeResults(result);

    EXPECT_LT(result.initial_cost_per_obs, 1.0e-15);

    EXPECT_TRUE(result.camera_mount_to_camera.isApprox(camera_mount_to_camera_truth));
    EXPECT_TRUE(result.target_mount_to_target.isApprox(target_mount_to_target_truth));
    EXPECT_TRUE(result.camera_base_to_target_base.isApprox(camera_base_to_target_base_truth));

    // Expect the resulting DH parameter offsets to be the same shape as the input
    // This is an important check of the case where the cost function modifies for 0 DoF chains
    EXPECT_EQ(problem.camera_chain.dof(), result.camera_chain_dh_offsets.rows());
    EXPECT_EQ(problem.target_chain.dof(), result.target_chain_dh_offsets.rows());

    // Expect the camera DH offsets to remain the same as the input (i.e. all zeros)
    for (Eigen::Index row = 0; row < result.camera_chain_dh_offsets.rows(); ++row)
    {
      for (Eigen::Index col = 0; col < result.camera_chain_dh_offsets.cols(); ++col)
      {
        EXPECT_LT(result.camera_chain_dh_offsets(row, col), 1.0e-15);
      }
    }
  }
};

/**
 * @brief Tests the Dual DH Chain kinematic calibration algorithm with
 * initial guesses for DH params that are slightly different from the true values
 */
class DHChainMeasurementTest_PerturbedDH : public DHChainMeasurementTest
{
public:
  using DHChainMeasurementTest::DHChainMeasurementTest;

  virtual void setInitialGuess() override
  {
    DHChainMeasurementTest::setInitialGuess();

    problem.camera_chain = test::createABBIRB2400();
  }

  virtual void applyMasks() override
  {
    // Mask a few DH parameters
    {
      Eigen::Matrix<bool, Eigen::Dynamic, 4> mask =
          Eigen::Matrix<bool, Eigen::Dynamic, 4>::Constant(problem.camera_chain.dof(), 4, false);

      // Mask the last row because they duplicate the camera mount to camera transform
      mask.bottomRows(1) << true, true, true, true;

      // Mask the joint 2 "r" parameter
      // It seems to have a high correlation with target_mount_to_target_y
      mask(1, 2) = true;

      // Add the mask to the problem
      problem.mask.at(0) = createDHMask(mask);
    }

    /* Mask the camera base to target base transform (duplicated by target mount to target transform when
     * the target chain has no joints */
    problem.mask.at(6) = { 0, 1, 2 };
    problem.mask.at(7) = { 0, 1, 2 };
  }

  virtual void analyzeResults(const KinematicCalibrationResult& result) override
  {
    DHChainMeasurementTest::analyzeResults(result);

    // Expect the masked variables not to have changed
    EXPECT_TRUE(result.camera_chain_dh_offsets.bottomRows(1).isApproxToConstant(0.0));
    EXPECT_DOUBLE_EQ(result.camera_chain_dh_offsets(1, 2), 0.0);
    EXPECT_TRUE(result.camera_base_to_target_base.isApprox(problem.camera_base_to_target_base_guess));
  }
};

/**
 * @brief Tests the Dual DH Chain kinematic calibration algorithm with
 * initial guesses for DH params and transforms that are slightly different from the true values
 */
class DHChainMeasurementTest_PerturbedDH_PertubedGuess : public DHChainMeasurementTest_PerturbedDH
{
public:
  using DHChainMeasurementTest_PerturbedDH::DHChainMeasurementTest_PerturbedDH;

  virtual void setInitialGuess() override
  {
    DHChainMeasurementTest_PerturbedDH::setInitialGuess();

    problem.camera_mount_to_camera_guess = test::perturbPose(camera_mount_to_camera_truth, 0.05, 0.05);
    problem.target_mount_to_target_guess = test::perturbPose(target_mount_to_target_truth, 0.05, 0.05);
    problem.camera_base_to_target_base_guess = test::perturbPose(camera_base_to_target_base_truth, 0.05, 0.05);
  }
};

TEST_F(DHChainMeasurementTest, TestCostFunction)
{
  // Initialize the optimization variables
  // Camera mount to camera (cm_to_c) quaternion and translation
  Eigen::Vector3d t_cm_to_c(camera_mount_to_camera_truth.translation());
  Eigen::AngleAxisd rot_cm_to_c(camera_mount_to_camera_truth.rotation());
  Eigen::Vector3d aa_cm_to_c(rot_cm_to_c.angle() * rot_cm_to_c.axis());

  // Target mount to target (cm_to_c) quaternion and translation
  Eigen::Vector3d t_tm_to_t(target_mount_to_target_truth.translation());
  Eigen::AngleAxisd rot_tm_to_t(target_mount_to_target_truth.rotation());
  Eigen::Vector3d aa_tm_to_t(rot_tm_to_t.angle() * rot_tm_to_t.axis());

  // Camera chain base to target_chain_base (ccb_to_tcb) quaternion and translation
  Eigen::Vector3d t_ccb_to_tcb(camera_base_to_target_base_truth.translation());
  Eigen::AngleAxisd rot_ccb_to_tcb(camera_base_to_target_base_truth.rotation());
  Eigen::Vector3d aa_ccb_to_tcb(rot_ccb_to_tcb.angle() * rot_ccb_to_tcb.axis());

  // Create containers for the kinematic chain DH offsets
  // Ceres will not work with parameter blocks of size zero, so create a dummy set of DH offsets for chains with DoF ==
  // 0
  Eigen::MatrixX4d camera_chain_dh_offsets = Eigen::MatrixX4d::Zero(camera_chain_truth.dof(), 4);
  Eigen::MatrixX4d target_chain_dh_offsets = Eigen::MatrixX4d::Zero(target_chain_truth.dof(), 4);

  // Create a vector of the pointers to the optimization variables in the order that the cost function expects them
  std::vector<double*> parameters =
      DualDHChainMeasurementCost::constructParameters(camera_chain_dh_offsets, target_chain_dh_offsets, t_cm_to_c,
                                                      aa_cm_to_c, t_tm_to_t, aa_tm_to_t, t_ccb_to_tcb, aa_ccb_to_tcb);

  // Create observations
  KinematicMeasurement::Set observations =
      test::createKinematicMeasurements(camera_chain_truth, target_chain_truth, camera_mount_to_camera_truth,
                                        target_mount_to_target_truth, camera_base_to_target_base_truth, n_observations);

  // Test the cost function
  for (const auto& obs : observations)
  {
    DualDHChainMeasurementCost cost(obs, camera_chain_truth, target_chain_truth, orientation_weight);

    double residual[4];
    EXPECT_TRUE(cost(parameters.data(), residual));

    EXPECT_NEAR(residual[0], 0, 1.0e-12);  // x
    EXPECT_NEAR(residual[1], 0, 1.0e-12);  // y
    EXPECT_NEAR(residual[2], 0, 1.0e-12);  // z

    EXPECT_NEAR(residual[3], 0, 1.0e-12);  // rotation
  }
}

TEST_F(DHChainMeasurementTest_PerfectInitial, PerfectInitialConditions)
{
  KinematicCalibrationResult result = optimize(problem, orientation_weight, options);
  std::cout << result.covariance.printCorrelationCoeffAboveThreshold(0.5) << std::endl;
  analyzeResults(result);
}

TEST_F(DHChainMeasurementTest_PerturbedDH, PerfectGuessPerturbedDH)
{
  KinematicCalibrationResult result = optimize(problem, orientation_weight, options);
  std::cout << result.covariance.printCorrelationCoeffAboveThreshold(0.5) << std::endl;
  analyzeResults(result);
}

TEST_F(DHChainMeasurementTest_PerturbedDH_PertubedGuess, PerturbedDHPerturbedGuess)
{
  KinematicCalibrationResult result = optimize(problem, orientation_weight, options);
  std::cout << result.covariance.printCorrelationCoeffAboveThreshold(0.5) << std::endl;
  analyzeResults(result);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
