#include <industrial_calibration/exceptions.h>
#include <industrial_calibration/optimizations/analysis/statistics.h>
#include <industrial_calibration/optimizations/dh_chain_kinematic_calibration.h>

#if __GNUC__ >= 8
#include <filesystem>
using path = std::filesystem::path;
#else
#include <experimental/filesystem>
using path = std::experimental::filesystem::path;
#endif

#include <random>
#include <yaml-cpp/yaml.h>

using namespace industrial_calibration;

/** This example performs a kinematic calibration for a two-axis part positioner using pose measurements acquired from
 * a statically mounted metrology sensor observing a target mounted on the positioner. Each observation consists of a
 * 6-DOF pose measurement and the joint state of the positioner. The calibration problem is formulated with two DH
 * chains connected by a fixed transform. The target is mounted to the end of one DH chain (in this case the two-axis
 * positioner), and the camera is mounted to the end of the other DH chain (in this case an empty chain). The goal of
 * the kinematic calibration is to estimate the DH parameters of both chains (in this case just the 8 DH parameters of
 * the two-axis positioner), the pose from the target mount frame (i.e. tip of the positioner kinematic chain) to the
 * target frame, the pose from the camera mount frame (i.e. world frame) to the camera, and the pose from the camera DH
 * chain base to the target DH chain base (i.e. world frame to positioner base frame)
 */

KinematicMeasurement::Set loadMeasurements(const std::string& filename)
{
  KinematicMeasurement::Set measurements;

  try
  {
    YAML::Node n = YAML::LoadFile(filename);
    measurements.reserve(n.size());

    for (auto it = n.begin(); it != n.end(); ++it)
    {
      KinematicMeasurement measurement;

      // Target chain joints
      {
        YAML::Node joints = it->second["target_joints"];
        measurement.target_chain_joints.resize(joints.size());
        for (std::size_t i = 0; i < joints.size(); ++i)
        {
          measurement.target_chain_joints[i] = joints[i].as<double>();
        }
      }

      // Camera chain joints
      {
        YAML::Node joints = it->second["camera_joints"];
        measurement.camera_chain_joints.resize(joints.size());
        for (std::size_t i = 0; i < joints.size(); ++i)
        {
          measurement.camera_chain_joints[i] = joints[i].as<double>();
        }
      }

      // Camera to target pose
      {
        YAML::Node pose = it->second["pose"];
        double x = pose["x"].as<double>();
        double y = pose["y"].as<double>();
        double z = pose["z"].as<double>();
        double qw = pose["qw"].as<double>();
        double qx = pose["qx"].as<double>();
        double qy = pose["qy"].as<double>();
        double qz = pose["qz"].as<double>();

        measurement.camera_to_target = Eigen::Isometry3d::Identity();
        measurement.camera_to_target.translate(Eigen::Vector3d(x, y, z));
        measurement.camera_to_target.rotate(Eigen::Quaterniond(qw, qx, qy, qz));

        // Add the measurement to the set
        measurements.push_back(measurement);
      }
    }
  }
  catch (YAML::Exception& ex)
  {
    throw BadFileException(std::string("YAML failure: ") + ex.what());
  }

  return measurements;
}

/**
 * @brief Creates a DH chain representing the nominally known kinematics of a two-axis positioner
 */
DHChain createTwoAxisPositioner()
{
  std::vector<DHTransform> transforms;
  transforms.reserve(2);

  Eigen::Vector4d p1, p2;
  p1 << 0.0, 0.0, 0.0, -M_PI / 2.0;
  p2 << -0.475, -M_PI / 2.0, 0.0, 0.0;

  // Add the first DH transform
  {
    DHTransform t(p1, DHJointType::REVOLUTE, "j1");
    t.max = M_PI;
    t.min = -M_PI;
    transforms.push_back(t);
  }
  // Add the second DH transform
  {
    DHTransform dh_transform(p2, DHJointType::REVOLUTE, "j2");
    dh_transform.max = 2.0 * M_PI;
    dh_transform.min = -2.0 * M_PI;
    transforms.push_back(dh_transform);
  }

  // Set an arbitrary base offset
  Eigen::Isometry3d base_offset(Eigen::Isometry3d::Identity());
  base_offset.translate(Eigen::Vector3d(2.2, 0.0, 1.6));
  base_offset.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()));

  return DHChain(transforms, base_offset);
}

void printResults(const KinematicCalibrationResult& result)
{
  Eigen::IOFormat fmt(4, 0, "|", "\n", "|", "|");

  std::stringstream ss;
  ss << "\nCalibration " << (result.converged ? "did" : "did not") << " converge\n";
  ss << "Initial cost per observation: " << std::sqrt(result.initial_cost_per_obs) << "\n";
  ss << "Final cost per observation: " << std::sqrt(result.final_cost_per_obs) << "\n";

  ss << "\nCamera mount to camera\n" << result.camera_mount_to_camera.matrix().format(fmt) << "\n";
  ss << "Euler ZYX: " << result.camera_mount_to_camera.rotation().eulerAngles(2, 1, 0).transpose().format(fmt) << "\n";

  ss << "\nTarget mount to target\n" << result.target_mount_to_target.matrix().format(fmt) << "\n";
  ss << "Euler ZYX: " << result.target_mount_to_target.rotation().eulerAngles(2, 1, 0).transpose().format(fmt) << "\n";

  ss << "\nDH parameter offsets\n" << result.target_chain_dh_offsets.matrix().format(fmt) << "\n";
  ss << result.covariance.printCorrelationCoeffAboveThreshold(0.5);

  std::cout << ss.str() << std::endl;
}

struct Stats
{
  double pos_mean;
  double pos_stdev;
  double rot_mean;
  double rot_stdev;

  void print() const
  {
    std::cout << "Position Difference Mean: " << pos_mean << std::endl;
    std::cout << "Position Difference Std. Dev.: " << pos_stdev << std::endl;
    std::cout << "Orientation Difference Mean: " << rot_mean << std::endl;
    std::cout << "Orientation difference Std. Dev.: " << rot_stdev << "\n" << std::endl;
  }

  /**
   * @brief Operator for returing the percentage difference between the values in this class and another instance
   */
  Stats operator%(const Stats& other) const
  {
    Stats out;
    out.pos_mean = (pos_mean - other.pos_mean) / pos_mean;
    out.pos_stdev = (pos_stdev - other.pos_stdev) / pos_stdev;
    out.rot_mean = (rot_mean - other.rot_mean) / rot_mean;
    out.rot_stdev = (rot_stdev - other.rot_stdev) / rot_stdev;
    return out;
  }
};

Stats compareToMeasurements(const DHChain& initial_camera_chain, const DHChain& initial_target_chain,
                            const KinematicCalibrationResult& result, const KinematicMeasurement::Set& measurements)
{
  // Test the result by moving the robot around to a lot of positions and seeing if the results match
  DHChain camera_chain(initial_camera_chain, result.camera_chain_dh_offsets);
  DHChain target_chain(initial_target_chain, result.target_chain_dh_offsets);

  std::vector<double> pos_acc, ori_acc;
  pos_acc.reserve(measurements.size());
  ori_acc.reserve(measurements.size());

  for (const KinematicMeasurement& m : measurements)
  {
    // Build the transforms from the camera chain base out to the camera
    Eigen::Isometry3d camera_chain_fk = camera_chain.getFK<double>(m.camera_chain_joints);
    Eigen::Isometry3d camera_base_to_camera = camera_chain_fk * result.camera_mount_to_camera;

    // Build the transforms from the camera chain base out to the target
    Eigen::Isometry3d target_chain_fk = target_chain.getFK<double>(m.target_chain_joints);
    Eigen::Isometry3d camera_base_to_target =
        result.camera_base_to_target_base * target_chain_fk * result.target_mount_to_target;

    // Now that we have two transforms in the same frame, get the target point in the camera frame
    Eigen::Isometry3d camera_to_target = camera_base_to_camera.inverse() * camera_base_to_target;

    // Compare
    Eigen::Isometry3d diff = camera_to_target.inverse() * m.camera_to_target;
    pos_acc.push_back(diff.translation().norm());
    ori_acc.push_back(
        Eigen::Quaterniond(camera_to_target.linear()).angularDistance(Eigen::Quaterniond(m.camera_to_target.linear())));
  }

  Stats stats;
  std::tie(stats.pos_mean, stats.pos_stdev) = computeStats(pos_acc);
  std::tie(stats.rot_mean, stats.rot_stdev) = computeStats(ori_acc);

  return stats;
}

/**
 * @brief Performs the kinematic calibration of the two-axis positioner
 * @return a tuple containing the results of the kinematic calibration, stats from the validation of the calibration in
 * which the DH chain values were optimized, and stats from the validation of the calibration in which the DH chain
 * values were NOT optimized
 */
std::tuple<KinematicCalibrationResult, Stats, Stats> run()
{
  const path data_dir = path(EXAMPLE_DATA_DIR) / path("kinematic_calibration");

  // Load the observations
  KinematicMeasurement::Set measurements = loadMeasurements((data_dir / "measurements.yaml").string());

  // Create the problem
  KinematicCalibrationProblemPose6D problem(DHChain(std::vector<DHTransform>{}), createTwoAxisPositioner());

  // Add the observations
  problem.observations = measurements;

  // Set the initial transform guesses
  YAML::Node pose_guesses = YAML::LoadFile((data_dir / "pose_initial_guesses.yaml").string());
  problem.camera_mount_to_camera_guess = pose_guesses["camera_mount_to_camera_guess"].as<Eigen::Isometry3d>();
  problem.target_mount_to_target_guess = pose_guesses["target_mount_to_target_guess"].as<Eigen::Isometry3d>();
  problem.camera_base_to_target_base_guess = Eigen::Isometry3d::Identity();

  // Set the DH chain offset standard deviation expectations
  problem.camera_chain_offset_stdev = 0.001;
  problem.target_chain_offset_stdev = 0.005;

  // Mask a few DH parameters in the target chain (index 1)
  {
    Eigen::Matrix<bool, Eigen::Dynamic, 4> mask =
        Eigen::Matrix<bool, Eigen::Dynamic, 4>::Constant(problem.target_chain.dof(), 4, false);

    // Mask the last row because they duplicate the target mount to target transform
    mask.bottomRows(1) << true, true, true, true;

    // Add the mask to the problem
    problem.mask.at(1) = createDHMask(mask);
  }

  /* Mask the camera base to target base transform (duplicated by target mount to target transform when
   * the target chain has no joints */
  problem.mask.at(6) = { 0, 1, 2 };
  problem.mask.at(7) = { 0, 1, 2 };

  // Set up the Ceres optimization parameters
  ceres::Solver::Options options;
  options.max_num_iterations = 500;
  options.num_threads = 4;
  options.minimizer_progress_to_stdout = true;
  options.use_nonmonotonic_steps = true;

  // Run the calibration
  std::cout << "Starting kinematic calibration optimization..." << std::endl;

  KinematicCalibrationResult result_opt_dh = optimize(problem, 100.0, options);
  printResults(result_opt_dh);

  Stats stats_opt_dh = compareToMeasurements(problem.camera_chain, problem.target_chain, result_opt_dh, measurements);
  std::cout << "DH calibration validation:" << std::endl;
  stats_opt_dh.print();

  // Re-run the calibration without optimizing the chain parameters
  // Mask the entire target chain
  {
    Eigen::Matrix<bool, Eigen::Dynamic, 4> mask =
        Eigen::Matrix<bool, Eigen::Dynamic, 4>::Constant(problem.target_chain.dof(), 4, true);

    problem.mask.at(1) = createDHMask(mask);
  }

  KinematicCalibrationResult result_static_dh = optimize(problem, 100.0, options);
  printResults(result_static_dh);

  // Compare the results of this optimization with the measurements using the measured joints and nominal kinematic
  // chain
  Stats stats_static_dh =
      compareToMeasurements(problem.camera_chain, problem.target_chain, result_static_dh, measurements);
  std::cout << "Calibration validation - static DH parameters:" << std::endl;
  stats_static_dh.print();

  return std::make_tuple(result_opt_dh, stats_opt_dh, stats_static_dh);
}

#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING

int main(int argc, char** argv)
{
  try
  {
    KinematicCalibrationResult result;
    Stats stats_opt_dh, stats_static_dh;
    std::tie(result, stats_opt_dh, stats_static_dh) = run();

    // Print the percentage difference between the calibrations
    Stats diff_stats = stats_static_dh % stats_opt_dh;
    std::cout << "Percent improvement: calibration vs. nominal kinematic model" << std::endl;
    std::cout << "Position: " << 100.0 * diff_stats.pos_mean << "%" << std::endl;
    std::cout << "Position Std. Dev.: " << 100.0 * diff_stats.pos_stdev << "%" << std::endl;
    std::cout << "Orientation: " << 100.0 * diff_stats.rot_mean << "%" << std::endl;
    std::cout << "Orientation Std. Dev.: " << 100.0 * diff_stats.rot_stdev << "%"
              << "\n"
              << std::endl;

    return 0;
  }
  catch (const std::exception& ex)
  {
    std::cerr << ex.what() << std::endl;
    return -1;
  }
}

#else

#include <gtest/gtest.h>

TEST(KinematicCalibration, TwoAxisPositionerCalibrationFromMeasurements)
{
  KinematicCalibrationResult result;
  Stats stats_opt_dh, stats_static_dh;
  ASSERT_NO_THROW(std::tie(result, stats_opt_dh, stats_static_dh) = run());

  // Expect the optimization to have converged with low residual error
  // Note: error units are mixed position and orientation, so the intuitive value of the residual error is unclear
  ASSERT_TRUE(result.converged);
  ASSERT_LT(std::sqrt(result.final_cost_per_obs), 0.15);  // mixed units

  // Expect that the average positional and rotational errors of the post-calibration pose estimates compared to the
  // actual measurements are low
  ASSERT_LT(stats_opt_dh.pos_mean, 0.002);               // meters
  ASSERT_LT(stats_opt_dh.rot_mean, 0.2 * M_PI / 180.0);  // radians

  // Expect the optimization in which the DH parameters were adjusted to be some percentage improvement over the
  // optimization in which the DH parameters were not adjusted
  Stats diff_stats = stats_static_dh % stats_opt_dh;
  ASSERT_GT(diff_stats.pos_mean, 0.1);
  ASSERT_GT(diff_stats.pos_stdev, 0.1);
  ASSERT_GT(diff_stats.rot_mean, 0.1);
  ASSERT_GT(diff_stats.rot_stdev, 0.1);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#endif
