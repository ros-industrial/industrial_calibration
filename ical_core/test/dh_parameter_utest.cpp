#include <ical_core_tests/dh_chain_observation_creator.h>
#include <random>
#include <gtest/gtest.h>

using namespace industrial_calibration;

TEST(DHChain, FKTestABBIRB2400)
{
  DHChain robot = test::createABBIRB2400();
  Eigen::Isometry3d transform = robot.getFK<double>(Eigen::VectorXd::Zero(robot.dof()));
  std::cout << transform.matrix() << std::endl;

  Eigen::Isometry3d desired(Eigen::Isometry3d::Identity());
  desired.translate(Eigen::Vector3d(0.940, 0.0, 1.455));
  desired.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));
  std::cout << desired.matrix() << std::endl;

  EXPECT_TRUE(transform.isApprox(desired));
}

TEST(DHChain, FKTestPositioner)
{
  DHChain robot = test::createTwoAxisPositioner();
  Eigen::Isometry3d transform = robot.getFK<double>(Eigen::VectorXd::Zero(robot.dof()));

  Eigen::Isometry3d desired = Eigen::Isometry3d::Identity();
  desired.translate(Eigen::Vector3d(2.2, 0.0, 1.125));
  desired.rotate(Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ()));

  EXPECT_TRUE(transform.isApprox(desired));
}

TEST(DHChain, NoisyFKTest)
{
  DHChain robot = test::createABBIRB2400();

  // Create random gaussian-distributed DH offsets
  std::mt19937 mt_rand(RCT_RANDOM_SEED);
  std::uniform_real_distribution<double> dist(-0.01, 0.01);

  Eigen::MatrixX4d dh_offsets = Eigen::MatrixX4d(robot.dof(), 4).unaryExpr([&](float) { return dist(mt_rand); });

  Eigen::Isometry3d transform = robot.getFK<double>(Eigen::VectorXd::Zero(robot.dof()), dh_offsets);
  std::cout << "FK transform with noise\n" << transform.matrix() << std::endl;

  Eigen::Isometry3d desired(Eigen::Isometry3d::Identity());
  desired.translate(Eigen::Vector3d(0.940, 0.0, 1.455));
  desired.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));
  std::cout << "Desired FK transform \n" << desired.matrix() << std::endl;

  EXPECT_FALSE(transform.isApprox(desired));
}

TEST(DHChain, FKWithJointSubsetTest)
{
  DHChain robot = test::createABBIRB2400();
  for (std::size_t i = 0; i < robot.dof(); ++i)
  {
    EXPECT_NO_THROW(robot.getFK<double>(Eigen::VectorXd::Zero(i)));
  }
  EXPECT_THROW(robot.getFK<double>(Eigen::VectorXd::Zero(robot.dof() + 1)), std::runtime_error);
}

TEST(DHChain, generateObservations3D)
{
  const std::size_t n = 100;
  auto observations = test::createObservations(test::createABBIRB2400(), DHChain({}), Eigen::Isometry3d::Identity(),
                                               Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(),
                                               test::Target(5, 5, 0.025), n);
  EXPECT_EQ(observations.size(), n);
}

class DHChainObservationGeneration : public ::testing::Test
{
public:
  DHChainObservationGeneration() : camera_chain(test::createABBIRB2400()), target_chain({}), target(7, 5, 0.025)
  {
    // Create a transform to the tool0 of the robot at it's all-zero position
    camera_base_to_target_base = camera_chain.getFK<double>(Eigen::VectorXd::Zero(camera_chain.dof()));

    // Place the target 0.5m in front of the robot's all-zero position, facing back at the robot
    camera_base_to_target_base.translate(Eigen::Vector3d(0.0, 0.0, 0.5));
    camera_base_to_target_base.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
  }

protected:
  DHChain camera_chain;
  DHChain target_chain;
  Eigen::Isometry3d camera_base_to_target_base;
  const std::size_t n = 100;
  test::Target target;
};

TEST_F(DHChainObservationGeneration, generateObservations2D)
{
  const test::Camera camera = test::makeKinectCamera();
  Observation2D3D::Set observations;
  EXPECT_NO_THROW(observations = test::createObservations(camera_chain, target_chain, Eigen::Isometry3d::Identity(),
                                                          Eigen::Isometry3d::Identity(), camera_base_to_target_base,
                                                          target, camera, n));
  EXPECT_GE(observations.size(), n);
}

TEST_F(DHChainObservationGeneration, generateObservations3D)
{
  Observation3D3D::Set observations;
  EXPECT_NO_THROW(observations =
                      test::createObservations(camera_chain, target_chain, Eigen::Isometry3d::Identity(),
                                               Eigen::Isometry3d::Identity(), camera_base_to_target_base, target, n));
  EXPECT_EQ(observations.size(), n);
}

TEST_F(DHChainObservationGeneration, generateKinematicObservations2D)
{
  KinObservation2D3D::Set observations;
  test::Camera camera = test::makeKinectCamera();
  EXPECT_NO_THROW(observations = test::createKinematicObservations(
                      camera_chain, target_chain, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(),
                      camera_base_to_target_base, target, camera, n));
  EXPECT_GE(observations.size(), n);
}

TEST_F(DHChainObservationGeneration, generateKinematicObservations3D)
{
  KinObservation3D3D::Set observations;
  EXPECT_NO_THROW(observations = test::createKinematicObservations(
                      camera_chain, target_chain, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(),
                      camera_base_to_target_base, target, n));
  EXPECT_EQ(observations.size(), n);
}

TEST(DHChain, generateKinematicMeasurements)
{
  // Set up a scenario in which the camera is static and the target is the robot tool0 frame
  // Create the DH chains
  DHChain target_chain = test::createABBIRB2400();
  DHChain camera_chain({});

  Eigen::Isometry3d camera_base_to_target_base(Eigen::Isometry3d::Identity());
  // Move the target chain in front of the camera in X
  camera_base_to_target_base.translate(Eigen::Vector3d(2.0, 0.0, 0.0));
  // Rotate the target chain 180 degrees to face the camera
  camera_base_to_target_base.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));

  // Create the measurements
  const std::size_t n = 100;
  KinematicMeasurement::Set measurements =
      test::createKinematicMeasurements(camera_chain, target_chain, Eigen::Isometry3d::Identity(),
                                        Eigen::Isometry3d::Identity(), camera_base_to_target_base, n);

  EXPECT_EQ(measurements.size(), n);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
