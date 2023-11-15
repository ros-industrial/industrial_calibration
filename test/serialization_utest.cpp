#include <industrial_calibration/serialization/problems.h>
#include <industrial_calibration_tests/utilities.h>
#include <industrial_calibration_tests/observation_creator.h>
#include <industrial_calibration/exceptions.h>

#include <gtest/gtest.h>
#include <fstream>

using namespace industrial_calibration;

template <class T>
void serialize(const std::string& file, const T& val)
{
  std::ofstream ofh(file);
  if (!ofh) throw ICalException("Failed to open file '" + file + "'");

  YAML::Node n = YAML::Node(val);
  ofh << n;
}

template <class T>
T deserialize(const std::string& file)
{
  YAML::Node node;
  node = YAML::LoadFile(file);
  return node.as<T>();
}

template <typename ProblemT>
struct ProblemCreator
{
  static ProblemT createProblem(const Eigen::Isometry3d& true_target, const Eigen::Isometry3d& true_camera,
                                const std::shared_ptr<test::PoseGenerator>& pose_generator, const test::Target& target);
};

template <>
ExtrinsicHandEyeProblem2D3D ProblemCreator<ExtrinsicHandEyeProblem2D3D>::createProblem(
    const Eigen::Isometry3d& true_target, const Eigen::Isometry3d& true_camera,
    const std::shared_ptr<test::PoseGenerator>& pose_generator, const test::Target& target)
{
  test::Camera camera = test::makeKinectCamera();

  ExtrinsicHandEyeProblem2D3D problem;
  problem.intr = camera.intr;
  problem.target_mount_to_target_guess = true_target;
  problem.camera_mount_to_camera_guess = true_camera;
  problem.observations = test::createObservations(camera, target, { pose_generator }, true_target, true_camera);

  return problem;
}

template <>
ExtrinsicHandEyeProblem3D3D ProblemCreator<ExtrinsicHandEyeProblem3D3D>::createProblem(
    const Eigen::Isometry3d& true_target, const Eigen::Isometry3d& true_camera,
    const std::shared_ptr<test::PoseGenerator>& pose_generator, const test::Target& target)
{
  ExtrinsicHandEyeProblem3D3D problem;
  problem.target_mount_to_target_guess = true_target;
  problem.camera_mount_to_camera_guess = true_camera;
  problem.observations = test::createObservations(target, { pose_generator }, true_target, true_camera);
  return problem;
}

template <typename ProblemT>
class SerializationTestFixture : public ::testing::Test
{
public:
  SerializationTestFixture()
    : target_mount_to_target(Eigen::Isometry3d::Identity())
    , camera_mount_to_camera(Eigen::Isometry3d::Identity())
    , target(5, 7, 0.025)
    , pg(std::make_shared<test::HemispherePoseGenerator>())
  {
    target_mount_to_target.translate(Eigen::Vector3d(1.0, 0, 0.0));

    camera_mount_to_camera.translation() = Eigen::Vector3d(0.05, 0, 0.1);
    camera_mount_to_camera.linear() << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  }

  Eigen::Isometry3d target_mount_to_target;
  Eigen::Isometry3d camera_mount_to_camera;
  test::Target target;
  std::shared_ptr<test::PoseGenerator> pg;
};

using Implementations = testing::Types<ExtrinsicHandEyeProblem2D3D, ExtrinsicHandEyeProblem3D3D>;

TYPED_TEST_CASE(SerializationTestFixture, Implementations);

TYPED_TEST(SerializationTestFixture, Test)
{
  TypeParam problem = ProblemCreator<TypeParam>::createProblem(this->target_mount_to_target,
                                                               this->camera_mount_to_camera, this->pg, this->target);

  const std::string filename = "/tmp/problem.yaml";
  ASSERT_NO_THROW(serialize(filename, problem));
  TypeParam deserialized_problem;
  ASSERT_NO_THROW(deserialized_problem = deserialize<TypeParam>(filename));
  ASSERT_EQ(problem, deserialized_problem);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
