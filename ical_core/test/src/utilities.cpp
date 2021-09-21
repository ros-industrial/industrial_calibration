#include <ical_core_tests/utilities.h>
#include <random>

namespace industrial_calibration
{
namespace test
{
Camera makeKinectCamera()
{
  CameraIntrinsics intr;
  intr.fx() = 550.0;
  intr.fy() = 550.0;
  intr.cx() = 640 / 2;
  intr.cy() = 480 / 2;

  Camera camera;
  camera.intr = intr;
  camera.width = 640;
  camera.height = 480;

  return camera;
}

Eigen::Isometry3d perturbPose(const Eigen::Isometry3d& pose, double spatial_noise, double angle_noise)
{
  auto mt_rand = std::mt19937(RANDOM_SEED);

  auto spatial_dist = std::bind(std::uniform_real_distribution<double>{ -spatial_noise, spatial_noise }, mt_rand);
  auto angle_dist = std::bind(std::uniform_real_distribution<double>{ -angle_noise, angle_noise }, mt_rand);
  auto one_to_one = std::bind(std::uniform_real_distribution<double>{ -1, 1 }, mt_rand);

  Eigen::Vector3d translation(spatial_dist(), spatial_dist(), spatial_dist());
  Eigen::Vector3d rot_axis(one_to_one(), one_to_one(), one_to_one());
  rot_axis.normalize();

  double angle = angle_dist();

  Eigen::Isometry3d new_pose = pose * Eigen::Translation3d(translation) * Eigen::AngleAxisd(angle, rot_axis);

  return new_pose;
}

DHChain createABBIRB2400()
{
  std::vector<DHTransform> transforms;
  transforms.reserve(6);

  Eigen::Vector4d p1, p2, p3, p4, p5, p6;

  p1 << 0.615, 0.0, 0.100, -M_PI / 2.0;
  p2 << 0.0, -M_PI / 2.0, 0.705, 0.0;
  p3 << 0.0, 0.0, 0.135, -M_PI / 2.0;
  p4 << 0.755, 0.0, 0.0, M_PI / 2.0;
  p5 << 0.0, 0.0, 0.0, -M_PI / 2.0;
  p6 << 0.085, M_PI, 0.0, 0.0;

  transforms.emplace_back(p1, DHJointType::REVOLUTE, "j1", RANDOM_SEED);
  transforms.emplace_back(p2, DHJointType::REVOLUTE, "j2", RANDOM_SEED);
  transforms.emplace_back(p3, DHJointType::REVOLUTE, "j3", RANDOM_SEED);
  transforms.emplace_back(p4, DHJointType::REVOLUTE, "j4", RANDOM_SEED);
  transforms.emplace_back(p5, DHJointType::REVOLUTE, "j5", RANDOM_SEED);
  transforms.emplace_back(p6, DHJointType::REVOLUTE, "j6", RANDOM_SEED);

  return DHChain(transforms);
}

DHChain createTwoAxisPositioner()
{
  std::vector<DHTransform> transforms;
  transforms.reserve(2);

  Eigen::Vector4d p1, p2;
  p1 << 0.0, 0.0, 0.0, -M_PI / 2.0;
  p2 << -0.475, -M_PI / 2.0, 0.0, 0.0;

  // Add the first DH transform
  {
    DHTransform t(p1, DHJointType::REVOLUTE, "j1", RANDOM_SEED);
    t.max = M_PI;
    t.min = -M_PI;
    transforms.push_back(t);
  }
  // Add the second DH transform
  {
    DHTransform dh_transform(p2, DHJointType::REVOLUTE, "j2", RANDOM_SEED);
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

DHChain perturbDHChain(const DHChain& in, const double stddev)
{
  std::mt19937 mt_rand(RANDOM_SEED);
  std::normal_distribution<double> norm(0.0, stddev);

  // Get the joint types and nominal DH table
  std::vector<DHJointType> joint_types = in.getJointTypes();
  Eigen::MatrixX4d dh = in.getDHTable();

  // Perturb each value in the DH table randomly by the input standard deviation
  dh = dh.unaryExpr([&norm, &mt_rand](const double val) { return val + norm(mt_rand); });

  std::vector<DHTransform> transforms;
  transforms.reserve(joint_types.size());
  for (std::size_t i = 0; i < joint_types.size(); ++i)
  {
    transforms.emplace_back(dh.row(i), joint_types[i], "j" + std::to_string(i + 1), RANDOM_SEED);
  }
  return DHChain(transforms, in.getBaseOffset());
}

}  // namespace test
}  // namespace industrial_calibration
