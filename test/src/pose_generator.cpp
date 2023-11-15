#include <industrial_calibration_tests/pose_generator.h>
#include <industrial_calibration_tests/utilities.h>

namespace industrial_calibration
{
namespace test
{
Eigen::Isometry3d lookAt(const Eigen::Vector3d& origin, const Eigen::Vector3d& eye, const Eigen::Vector3d& up) noexcept
{
  Eigen::Vector3d z = (eye - origin).normalized();
  Eigen::Vector3d x = z.cross(up).normalized();
  Eigen::Vector3d y = z.cross(x).normalized();

  auto p = Eigen::Isometry3d::Identity();
  p.translation() = origin;
  p.matrix().col(0).head<3>() = x;
  p.matrix().col(1).head<3>() = y;
  p.matrix().col(2).head<3>() = z;
  return p;
}

// hemisphere
std::vector<Eigen::Isometry3d> HemispherePoseGenerator::generate(const Eigen::Isometry3d& target_origin)
{
  std::size_t position_cnt = theta_cnt * phi_cnt;

  // all in the target coordinate system, with z forward
  std::vector<Eigen::Isometry3d> camera_positions;
  camera_positions.reserve(position_cnt);

  Eigen::VectorXd theta_range = Eigen::VectorXd::LinSpaced(theta_cnt, 0.0, M_PI);
  Eigen::VectorXd phi_range = Eigen::VectorXd::LinSpaced(phi_cnt, 0.0, M_PI);

  for (std::size_t theta_it = 0; theta_it < theta_cnt; ++theta_it)
  {
    for (std::size_t phi_it = 0; phi_it < phi_cnt; ++phi_it)
    {
      double theta_cur = theta_range(theta_it);
      double phi_cur = phi_range(phi_it);

      // position in target coordinate frame
      Eigen::Isometry3d camera = target_origin;

      Eigen::Vector3d position;
      position(0) = r * std::cos(theta_cur) * std::sin(phi_cur);
      position(1) = r * std::sin(theta_cur) * std::sin(phi_cur);
      position(2) = r * std::cos(phi_cur);

      camera.translate(position);

      // x is 'up' in target frame
      Eigen::Isometry3d camera_oriented =
          target_offset * lookAt(camera.translation(), target_origin.translation(), Eigen::Vector3d(1, 0, 0)) *
          Eigen::Isometry3d(Eigen::AngleAxisd(z_rot, Eigen::Vector3d::UnitZ()));

      // this vector is still in target spatial coordinates
      camera_positions.push_back(camera_oriented);
    }
  }

  return camera_positions;
}

// cone
std::vector<Eigen::Isometry3d> ConicalPoseGenerator::generate(const Eigen::Isometry3d& target_origin)
{
  std::vector<Eigen::Isometry3d> camera_positions;
  camera_positions.reserve(n_poses);

  // Generates positions in target frame; need to convert to world frame
  double dt = 2.0f * M_PI / double(n_poses);  // delta theta about cone base

  for (unsigned i = 0; i < n_poses; ++i)
  {
    Eigen::Isometry3d camera_pose = target_origin;

    // preserving target spatial coordinate frame:
    camera_pose.translate(Eigen::Vector3d{ r * cos(i * dt), r * sin(i * dt), h });

    // change orientation to look at target
    Eigen::Isometry3d camera_oriented =
        target_offset * lookAt(camera_pose.translation(), target_origin.translation(), Eigen::Vector3d(1, 0, 0)) *
        Eigen::Isometry3d(Eigen::AngleAxisd(z_rot, Eigen::Vector3d::UnitZ()));

    camera_positions.push_back(camera_oriented);
  }

  return camera_positions;
}

// grid
std::vector<Eigen::Isometry3d> GridPoseGenerator::generate(const Eigen::Isometry3d& target_origin)
{
  // Generates positions in target frame; need to convert to world frame
  std::vector<Eigen::Isometry3d> camera_positions;
  camera_positions.reserve(grid_side * grid_side);

  double end_point = (static_cast<double>(grid_side - 1) / 2.0) * spacing;
  Eigen::VectorXd grid_coords = Eigen::VectorXd::LinSpaced(grid_side, -1 * end_point, end_point);

  for (std::size_t i = 0; i < grid_side; ++i)
  {
    for (std::size_t j = 0; j < grid_side; ++j)
    {
      Eigen::Isometry3d camera_pose = target_origin;
      camera_pose.translate(Eigen::Vector3d{ grid_coords(i), grid_coords(j), h });

      // change orientation to look at target
      Eigen::Isometry3d camera_oriented =
          target_offset * lookAt(camera_pose.translation(), target_origin.translation(), Eigen::Vector3d(1, 0, 0)) *
          Eigen::Isometry3d(Eigen::AngleAxisd(z_rot, Eigen::Vector3d::UnitZ()));

      camera_positions.push_back(camera_oriented);
    }
  }

  return camera_positions;
}

// random Z+
std::vector<Eigen::Isometry3d> RandomZRotPoseGenerator::generate(const Eigen::Isometry3d& target_origin)
{
  auto poses = pg->generate(target_origin);
  for (auto pose : poses)
  {
    pose.rotate(Eigen::AngleAxisd(z_sampler(mt_rand), Eigen::Vector3d::UnitZ()));
  }
  return poses;
}

}  // namespace test
}  // namespace industrial_calibration
