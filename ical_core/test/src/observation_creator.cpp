#include <ical_core_tests/observation_creator.h>
#include <ical_core/optimizations/utils/ceres_math_utilities.h>
#include <ical_core_tests/pose_generator.h>
#include <ical_core/exceptions.h>

namespace industrial_calibration
{
namespace test
{
static bool projectAndTest(const Eigen::Vector3d& pt_in_camera, const Camera& camera, Eigen::Vector2d& pt_in_image)
{
  projectPoint(camera.intr, pt_in_camera.data(), pt_in_image.data());
  if (pt_in_image(0) < 0 || pt_in_image(0) > camera.width || pt_in_image(1) < 0 || pt_in_image(1) > camera.height)
  {
    return false;
  }
  else
    return true;
}

Correspondence2D3D::Set getCorrespondences(const Eigen::Isometry3d& camera_pose, const Eigen::Isometry3d& target_pose,
                                           const Camera& camera, const Target& target, const bool require_all)
{
  Correspondence2D3D::Set correspondences;
  correspondences.reserve(target.points.size());

  Eigen::Isometry3d camera_to_target = camera_pose.inverse() * target_pose;

  // Loop over points
  for (const auto& point : target.points)
  {
    Correspondence2D3D corr;
    corr.in_target = point;

    // Calculate the target coordinates in the camera frame
    Eigen::Vector3d pt_in_camera = camera_to_target * point;

    // Include points only in front of the camera and inside the image
    if (pt_in_camera.z() > 0.0 && projectAndTest(pt_in_camera, camera, corr.in_image))
    {
      correspondences.push_back(corr);
    }
    else if (require_all)
    {
      throw ICalException("Target point was not observed");
    }
  }

  return correspondences;
}

Correspondence3D3D::Set getCorrespondences(const Eigen::Isometry3d& camera_pose, const Eigen::Isometry3d& target_pose,
                                           const Target& target) noexcept
{
  Correspondence3D3D::Set correspondences;
  correspondences.reserve(target.points.size());

  Eigen::Isometry3d camera_to_target = camera_pose.inverse() * target_pose;

  for (const auto& pt : target.points)
  {
    Correspondence3D3D corr;
    corr.in_target = pt;
    corr.in_image = camera_to_target * pt;
    correspondences.push_back(corr);
  }

  return correspondences;
}

Observation2D3D::Set createObservations(const Camera& camera, const Target& target,
                                        const std::vector<std::shared_ptr<PoseGenerator>>& pose_generators,
                                        const Eigen::Isometry3d& true_target_mount_to_target,
                                        const Eigen::Isometry3d& true_camera_mount_to_camera,
                                        const Eigen::Isometry3d& camera_base_to_target_base)
{
  // Generate camera poses relative to the target origin
  std::vector<Eigen::Isometry3d> camera_poses;
  for (auto it = pose_generators.begin(); it != pose_generators.end(); ++it)
  {
    auto new_poses = it->get()->generate(true_target_mount_to_target);
    camera_poses.insert(camera_poses.end(), new_poses.begin(), new_poses.end());
  }

  Observation2D3D::Set observations;
  observations.reserve(camera_poses.size());

  for (const auto& target_to_camera_pose : camera_poses)
  {
    Observation2D3D obs;
    // Assumption: the transform to the target mount is identity
    obs.to_target_mount = Eigen::Isometry3d::Identity();
    obs.to_camera_mount = camera_base_to_target_base * obs.to_target_mount * true_target_mount_to_target *
                          target_to_camera_pose * true_camera_mount_to_camera.inverse();

    // Get the location of the camera and target relative to the same frame (i.e. camera base)
    Eigen::Isometry3d camera_base_to_camera = obs.to_camera_mount * true_camera_mount_to_camera;
    Eigen::Isometry3d camera_base_to_target =
        camera_base_to_target_base * obs.to_target_mount * true_target_mount_to_target;

    // Get correspondences
    // All camera poses may not see all target points, so don't require all to be seen
    obs.correspondence_set = getCorrespondences(camera_base_to_camera, camera_base_to_target, camera, target, false);

    observations.push_back(std::move(obs));
  }

  return observations;
}

Observation3D3D::Set createObservations(const Target& target,
                                        const std::vector<std::shared_ptr<PoseGenerator>>& pose_generators,
                                        const Eigen::Isometry3d& true_target_mount_to_target,
                                        const Eigen::Isometry3d& true_camera_mount_to_camera,
                                        const Eigen::Isometry3d& camera_base_to_target_base)
{
  // Generate camera poses relative to the target origin
  std::vector<Eigen::Isometry3d> camera_poses;
  for (auto it = pose_generators.begin(); it != pose_generators.end(); ++it)
  {
    auto new_poses = it->get()->generate(true_target_mount_to_target);
    camera_poses.insert(camera_poses.end(), new_poses.begin(), new_poses.end());
  }

  Observation3D3D::Set observations;
  observations.reserve(camera_poses.size());

  for (const auto& target_to_camera_pose : camera_poses)
  {
    Observation3D3D obs;
    // Assumption: the transform to the target mount is identity
    obs.to_target_mount = Eigen::Isometry3d::Identity();
    obs.to_camera_mount = camera_base_to_target_base * obs.to_target_mount * true_target_mount_to_target *
                          target_to_camera_pose * true_camera_mount_to_camera.inverse();

    // Get the location of the camera target relative to the same frame (i.e. in camera base)
    Eigen::Isometry3d camera_base_to_camera = obs.to_camera_mount * true_camera_mount_to_camera;
    Eigen::Isometry3d camera_base_to_target =
        camera_base_to_target_base * obs.to_target_mount * true_target_mount_to_target;

    // Get correspondences
    // Each camera pose should "see" the entire target, so don't catch any exceptions that might be thrown
    obs.correspondence_set = getCorrespondences(camera_base_to_camera, camera_base_to_target, target);

    observations.push_back(std::move(obs));
  }

  return observations;
}

}  // namespace test
}  // namespace industrial_calibration
