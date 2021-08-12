#include <ical_core_tests/dh_chain_observation_creator.h>
#include <ical_core_tests/observation_creator.h>

namespace industrial_calibration
{
namespace test
{
KinObservation3D3D::Set createKinematicObservations(const DHChain& to_camera_chain, const DHChain& to_target_chain,
                                                    const Eigen::Isometry3d& true_mount_to_camera,
                                                    const Eigen::Isometry3d& true_mount_to_target,
                                                    const Eigen::Isometry3d& camera_base_to_target_base,
                                                    const Target& target, const std::size_t n)
{
  KinObservation3D3D::Set observations;
  observations.reserve(n);

  for (std::size_t i = 0; i < n; ++i)
  {
    KinObservation3D3D obs;
    obs.camera_chain_joints = to_camera_chain.createUniformlyRandomPose();
    obs.target_chain_joints = to_target_chain.createUniformlyRandomPose();

    Eigen::Isometry3d to_camera_mount = to_camera_chain.getFK<double>(obs.camera_chain_joints);
    Eigen::Isometry3d to_target_mount = to_target_chain.getFK<double>(obs.target_chain_joints);

    obs.correspondence_set =
        getCorrespondences(to_camera_mount * true_mount_to_camera,
                           camera_base_to_target_base * to_target_mount * true_mount_to_target, target);
    observations.push_back(obs);
  }

  return observations;
}

Observation3D3D::Set createObservations(const DHChain& to_camera_chain, const DHChain& to_target_chain,
                                        const Eigen::Isometry3d& true_mount_to_camera,
                                        const Eigen::Isometry3d& true_mount_to_target,
                                        const Eigen::Isometry3d& camera_base_to_target_base, const Target& target,
                                        const std::size_t n)
{
  // Get a set of kinematic observations
  KinObservation3D3D::Set kin_obs_set =
      createKinematicObservations(to_camera_chain, to_target_chain, true_mount_to_camera, true_mount_to_target,
                                  camera_base_to_target_base, target, n);

  // Convert the joint states of the kinematic observations into Cartesian poses
  Observation3D3D::Set obs_set;
  obs_set.reserve(kin_obs_set.size());
  for (const auto& kin_obs : kin_obs_set)
  {
    Observation3D3D obs;
    obs.to_camera_mount = to_camera_chain.getFK(kin_obs.camera_chain_joints);
    obs.to_target_mount = to_target_chain.getFK(kin_obs.target_chain_joints);
    obs.correspondence_set = kin_obs.correspondence_set;
    obs_set.push_back(obs);
  }

  return obs_set;
}

KinObservation2D3D::Set createKinematicObservations(const DHChain& to_camera_chain, const DHChain& to_target_chain,
                                                    const Eigen::Isometry3d& true_mount_to_camera,
                                                    const Eigen::Isometry3d& true_mount_to_target,
                                                    const Eigen::Isometry3d& camera_base_to_target_base,
                                                    const Target& target, const Camera& camera, const std::size_t n)
{
  KinObservation2D3D::Set observations;
  observations.reserve(n);

  std::size_t correspondences = 0;
  std::size_t attempts = 0;
  const std::size_t max_attempts = 10000;
  while (correspondences < n * target.points.size() && attempts < max_attempts)
  {
    KinObservation2D3D obs;
    obs.camera_chain_joints = to_camera_chain.createUniformlyRandomPose();
    obs.target_chain_joints = to_target_chain.createUniformlyRandomPose();

    Eigen::Isometry3d to_camera_mount = to_camera_chain.getFK(obs.camera_chain_joints);
    Eigen::Isometry3d to_target_mount = to_target_chain.getFK(obs.target_chain_joints);

    const Eigen::Isometry3d base_to_camera = to_camera_mount * true_mount_to_camera;
    const Eigen::Isometry3d base_to_target = camera_base_to_target_base * to_target_mount * true_mount_to_target;

    const Eigen::Vector3d& camera_z = base_to_camera.matrix().col(2).head<3>();
    const Eigen::Vector3d& target_z = base_to_target.matrix().col(2).head<3>();

    // Make sure the camera and target normals are not pointing in the same direction
    double dp = camera_z.dot(target_z);
    if (dp < -0.2)
    {
      try
      {
        obs.correspondence_set = getCorrespondences(base_to_camera, base_to_target, camera, target, false);
      }
      catch (const std::exception&)
      {
        continue;
      }

      if (obs.correspondence_set.size() > 0)
      {
        observations.push_back(obs);
        correspondences += obs.correspondence_set.size();
      }
    }

    ++attempts;
  }

  if (attempts > max_attempts || correspondences < n * target.points.size())
  {
    std::stringstream ss;
    ss << "Failed to generate required number of observations (" << observations.size() << "/"
       << observations.capacity() << ")";
    throw std::runtime_error(ss.str());
  }

  return observations;
}

Observation2D3D::Set createObservations(const DHChain& to_camera_chain, const DHChain& to_target_chain,
                                        const Eigen::Isometry3d& true_mount_to_camera,
                                        const Eigen::Isometry3d& true_mount_to_target,
                                        const Eigen::Isometry3d& camera_base_to_target_base, const Target& target,
                                        const Camera& camera, const std::size_t n)
{
  // Get a set of kinematic observations
  KinObservation2D3D::Set kin_obs_set =
      createKinematicObservations(to_camera_chain, to_target_chain, true_mount_to_camera, true_mount_to_target,
                                  camera_base_to_target_base, target, camera, n);

  // Convert the joint states of the kinematic observations into Cartesian poses
  Observation2D3D::Set obs_set;
  obs_set.reserve(kin_obs_set.size());
  for (const auto& kin_obs : kin_obs_set)
  {
    Observation2D3D obs;
    obs.to_camera_mount = to_camera_chain.getFK(kin_obs.camera_chain_joints);
    obs.to_target_mount = to_target_chain.getFK(kin_obs.target_chain_joints);
    obs.correspondence_set = kin_obs.correspondence_set;
    obs_set.push_back(obs);
  }

  return obs_set;
}

KinematicMeasurement::Set createKinematicMeasurements(const DHChain& to_camera_chain, const DHChain& to_target_chain,
                                                      const Eigen::Isometry3d& true_mount_to_camera,
                                                      const Eigen::Isometry3d& true_mount_to_target,
                                                      const Eigen::Isometry3d& camera_base_to_target_base,
                                                      const std::size_t n)
{
  KinematicMeasurement::Set measurements;
  measurements.reserve(n);

  for (std::size_t i = 0; i < n; ++i)
  {
    KinematicMeasurement m;
    m.camera_chain_joints = to_camera_chain.createUniformlyRandomPose();
    m.target_chain_joints = to_target_chain.createUniformlyRandomPose();

    const Eigen::Isometry3d camera_base_to_camera = to_camera_chain.getFK(m.camera_chain_joints) * true_mount_to_camera;
    const Eigen::Isometry3d camera_base_to_target =
        camera_base_to_target_base * to_target_chain.getFK(m.target_chain_joints) * true_mount_to_target;
    m.camera_to_target = camera_base_to_camera.inverse() * camera_base_to_target;

    measurements.push_back(m);
  }

  return measurements;
}

}  // namespace test
}  // namespace industrial_calibration
