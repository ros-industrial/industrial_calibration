#include <ical_core/optimizations/analysis/camera_intrinsic_calibration_analysis.h>
#include <ical_core/optimizations/analysis/statistics.h>
#include <ical_core/optimizations/pnp.h>
#include <ical_core/exceptions.h>

namespace industrial_calibration
{
VirtualCorrespondenceResult measureVirtualTargetDiff(const Correspondence2D3D::Set& correspondences,
                                                     const CameraIntrinsics& intr,
                                                     const Eigen::Isometry3d& camera_to_target_guess,
                                                     const double pnp_sq_error_threshold)
{
  // Create a lambda for doing the PnP optimization
  auto solve_pnp = [&intr, &camera_to_target_guess,
                    &pnp_sq_error_threshold](const Correspondence2D3D::Set& corr) -> Eigen::Isometry3d {
    // Create the first virtual target PnP problem
    PnPProblem problem;
    problem.intr = intr;
    problem.correspondences = corr;
    problem.camera_to_target_guess = camera_to_target_guess;

    PnPResult result = optimize(problem);
    if (!result.converged || result.final_cost_per_obs > pnp_sq_error_threshold)
    {
      std::stringstream ss;
      ss << "PnP optimization " << (result.converged ? "converged" : "did not converge") << " with residual error of "
         << result.final_cost_per_obs << " (" << pnp_sq_error_threshold << " max)";
      throw ICalException(ss.str());
    }

    return result.camera_to_target;
  };

  // Calculate the size of half of the correspondence set
  std::size_t half_size = correspondences.size() / 2;

  // Create two half sets of correspondences
  Correspondence2D3D::Set set_1(correspondences.begin(), correspondences.begin() + half_size);
  Correspondence2D3D::Set set_2(correspondences.begin() + half_size, correspondences.end());

  /* Get the camera to target transformation for each half set
   * Note: these transforms are from the camera to the origin of each virtual target.
   *   The origin of the second virtual target is still the same as the first virtual target,
   *   so the two transforms should be the same, given perfect camera intrinsics */
  Eigen::Isometry3d camera_to_target_1 = solve_pnp(set_1);
  Eigen::Isometry3d camera_to_target_2 = solve_pnp(set_2);

  VirtualCorrespondenceResult res;

  Eigen::Quaterniond q1(camera_to_target_1.rotation());
  Eigen::Quaterniond q2(camera_to_target_2.rotation());
  res.angular_error = q1.angularDistance(q2);
  res.positional_error = (camera_to_target_1.inverse() * camera_to_target_2).translation().norm();

  // Return the transformation from target 1 to target 2
  res.t1_to_t2 = camera_to_target_1.inverse() * camera_to_target_2;
  return res;
}

IntrinsicCalibrationAccuracyResult measureIntrinsicCalibrationAccuracy(
    const Observation2D3D::Set& observations, const CameraIntrinsics& intr,
    const Eigen::Isometry3d& camera_mount_to_camera, const Eigen::Isometry3d& target_mount_to_target,
    const Eigen::Isometry3d& camera_base_to_target_base, const double pnp_sq_error_threshold)
{
  // Check that the observations are all the same size
  // Assuming that each observation's correspondences are ordered the same
  for (std::size_t i = 0; i < observations.size() - 1; ++i)
  {
    const Observation2D3D& obs_1 = observations[i];
    const Observation2D3D& obs_2 = observations[i + 1];

    // Check that the correspondences in all observations are the same size
    if (obs_1.correspondence_set.size() != obs_2.correspondence_set.size())
    {
      std::stringstream ss;
      ss << "Correspondence sizes do not match between observations " << i << " and " << i + 1;
      throw OptimizationException(ss.str());
    }
  }

  // Create accumulators for mean and variance
  std::vector<double> pos_acc, ang_acc;
  pos_acc.reserve(observations.size());
  ang_acc.reserve(observations.size());

  // Accumulate the position vector of the transformation
  for (const auto& obs : observations)
  {
    Eigen::Isometry3d camera_base_to_camera = obs.to_camera_mount * camera_mount_to_camera;
    Eigen::Isometry3d camera_base_to_target = camera_base_to_target_base * obs.to_target_mount * target_mount_to_target;

    Eigen::Isometry3d camera_to_target = camera_base_to_camera.inverse() * camera_base_to_target;

    VirtualCorrespondenceResult res =
        measureVirtualTargetDiff(obs.correspondence_set, intr, camera_to_target, pnp_sq_error_threshold);
    pos_acc.push_back(res.positional_error);
    ang_acc.push_back(res.angular_error);
  }

  // Calculate the mean and variance of the measurements
  // Theoretically each transform from virtual target 1 to virtual target 2 should be zero; thus the mean should be zero
  // In practice the mean represents bias from the ideal zero state and the variance is the amount of change around the
  // mean
  IntrinsicCalibrationAccuracyResult res;
  std::tie(res.pos_error.first, res.pos_error.second) = computeStats(pos_acc);
  std::tie(res.ang_error.first, res.ang_error.second) = computeStats(ang_acc);

  return res;
}

}  // namespace industrial_calibration
