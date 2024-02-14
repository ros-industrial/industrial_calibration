#include <industrial_calibration/optimizations/analysis/extrinsic_hand_eye_calibration_analysis.h>
#include <industrial_calibration/optimizations/analysis/statistics.h>
#include <industrial_calibration/optimizations/pnp.h>

namespace industrial_calibration
{
ExtrinsicHandEyeAnalysisStats analyzeResults(const ExtrinsicHandEyeProblem2D3D& problem,
                                             const ExtrinsicHandEyeResult& opt_result)
{
  // Create accumulators to more easily calculate the mean and standard deviation of the position and orientation
  // differences
  std::vector<double> pos_diff_acc, ori_diff_acc;
  pos_diff_acc.reserve(problem.observations.size());
  ori_diff_acc.reserve(problem.observations.size());

  // Iterate over all of the images in which an observation of the target was made
  for (std::size_t i = 0; i < problem.observations.size(); ++i)
  {
    // Get the observation
    const Observation2D3D& obs = problem.observations.at(i);

    // Calculate the optimized transform from the camera to the target for the ith observation
    Eigen::Isometry3d camera_to_target = opt_result.camera_mount_to_camera.inverse() * obs.to_camera_mount.inverse() *
                                         obs.to_target_mount * opt_result.target_mount_to_target;

    // Get the same transformation from a PnP optimization with the known camera intrinsic parameters
    PnPProblem pnp;
    pnp.camera_to_target_guess = camera_to_target;
    pnp.correspondences = obs.correspondence_set;
    pnp.intr = problem.intr;
    PnPResult pnp_result = optimize(pnp);

    // Calculate the difference between the two transforms
    Eigen::Isometry3d diff = camera_to_target.inverse() * pnp_result.camera_to_target;

    // Accumulate the differences
    pos_diff_acc.push_back(diff.translation().norm());
    ori_diff_acc.push_back(Eigen::Quaterniond(camera_to_target.linear())
                               .angularDistance(Eigen::Quaterniond(pnp_result.camera_to_target.linear())));
  }

  ExtrinsicHandEyeAnalysisStats stats;
  std::tie(stats.pos_diff_mean, stats.pos_diff_stdev) = computeStats(pos_diff_acc);
  std::tie(stats.ori_diff_mean, stats.ori_diff_stdev) = computeStats(ori_diff_acc);

  return stats;
}

}  // namespace industrial_calibration
