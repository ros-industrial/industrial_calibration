#include <industrial_calibration/optimizations/analysis/extrinsic_hand_eye_calibration_analysis.h>
#include <industrial_calibration/optimizations/analysis/statistics.h>
#include <industrial_calibration/optimizations/analysis/projection.h>
#include <industrial_calibration/optimizations/extrinsic_hand_eye.h>
#include <industrial_calibration/optimizations/pnp.h>

namespace industrial_calibration
{
std::ostream& operator<<(std::ostream& stream, const ExtrinsicHandEyeAnalysisStats& stats)
{
  stream << "Difference in camera to target transform between extrinsic calibration and PnP optimization\n"
         << "Position:\n\tMean (m): " << stats.pos_diff_mean << "\n\tStd. Dev. (m): " << stats.pos_diff_stdev << "\n"
         << "Orientation:\n\tMean (deg): " << stats.ori_diff_mean * 180.0 / M_PI
         << "\n\tStd. Dev. (deg): " << stats.ori_diff_stdev * 180.0 / M_PI;
  return stream;
}

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

std::ostream& operator<<(std::ostream& stream, const ExtrinsicHandEye3dProjectionStats& stats)
{
  stream << "3D reprojection error statistics:"
         << "\n\tMean +/- Std. Dev. (m): " << stats.mean << " +/- " << stats.stdev << "\n\tMin (m): " << stats.min
         << "\n\tMax (m): " << stats.max;
  return stream;
}

ExtrinsicHandEye3dProjectionStats analyze3dProjectionError(const ExtrinsicHandEyeProblem2D3D& problem,
                                                           const ExtrinsicHandEyeResult& opt_result)
{
  Eigen::ArrayXd error = compute3DProjectionError(problem.observations, problem.intr, opt_result.camera_mount_to_camera,
                                                  opt_result.target_mount_to_target);

  // Compute stats
  ExtrinsicHandEye3dProjectionStats stats;
  stats.mean = error.mean();
  stats.stdev = std::sqrt((error - error.mean()).square().sum() / (error.size() - 1));
  stats.min = error.minCoeff();
  stats.max = error.maxCoeff();

  return stats;
}

}  // namespace industrial_calibration
