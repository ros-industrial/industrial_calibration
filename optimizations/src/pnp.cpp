#include <industrial_calibration/optimizations/pnp.h>
#include <industrial_calibration/optimizations/pnp_cost.h>
#include <industrial_calibration/optimizations/covariance_analysis.h>
#include <industrial_calibration/core/serialization.h>

#include <ceres/ceres.h>

namespace industrial_calibration
{
std::ostream& operator<<(std::ostream& stream, const PnPResult& result)
{
  stream << "Optimization " << (result.converged ? "converged" : "did not converge") << "\n";
  stream << "Initial cost per observation (pixels): " << std::sqrt(result.initial_cost_per_obs) << "\n";
  stream << "Final cost per obsrevation (pixels): " << std::sqrt(result.final_cost_per_obs);

  if (result.converged)
  {
    std::stringstream ss;
    ss << "Camera to target transform\n";
    writeTransform(ss, result.camera_to_target);
    stream << "\n" << ss.str();
  }

  return stream;
}

PnPResult optimize(const PnPProblem& params)
{
  // Create the optimization variables from the input guess
  Eigen::AngleAxisd cam_to_tgt_rotation(params.camera_to_target_guess.rotation());
  Eigen::Vector3d cam_to_tgt_angle_axis = cam_to_tgt_rotation.angle() * cam_to_tgt_rotation.axis();
  Eigen::Vector3d cam_to_tgt_translation(params.camera_to_target_guess.translation());

  ceres::Problem problem;

  // For each 3D point seen in the 2D image
  for (const auto& corr : params.correspondences)
  {
    // Allocate Ceres data structures - ownership is taken by the ceres
    // Problem data structure
    auto* cost_fn = new PnPCost(params.intr, corr.in_target, corr.in_image);

    auto* cost_block = new ceres::AutoDiffCostFunction<PnPCost, 2, 3, 3>(cost_fn);

    problem.AddResidualBlock(cost_block, nullptr, cam_to_tgt_angle_axis.data(), cam_to_tgt_translation.data());
  }

  ceres::Solver::Summary summary;
  ceres::Solver::Options options;
  ceres::Solve(options, &problem, &summary);

  PnPResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;
  result.camera_to_target = Eigen::Translation3d(cam_to_tgt_translation) *
                            Eigen::AngleAxisd(cam_to_tgt_angle_axis.norm(), cam_to_tgt_angle_axis.normalized());

  // compose labels "camera_to_target_x", etc.
  std::vector<std::string> labels_camera_to_target_guess_translation;
  for (auto label_t : params.labels_translation)
  {
    labels_camera_to_target_guess_translation.emplace_back(params.label_camera_to_target_guess + "_" + label_t);
  }

  // compose labels "camera_to_target_qx", etc.
  std::vector<std::string> labels_camera_to_target_guess_quaternion;
  for (auto label_r : params.labels_rotation)
  {
    labels_camera_to_target_guess_quaternion.emplace_back(params.label_camera_to_target_guess + "_" + label_r);
  }
  std::map<const double*, std::vector<std::string>> param_labels;
  param_labels[cam_to_tgt_translation.data()] = labels_camera_to_target_guess_translation;
  param_labels[cam_to_tgt_angle_axis.data()] = labels_camera_to_target_guess_quaternion;

  // Optionally compute covariance
  try
  {
    std::map<const double*, std::vector<int>> param_masks;
    ceres::Covariance::Options cov_options = DefaultCovarianceOptions();
    cov_options.null_space_rank = -1;  // automatically drop terms below min_reciprocal_condition_number

    result.covariance = computeCovariance(
        problem, std::vector<const double*>({ cam_to_tgt_translation.data(), cam_to_tgt_angle_axis.data() }),
        param_labels, param_masks, cov_options);
  }
  catch (const ICalException& ex)
  {
  }

  return result;
}

PnPResult optimize(const PnPProblem3D& params)
{
  // Create the optimization variables from the input guess
  Eigen::AngleAxisd cam_to_tgt_rotation(params.camera_to_target_guess.rotation());
  Eigen::Vector3d cam_to_tgt_angle_axis(cam_to_tgt_rotation.angle() * cam_to_tgt_rotation.axis());
  Eigen::Vector3d cam_to_tgt_translation(params.camera_to_target_guess.translation());

  ceres::Problem problem;

  // only one loop, for correspondences
  for (const auto& corr : params.correspondences)  // For each 3D point seen in the 3D image
  {
    // Allocate Ceres data structures - ownership is taken by the ceres
    // Problem data structure

    auto* cost_fn = new PnP3DCost(corr.in_target, corr.in_image);

    auto* cost_block = new ceres::AutoDiffCostFunction<PnP3DCost, 3, 3, 3>(cost_fn);

    problem.AddResidualBlock(cost_block, nullptr, cam_to_tgt_angle_axis.data(), cam_to_tgt_translation.data());
  }

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  PnPResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;
  result.camera_to_target = Eigen::Translation3d(cam_to_tgt_translation) *
                            Eigen::AngleAxisd(cam_to_tgt_angle_axis.norm(), cam_to_tgt_angle_axis.normalized());

  // compose labels "camera_to_target_x", etc.
  std::vector<std::string> labels_camera_to_target_guess_translation;
  for (auto label_t : params.labels_translation)
  {
    labels_camera_to_target_guess_translation.emplace_back(params.label_camera_to_target_guess + "_" + label_t);
  }

  // compose labels "camera_to_target_qx", etc.
  std::vector<std::string> labels_camera_to_target_guess_quaternion;
  for (auto label_r : params.labels_rotation)
  {
    labels_camera_to_target_guess_quaternion.emplace_back(params.label_camera_to_target_guess + "_" + label_r);
  }
  std::map<const double*, std::vector<std::string>> param_labels;
  param_labels[cam_to_tgt_translation.data()] = labels_camera_to_target_guess_translation;
  param_labels[cam_to_tgt_angle_axis.data()] = labels_camera_to_target_guess_quaternion;

  result.covariance = computeCovariance(
      problem, std::vector<const double*>({ cam_to_tgt_translation.data(), cam_to_tgt_angle_axis.data() }),
      param_labels);

  return result;
}

}  // namespace industrial_calibration
