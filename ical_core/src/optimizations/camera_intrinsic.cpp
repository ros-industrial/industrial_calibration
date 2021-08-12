#include <ical_core/optimizations/camera_intrinsic.h>
#include <ical_core/cost_functions/camera_intrinsic.h>
#include <ical_core/optimizations/utils/ceres_math_utilities.h>
#include <ical_core/optimizations/utils/covariance_analysis.h>
#include <ical_core/optimizations/pnp.h>

#include <ceres/ceres.h>

namespace industrial_calibration
{
static Pose6d solvePnP(const CameraIntrinsics& intr, const Correspondence2D3D::Set& obs, const Pose6d& guess)
{
  using namespace industrial_calibration;

  PnPProblem problem;
  problem.camera_to_target_guess = poseCalToEigen(guess);
  problem.intr = intr;
  problem.correspondences = obs;

  PnPResult result = optimize(problem);

  if (!result.converged) throw std::runtime_error("unable to solve PnP sub-problem");

  return poseEigenToCal(result.camera_to_target);
}

static Pose6d guessInitialPose()
{
  Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
  guess = guess * Eigen::Translation3d(0, 0, 0.5) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  return poseEigenToCal(guess);
}

CameraIntrinsicResult optimize(const CameraIntrinsicProblem& params)
{
  // Prepare data structure for the camera parameters to optimize
  std::array<double, CalibCameraIntrinsics<double>::size()> internal_intrinsics_data;
  for (int i = 0; i < 9; ++i)
    internal_intrinsics_data[i] = 0.0;

  MutableCalibCameraIntrinsics<double> internal_intrinsics(internal_intrinsics_data.data());
  internal_intrinsics.fx() = params.intrinsics_guess.fx();
  internal_intrinsics.fy() = params.intrinsics_guess.fy();
  internal_intrinsics.cx() = params.intrinsics_guess.cx();
  internal_intrinsics.cy() = params.intrinsics_guess.cy();

  // Prepare space for the target poses to estimate (1 for each observation set)
  std::vector<Pose6d> internal_poses(params.image_observations.size());

  // All of the target poses are seeded to be "in front of" and "looking at" the camera
  for (std::size_t i = 0; i < params.image_observations.size(); ++i)
  {
    internal_poses[i] = solvePnP(params.intrinsics_guess, params.image_observations[i], guessInitialPose());
  }

  ceres::Problem problem;

  // Create a set of cost functions for each observation set
  for (std::size_t i = 0; i < params.image_observations.size(); ++i)
  {
    // Create a cost for each 2D -> 3D image correspondence
    for (std::size_t j = 0; j < params.image_observations[i].size(); ++j)
    {
      const auto& point_in_target = params.image_observations[i][j].in_target;
      const auto& point_in_image = params.image_observations[i][j].in_image;

      // Allocate Ceres data structures - ownership is taken by the ceres
      // Problem data structure
      auto* cost_fn = new CameraIntrinsicCost(point_in_target, point_in_image);

      auto* cost_block = new ceres::AutoDiffCostFunction<CameraIntrinsicCost, 2, 6, 9>(cost_fn);

      problem.AddResidualBlock(cost_block, NULL, internal_poses[i].values.data(), internal_intrinsics_data.data());
    }
  }

  std::vector<const double*> param_blocks;
  std::vector<std::vector<std::string>> param_labels;

  param_blocks.emplace_back(internal_intrinsics_data.data());
  param_labels.emplace_back(params.labels_intrinsic_params.begin(), params.labels_intrinsic_params.end());

  for (std::size_t i = 0; i < internal_poses.size(); i++)
  {
    param_blocks.emplace_back(internal_poses[i].values.data());
    std::vector<std::string> labels;
    // compose labels poseN_x, etc.
    for (auto label_extr : params.labels_isometry3d)
    {
      labels.push_back(params.label_extr + std::to_string(i) + "_" + label_extr);
    }
    param_labels.push_back(labels);
  }

  // Solve
  ceres::Solver::Options options;
  options.max_num_iterations = 1000;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Package results
  CameraIntrinsicResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;

  result.intrinsics.fx() = internal_intrinsics.fx();
  result.intrinsics.fy() = internal_intrinsics.fy();
  result.intrinsics.cx() = internal_intrinsics.cx();
  result.intrinsics.cy() = internal_intrinsics.cy();

  result.distortions[0] = internal_intrinsics_data[4];
  result.distortions[1] = internal_intrinsics_data[5];
  result.distortions[2] = internal_intrinsics_data[6];
  result.distortions[3] = internal_intrinsics_data[7];
  result.distortions[4] = internal_intrinsics_data[8];

  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;

  result.covariance = computeCovariance(problem, param_blocks, param_labels);

  return result;
}

}  // namespace industrial_calibration
