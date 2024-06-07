#include <industrial_calibration/optimizations/camera_intrinsic.h>
#include <industrial_calibration/optimizations/camera_intrinsic_cost.h>
#include <industrial_calibration/optimizations/ceres_math_utilities.h>
#include <industrial_calibration/optimizations/covariance_analysis.h>
#include <industrial_calibration/optimizations/pnp.h>
#include <industrial_calibration/core/exceptions.h>
#include <industrial_calibration/core/serialization.h>

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

  if (!result.converged) throw ICalException("Unable to solve PnP sub-problem");

  return poseEigenToCal(result.camera_to_target);
}

static Pose6d guessInitialPose()
{
  Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
  guess = guess * Eigen::Translation3d(0, 0, 0.5) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  return poseEigenToCal(guess);
}

std::ostream& operator<<(std::ostream& stream, const CameraIntrinsicResult& result)
{
  stream << "Optimization " << (result.converged ? "converged" : "did not converge") << "\n"
         << "Initial cost per observation (pixels): " << std::sqrt(result.initial_cost_per_obs) << "\n"
         << "Final cost per observation (pixels): " << std::sqrt(result.final_cost_per_obs);
  if (result.converged)
  {
    stream << "\n"
           << result.intrinsics << "\n"
           << "Distortion:\n\tk1 = " << result.distortions[0] << "\n\tk2 = " << result.distortions[1]
           << "\n\tp1 = " << result.distortions[2] << "\n\tp2 = " << result.distortions[3]
           << "\n\tk3 = " << result.distortions[4];
  }
  return stream;
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
    if (params.use_extrinsic_guesses)
      internal_poses[i] = poseEigenToCal(params.extrinsic_guesses.at(i));
    else
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

  // Add constraints on the lower bounds of the focal lengths and camera center
  problem.SetParameterLowerBound(internal_intrinsics_data.data(), 0, 0.0);
  problem.SetParameterLowerBound(internal_intrinsics_data.data(), 1, 0.0);
  problem.SetParameterLowerBound(internal_intrinsics_data.data(), 2, 0.0);
  problem.SetParameterLowerBound(internal_intrinsics_data.data(), 3, 0.0);

  std::vector<const double*> param_blocks;
  std::map<const double*, std::vector<std::string>> param_labels;

  param_blocks.emplace_back(internal_intrinsics_data.data());
  param_labels[internal_intrinsics_data.data()] =
      std::vector<std::string>(params.labels_intrinsic_params.begin(), params.labels_intrinsic_params.end());

  for (std::size_t i = 0; i < internal_poses.size(); i++)
  {
    param_blocks.emplace_back(internal_poses[i].values.data());
    std::vector<std::string> labels;
    // compose labels poseN_x, etc.
    for (auto label_extr : params.labels_isometry3d)
    {
      labels.push_back(params.label_extr + std::to_string(i) + "_" + label_extr);
    }
    param_labels[internal_poses[i].values.data()] = labels;
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

  result.target_transforms.reserve(internal_poses.size());
  for (const Pose6d& pose : internal_poses)
    result.target_transforms.push_back(poseCalToEigen(pose));

  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;

  try
  {
    std::map<const double*, std::vector<int>> param_masks;
    ceres::Covariance::Options cov_options = DefaultCovarianceOptions();
    cov_options.null_space_rank = -1;  // automatically drop terms below min_reciprocal_condition_number
    result.covariance = computeCovariance(problem, param_blocks, param_labels);
  }
  catch (const ICalException& ex)
  {
  }

  return result;
}

}  // namespace industrial_calibration

namespace YAML
{
Node convert<CameraIntrinsicProblem>::encode(const CameraIntrinsicProblem& rhs)
{
  Node node;

  node["intrisics_guess"] = rhs.intrinsics_guess;
  node["image_observations"] = rhs.image_observations;
  node["extrinsic_guesses"] = rhs.extrinsic_guesses;
  node["use_extrinsic_guesses"] = rhs.use_extrinsic_guesses;

  return node;
}

bool convert<CameraIntrinsicProblem>::decode(const Node& node, CameraIntrinsicProblem& rhs)
{
  rhs.intrinsics_guess = getMember<decltype(rhs.intrinsics_guess)>(node, "intrinsics_guess");
  rhs.image_observations = getMember<decltype(rhs.image_observations)>(node, "image_observations");
  rhs.extrinsic_guesses = getMember<decltype(rhs.extrinsic_guesses)>(node, "extrinsic_guesses");
  rhs.use_extrinsic_guesses = getMember<decltype(rhs.use_extrinsic_guesses)>(node, "use_extrinsic_guesses");

  return true;
}

Node convert<CameraIntrinsicResult>::encode(const CameraIntrinsicResult& rhs)
{
  Node node;

  node["converged"] = rhs.converged;
  node["initial_cost_per_obs"] = rhs.initial_cost_per_obs;
  node["final_cost_per_obs"] = rhs.final_cost_per_obs;
  node["intrinsics"] = rhs.intrinsics;
  node["distortions"] = rhs.distortions;
  node["target_transforms"] = rhs.target_transforms;

  return node;
}

bool convert<CameraIntrinsicResult>::decode(const YAML::Node& node, CameraIntrinsicResult& rhs)
{
  rhs.converged = getMember<decltype(rhs.converged)>(node, "converged");
  rhs.initial_cost_per_obs = getMember<decltype(rhs.initial_cost_per_obs)>(node, "initial_cost_per_obs");
  rhs.final_cost_per_obs = getMember<decltype(rhs.final_cost_per_obs)>(node, "final_cost_per_obs");
  rhs.intrinsics = getMember<decltype(rhs.intrinsics)>(node, "intrinsics");
  rhs.distortions = getMember<decltype(rhs.distortions)>(node, "distortions");
  rhs.target_transforms = getMember<decltype(rhs.target_transforms)>(node, "target_transforms");

  return true;
}

}  // namespace YAML
