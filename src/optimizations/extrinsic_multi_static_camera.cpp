#include <industrial_calibration/optimizations/extrinsic_multi_static_camera.h>
#include <industrial_calibration/cost_functions/extrinsic_multi_static_camera.h>
#include <industrial_calibration/optimizations/utils/ceres_math_utilities.h>
#include <industrial_calibration/types.h>
#include <industrial_calibration/optimizations/analysis/covariance_analysis.h>

#include <ceres/ceres.h>
#include <iostream>

namespace industrial_calibration
{
ExtrinsicMultiStaticCameraMovingTargetResult optimize(const ExtrinsicMultiStaticCameraMovingTargetProblem& params)
{
  Pose6d internal_wrist_to_target = poseEigenToCal(params.wrist_to_target_guess);

  std::vector<Pose6d> internal_camera_to_base;
  internal_camera_to_base.resize(params.base_to_camera_guess.size());

  ceres::Problem problem;

  for (std::size_t c = 0; c < params.base_to_camera_guess.size(); ++c)  // For each camera
  {
    assert(params.image_observations[c].size() == params.wrist_poses[c].size());
    internal_camera_to_base[c] = poseEigenToCal(params.base_to_camera_guess[c].inverse());
    for (std::size_t i = 0; i < params.wrist_poses[c].size(); ++i)  // For each wrist pose / image set
    {
      for (std::size_t j = 0; j < params.image_observations[c][i].size();
           ++j)  // For each 3D point seen in the 2D image
      {
        // Define
        const auto& img_obs = params.image_observations[c][i][j].in_image;
        const auto& point_in_target = params.image_observations[c][i][j].in_target;
        const auto wrist_to_base = params.wrist_poses[c][i];  //.inverse();
        const auto& intr = params.intr[c];

        // Allocate Ceres data structures - ownership is taken by the ceres
        // Problem data structure
        auto* cost_fn = new ExtrinsicMultiStaticCameraCost(img_obs, intr, wrist_to_base, point_in_target);

        auto* cost_block = new ceres::AutoDiffCostFunction<ExtrinsicMultiStaticCameraCost, 2, 6, 6>(cost_fn);

        problem.AddResidualBlock(cost_block, NULL, internal_camera_to_base[c].values.data(),
                                 internal_wrist_to_target.values.data());
      }
    }  // for each wrist pose
  }    // end for each camera

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  ExtrinsicMultiStaticCameraMovingTargetResult result;
  result.base_to_camera.resize(params.base_to_camera_guess.size());
  result.converged = summary.termination_type == ceres::CONVERGENCE;

  for (std::size_t i = 0; i < params.base_to_camera_guess.size(); ++i)
    result.base_to_camera[i] = poseCalToEigen(internal_camera_to_base[i]).inverse();

  result.wrist_to_target = poseCalToEigen(internal_wrist_to_target);
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;

  // collect parameter blocks so that order is consistent with labels
  std::vector<const double*> param_blocks(internal_camera_to_base.size());
  for (std::size_t c = 0; c < internal_camera_to_base.size(); c++)
  {
    param_blocks[c] = internal_camera_to_base[c].values.data();
  }
  param_blocks.push_back(internal_wrist_to_target.values.data());

  std::map<const double*, std::vector<std::string>> param_labels;

  // compose labels "camera_mount_to_camera_x", etc.
  for (std::size_t index_camera = 0; index_camera < internal_camera_to_base.size(); index_camera++)
  {
    std::vector<std::string> labels_base_to_camera;
    for (auto label_isometry : params.labels_isometry3d)
    {
      labels_base_to_camera.emplace_back(params.label_base_to_camera + std::to_string(index_camera) + "_" +
                                         label_isometry);
    }
    param_labels[internal_camera_to_base[index_camera].values.data()] = labels_base_to_camera;
  }

  // compose labels "target_mount_to_target_x", etc.
  std::vector<std::string> labels_wrist_to_target;
  for (auto label_isometry : params.labels_isometry3d)
  {
    labels_wrist_to_target.emplace_back(params.label_wrist_to_target + "_" + label_isometry);
  }
  param_labels[internal_wrist_to_target.values.data()] = labels_wrist_to_target;

  result.covariance = computeCovariance(problem, param_blocks, param_labels);

  return result;
}

ExtrinsicMultiStaticCameraOnlyResult optimize(const ExtrinsicMultiStaticCameraOnlyProblem& params)
{
  std::vector<Pose6d> internal_base_to_target;
  std::vector<Pose6d> internal_camera_to_base;
  internal_camera_to_base.resize(params.base_to_camera_guess.size());
  internal_base_to_target.resize(params.base_to_target_guess.size());

  ceres::Problem problem;

  for (std::size_t i = 0; i < params.base_to_target_guess.size(); ++i)  // For each wrist pose / image set
  {
    internal_base_to_target[i] = poseEigenToCal(params.base_to_target_guess[i]);
    for (std::size_t c = 0; c < params.base_to_camera_guess.size(); ++c)  // For each camera
    {
      assert(params.image_observations[c].size() == params.base_to_target_guess.size());
      internal_camera_to_base[c] = poseEigenToCal(params.base_to_camera_guess[c].inverse());

      for (std::size_t j = 0; j < params.image_observations[c][i].size();
           ++j)  // For each 3D point seen in the 2D image
      {
        // Define
        const auto& img_obs = params.image_observations[c][i][j].in_image;
        const auto& point_in_target = params.image_observations[c][i][j].in_target;
        const auto& intr = params.intr[c];

        // Allocate Ceres data structures - ownership is taken by the ceres
        // Problem data structure
        if (params.fix_first_camera && (c == 0))
        {
          auto* cost_fn =
              new ExtrinsicMultiStaticFixedCameraCost(img_obs, intr, params.base_to_camera_guess[c], point_in_target);

          auto* cost_block = new ceres::AutoDiffCostFunction<ExtrinsicMultiStaticFixedCameraCost, 2, 6>(cost_fn);

          problem.AddResidualBlock(cost_block, NULL, internal_base_to_target[i].values.data());
        }
        else
        {
          auto* cost_fn = new ExtrinsicMultiStaticFreeCameraCost(img_obs, intr, point_in_target);

          auto* cost_block = new ceres::AutoDiffCostFunction<ExtrinsicMultiStaticFreeCameraCost, 2, 6, 6>(cost_fn);

          problem.AddResidualBlock(cost_block, NULL, internal_camera_to_base[c].values.data(),
                                   internal_base_to_target[i].values.data());
        }
      }
    }  // for each wrist pose
  }    // end for each camera

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  ExtrinsicMultiStaticCameraOnlyResult result;
  result.base_to_camera.resize(params.base_to_camera_guess.size());
  result.base_to_target.resize(params.base_to_target_guess.size());
  result.converged = summary.termination_type == ceres::CONVERGENCE;

  for (std::size_t i = 0; i < params.base_to_camera_guess.size(); ++i)
    result.base_to_camera[i] = poseCalToEigen(internal_camera_to_base[i]).inverse();

  for (std::size_t i = 0; i < params.base_to_target_guess.size(); ++i)
    result.base_to_target[i] = poseCalToEigen(internal_base_to_target[i]);

  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;

  return result;
}

ExtrinsicMultiStaticCameraMovingTargetWristOnlyResult
optimize(const ExtrinsicMultiStaticCameraMovingTargetWristOnlyProblem& params)
{
  Pose6d internal_wrist_to_target = poseEigenToCal(params.wrist_to_target_guess);

  Pose6d internal_camera_to_base_correction = poseEigenToCal(Eigen::Isometry3d::Identity());

  ceres::Problem problem;

  for (std::size_t c = 0; c < params.base_to_camera_guess.size(); ++c)  // For each camera
  {
    assert(params.image_observations[c].size() == params.wrist_poses.size());
    for (std::size_t i = 0; i < params.wrist_poses.size(); ++i)  // For each wrist pose / image set
    {
      for (std::size_t j = 0; j < params.image_observations[c][i].size();
           ++j)  // For each 3D point seen in the 2D image
      {
        // Define
        const auto& img_obs = params.image_observations[c][i][j].in_image;
        const auto& point_in_target = params.image_observations[c][i][j].in_target;
        const auto& base_to_wrist = params.wrist_poses[i];
        const auto& base_to_camera_orig = params.base_to_camera_guess[c];
        const auto& intr = params.intr[c];

        // Allocate Ceres data structures - ownership is taken by the ceres
        // Problem data structure
        auto* cost_fn = new ExtrinsicMultiStaticCameraWristOnlyCost(img_obs, intr, base_to_wrist, base_to_camera_orig,
                                                                    point_in_target);

        auto* cost_block = new ceres::AutoDiffCostFunction<ExtrinsicMultiStaticCameraWristOnlyCost, 2, 6, 6>(cost_fn);

        problem.AddResidualBlock(cost_block, NULL, internal_camera_to_base_correction.values.data(),
                                 internal_wrist_to_target.values.data());
      }
    }  // for each wrist pose
  }    // end for each camera

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  ExtrinsicMultiStaticCameraMovingTargetWristOnlyResult result;
  result.base_to_camera.resize(params.base_to_camera_guess.size());
  result.converged = summary.termination_type == ceres::CONVERGENCE;

  Eigen::Isometry3d base_to_camera_correction = poseCalToEigen(internal_camera_to_base_correction).inverse();
  for (std::size_t i = 0; i < params.base_to_camera_guess.size(); ++i)
    result.base_to_camera[i] = base_to_camera_correction * params.base_to_camera_guess[i];

  result.wrist_to_target = poseCalToEigen(internal_wrist_to_target);
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;
  return result;
}

}  // namespace industrial_calibration
