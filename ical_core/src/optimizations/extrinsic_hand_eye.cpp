#include <ical_core/optimizations/extrinsic_hand_eye.h>
#include <ical_core/cost_functions/extrinsic_hand_eye.h>
#include <ical_core/optimizations/utils/ceres_math_utilities.h>
#include <ical_core/types.h>
#include <ical_core/optimizations/utils/covariance_analysis.h>
#include <ical_core/exceptions.h>

#include <ceres/ceres.h>
#include <iostream>

namespace industrial_calibration
{
bool isPointVisible(const Pose6d& camera_to_camera_mount, const Pose6d& target_mount_to_target,
                    const ExtrinsicHandEye2D3DCost* cost_fn)
{
  const double* pose_camera_to_camera_mount = camera_to_camera_mount.values.data();
  const double* pose_target_mount_to_target = target_mount_to_target.values.data();

  double camera_point[3];
  cost_fn->getTargetPointInCamera(pose_camera_to_camera_mount, pose_target_mount_to_target, camera_point);

  // Return whether or not the projected point's Z value is greater than zero
  return camera_point[2] > 0.0;
}

ExtrinsicHandEyeResult optimize(const ExtrinsicHandEyeProblem2D3D& params)
{
  Pose6d internal_base_to_target = poseEigenToCal(params.target_mount_to_target_guess);
  Pose6d internal_camera_to_wrist = poseEigenToCal(params.camera_mount_to_camera_guess.inverse());

  ceres::Problem problem;

  for (const auto& observation : params.observations)
  {
    for (const auto& correspondence : observation.correspondence_set)
    {
      // Define
      const auto& img_obs = correspondence.in_image;
      const auto& point_in_target = correspondence.in_target;
      const auto wrist_to_base = observation.to_camera_mount.inverse();

      // Allocate Ceres data structures - ownership is taken by the ceres
      // Problem data structure
      auto* cost_fn = new ExtrinsicHandEye2D3DCost(img_obs, wrist_to_base, observation.to_target_mount, point_in_target,
                                                   params.intr);

      auto* cost_block = new ceres::AutoDiffCostFunction<ExtrinsicHandEye2D3DCost, 2, 6, 6>(cost_fn);

      // Check that the target feature in camera coordinates is visible by the camera
      // Target features that project behind the camera tend to prevent the optimization from converging
      if (!isPointVisible(internal_camera_to_wrist, internal_base_to_target, cost_fn))
      {
        throw ICalException("Projected target feature lies behind the image plane using the "
                            "current target mount and camera mount transform guesses. Try updating the initial "
                            "transform guesses to more accurately represent the problem");
      }

      problem.AddResidualBlock(cost_block, NULL, internal_camera_to_wrist.values.data(),
                               internal_base_to_target.values.data());
    }
  }

  ceres::Solver::Options options;
  options.max_num_iterations = 150;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  ExtrinsicHandEyeResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.target_mount_to_target = poseCalToEigen(internal_base_to_target);
  result.camera_mount_to_camera = poseCalToEigen(internal_camera_to_wrist).inverse();
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;

  // compose labels "camera_mount_to_camera_x", etc.
  std::vector<std::string> labels_camera_mount_to_camera;
  for (auto label_isometry : params.labels_isometry3d)
  {
    labels_camera_mount_to_camera.emplace_back(params.label_camera_mount_to_camera + "_" + label_isometry);
  }

  // compose labels "target_mount_to_target_x", etc.
  std::vector<std::string> labels_target_mount_to_target;
  for (auto label_isometry : params.labels_isometry3d)
  {
    labels_target_mount_to_target.emplace_back(params.label_target_mount_to_target + "_" + label_isometry);
  }

  std::vector<std::vector<std::string>> param_labels;
  param_labels.push_back(labels_camera_mount_to_camera);
  param_labels.push_back(labels_target_mount_to_target);

  result.covariance = computeCovariance(problem, param_labels);

  return result;
}

ExtrinsicHandEyeResult optimize(const ExtrinsicHandEyeProblem3D3D& params)
{
  Pose6d internal_base_to_target = poseEigenToCal(params.target_mount_to_target_guess);
  Pose6d internal_camera_to_wrist = poseEigenToCal(params.camera_mount_to_camera_guess.inverse());

  ceres::Problem problem;

  for (const auto& observation : params.observations)
  {
    for (const auto& correspondence : observation.correspondence_set)
    {
      // Define
      const auto& img_obs = correspondence.in_image;
      const auto& point_in_target = correspondence.in_target;
      const auto wrist_to_base = observation.to_camera_mount.inverse();

      // Allocate Ceres data structures - ownership is taken by the ceres
      // Problem data structure
      auto* cost_fn =
          new ExtrinsicHandEye3D3DCost(img_obs, wrist_to_base, observation.to_target_mount, point_in_target);

      auto* cost_block = new ceres::AutoDiffCostFunction<ExtrinsicHandEye3D3DCost, 3, 6, 6>(cost_fn);

      problem.AddResidualBlock(cost_block, NULL, internal_camera_to_wrist.values.data(),
                               internal_base_to_target.values.data());
    }
  }

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  ExtrinsicHandEyeResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.target_mount_to_target = poseCalToEigen(internal_base_to_target);
  result.camera_mount_to_camera = poseCalToEigen(internal_camera_to_wrist).inverse();
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;

  // compose labels "camera_mount_to_camera_x", etc.
  std::vector<std::string> labels_camera_mount_to_camera;
  for (auto label_isometry : params.labels_isometry3d)
  {
    labels_camera_mount_to_camera.emplace_back(params.label_camera_mount_to_camera + "_" + label_isometry);
  }

  // compose labels "target_mount_to_target_x", etc.
  std::vector<std::string> labels_target_mount_to_target;
  for (auto label_isometry : params.labels_isometry3d)
  {
    labels_target_mount_to_target.emplace_back(params.label_target_mount_to_target + "_" + label_isometry);
  }

  std::vector<std::vector<std::string>> param_labels;
  param_labels.push_back(labels_camera_mount_to_camera);
  param_labels.push_back(labels_target_mount_to_target);

  result.covariance = computeCovariance(problem, param_labels);

  return result;
}

}  // namespace industrial_calibration
