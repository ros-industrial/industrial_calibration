#include <industrial_calibration/optimizations/extrinsic_hand_eye.h>
#include <industrial_calibration/optimizations/extrinsic_hand_eye_cost.h>
#include <industrial_calibration/optimizations/ceres_math_utilities.h>
#include <industrial_calibration/optimizations/covariance_analysis.h>
#include <industrial_calibration/core/types.h>
#include <industrial_calibration/core/exceptions.h>
#include <industrial_calibration/core/serialization.h>

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

std::ostream& operator<<(std::ostream& stream, const ExtrinsicHandEyeResult& result)
{
  stream << "Optimization " << (result.converged ? "converged" : "did not converge") << "\n\n";
  stream << "Initial cost per observation: " << std::sqrt(result.initial_cost_per_obs) << "\n";
  stream << "Final cost per observation: " << std::sqrt(result.final_cost_per_obs);

  if (result.converged)
  {
    std::stringstream ss;
    ss << "Camera mount to camera transform\n";
    writeTransform(ss, result.camera_mount_to_camera);
    ss << "\nTarget mount to target transform\n";
    writeTransform(ss, result.target_mount_to_target);
    stream << "\n\n" << ss.str();
  }

  return stream;
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

  std::map<const double*, std::vector<std::string>> param_labels;
  param_labels[internal_camera_to_wrist.values.data()] = labels_camera_mount_to_camera;
  param_labels[internal_base_to_target.values.data()] = labels_target_mount_to_target;

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

  std::map<const double*, std::vector<std::string>> param_labels;
  param_labels[internal_camera_to_wrist.values.data()] = labels_camera_mount_to_camera;
  param_labels[internal_base_to_target.values.data()] = labels_target_mount_to_target;

  // Optionally compute covariance
  try
  {
    std::map<const double*, std::vector<int>> param_masks;
    ceres::Covariance::Options cov_options = DefaultCovarianceOptions();
    cov_options.null_space_rank = -1;  // automatically drop terms below min_reciprocal_condition_number
    result.covariance = computeCovariance(problem, param_labels, param_masks, cov_options);
  }
  catch (const ICalException& ex)
  {
  }

  return result;
}

}  // namespace industrial_calibration

using namespace industrial_calibration;

// @cond
namespace YAML
{
Node convert<ExtrinsicHandEyeProblem2D3D>::encode(const ExtrinsicHandEyeProblem2D3D& rhs)
{
  Node node;

  node["intr"] = rhs.intr;
  node["camera_mount_to_camera_guess"] = rhs.camera_mount_to_camera_guess;
  node["target_mount_to_target_guess"] = rhs.target_mount_to_target_guess;
  node["observations"] = rhs.observations;

  return node;
}

bool convert<ExtrinsicHandEyeProblem2D3D>::decode(const Node& node, ExtrinsicHandEyeProblem2D3D& rhs)
{
  rhs.intr = getMember<decltype(rhs.intr)>(node, "intr");
  rhs.camera_mount_to_camera_guess = getMember<decltype(rhs.camera_mount_to_camera_guess)>(node, "camera_mount_to_"
                                                                                                 "camera_guess");
  rhs.target_mount_to_target_guess = getMember<decltype(rhs.target_mount_to_target_guess)>(node, "target_mount_to_"
                                                                                                 "target_guess");
  rhs.observations = getMember<decltype(rhs.observations)>(node, "observations");

  return true;
}

Node convert<ExtrinsicHandEyeProblem3D3D>::encode(const ExtrinsicHandEyeProblem3D3D& rhs)
{
  Node node;

  node["camera_mount_to_camera_guess"] = rhs.camera_mount_to_camera_guess;
  node["target_mount_to_target_guess"] = rhs.target_mount_to_target_guess;
  node["observations"] = rhs.observations;

  return node;
}

bool convert<ExtrinsicHandEyeProblem3D3D>::decode(const Node& node, ExtrinsicHandEyeProblem3D3D& rhs)
{
  rhs.camera_mount_to_camera_guess = getMember<decltype(rhs.camera_mount_to_camera_guess)>(node, "camera_mount_to_"
                                                                                                 "camera_guess");
  rhs.target_mount_to_target_guess = getMember<decltype(rhs.target_mount_to_target_guess)>(node, "target_mount_to_"
                                                                                                 "target_guess");
  rhs.observations = getMember<decltype(rhs.observations)>(node, "observations");

  return true;
}

Node convert<ExtrinsicHandEyeResult>::encode(const ExtrinsicHandEyeResult& rhs)
{
  Node node;
  node["converged"] = rhs.converged;
  node["initial_cost_per_obs"] = rhs.initial_cost_per_obs;
  node["final_cost_per_obs"] = rhs.final_cost_per_obs;

  node["target_mount_to_target"] = rhs.target_mount_to_target;
  node["camera_mount_to_camera"] = rhs.camera_mount_to_camera;

  // Serialize transform orientation in RPY for convenience
  node["target_mount_to_target_rpy"] =
      Eigen::Vector3d(rhs.target_mount_to_target.rotation().eulerAngles(2, 1, 0).reverse());
  node["camera_mount_to_camera_rpy"] =
      Eigen::Vector3d(rhs.camera_mount_to_camera.rotation().eulerAngles(2, 1, 0).reverse());

  // [Deprecated] serialization of translations
  node["target_mount_to_target_pos"] = Eigen::Vector3d(rhs.target_mount_to_target.translation());
  node["camera_mount_to_camera_pos"] = Eigen::Vector3d(rhs.camera_mount_to_camera.translation());

  return node;
}

bool convert<ExtrinsicHandEyeResult>::decode(const YAML::Node& node, ExtrinsicHandEyeResult& rhs)
{
  rhs.converged = getMember<decltype(rhs.converged)>(node, "converged");
  rhs.initial_cost_per_obs = getMember<decltype(rhs.initial_cost_per_obs)>(node, "initial_cost_per_obs");
  rhs.final_cost_per_obs = getMember<decltype(rhs.final_cost_per_obs)>(node, "final_cost_per_obs");

  if (node["target_mount_to_target"])
  {
    rhs.target_mount_to_target = getMember<Eigen::Isometry3d>(node, "target_mount_to_target");
  }
  else
  {
    // [Deprecated] parsing of transform from position and RPY
    Eigen::Vector3d target_mount_to_target_pos = getMember<Eigen::Vector3d>(node, "target_mount_to_target_pos");
    Eigen::Vector3d target_mount_to_target_rpy = getMember<Eigen::Vector3d>(node, "target_mount_to_target_rpy");
    rhs.target_mount_to_target = toIsometry<double>(target_mount_to_target_pos, target_mount_to_target_rpy.reverse());
  }

  if (node["camera_mount_to_camera"])
  {
    rhs.camera_mount_to_camera = getMember<Eigen::Isometry3d>(node, "camera_mount_to_camera");
  }
  else
  {
    // [Deprecated] parsing of transform from position and RPY
    Eigen::Vector3d camera_mount_to_camera_pos = getMember<Eigen::Vector3d>(node, "camera_mount_to_camera_pos");
    Eigen::Vector3d camera_mount_to_camera_rpy = getMember<Eigen::Vector3d>(node, "camera_mount_to_camera_rpy");
    rhs.camera_mount_to_camera = toIsometry<double>(camera_mount_to_camera_pos, camera_mount_to_camera_rpy.reverse());
  }

  return true;
}

}  // namespace YAML
// @endcond
