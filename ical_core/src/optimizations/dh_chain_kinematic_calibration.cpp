#include <ical_core/optimizations/dh_chain_kinematic_calibration.h>
#include <ical_core/cost_functions/dh_chain_kinematic_calibration.h>
#include <ical_core/ceres_math_utilities.h>
#include <ical_core/optimizations/utils/covariance_analysis.h>
#include <ical_core/optimizations/utils/maximum_likelihood.h>
#include <ical_core/optimizations/utils/local_parameterization.h>

#include <ceres/ceres.h>

namespace industrial_calibration
{
Eigen::Isometry3d createTransform(const Eigen::Vector3d& t, const Eigen::Vector3d& aa)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity() * Eigen::Translation3d(t);

  double aa_norm = aa.norm();
  if (aa_norm > std::numeric_limits<double>::epsilon())
  {
    result *= Eigen::AngleAxisd(aa_norm, aa.normalized());
  }

  return result;
}

KinematicCalibrationResult optimize(const KinematicCalibrationProblem2D3D& params)
{
  // Initialize the optimization variables
  // Camera mount to camera (cm_to_c) unnormalized angle axis and translation
  Eigen::Vector3d t_cm_to_c(params.camera_mount_to_camera_guess.translation());
  Eigen::AngleAxisd rot_cm_to_c(params.camera_mount_to_camera_guess.rotation());
  Eigen::Vector3d aa_cm_to_c(rot_cm_to_c.angle() * rot_cm_to_c.axis());

  std::array<std::string, 3> t_cm_to_c_labels({ params.label_camera_mount_to_camera + "_x",
                                                params.label_camera_mount_to_camera + "_y",
                                                params.label_camera_mount_to_camera + "_z" });
  std::array<std::string, 3> aa_cm_to_c_labels({ params.label_camera_mount_to_camera + "_rx",
                                                 params.label_camera_mount_to_camera + "_ry",
                                                 params.label_camera_mount_to_camera + "_rz" });

  // Target mount to target (cm_to_c) quaternion and translation
  Eigen::Vector3d t_tm_to_t(params.target_mount_to_target_guess.translation());
  Eigen::AngleAxisd rot_tm_to_t(params.target_mount_to_target_guess.rotation());
  Eigen::Vector3d aa_tm_to_t(rot_tm_to_t.angle() * rot_tm_to_t.axis());

  std::array<std::string, 3> t_tm_to_t_labels({ params.label_target_mount_to_target + "_x",
                                                params.label_target_mount_to_target + "_y",
                                                params.label_target_mount_to_target + "_z" });
  std::array<std::string, 3> aa_tm_to_t_labels({ params.label_target_mount_to_target + "_rx",
                                                 params.label_target_mount_to_target + "_ry",
                                                 params.label_target_mount_to_target + "_rz" });

  // Camera chain base to target_chain_base (ccb_to_tcb) quaternion and translation
  Eigen::Vector3d t_ccb_to_tcb(params.camera_base_to_target_base_guess.translation());
  Eigen::AngleAxisd rot_ccb_to_tcb(params.camera_base_to_target_base_guess.rotation());
  Eigen::Vector3d aa_ccb_to_tcb(rot_ccb_to_tcb.angle() * rot_ccb_to_tcb.axis());

  std::array<std::string, 3> t_ccb_to_tcb_labels({ params.label_camera_base_to_target + "_x",
                                                   params.label_camera_base_to_target + "_y",
                                                   params.label_camera_base_to_target + "_z" });
  std::array<std::string, 3> aa_ccb_to_tcb_labels({ params.label_camera_base_to_target + "_rx",
                                                    params.label_camera_base_to_target + "_ry",
                                                    params.label_camera_base_to_target + "_rz" });

  // Create containers for the kinematic chain DH offsets
  // Ceres will not work with parameter blocks of size zero, so create a dummy set of DH offsets for chains with DoF ==
  // 0
  Eigen::MatrixX4d camera_chain_dh_offsets;
  std::vector<std::array<std::string, 4>> camera_chain_param_labels;
  if (params.camera_chain.dof() != 0)
  {
    camera_chain_dh_offsets = Eigen::MatrixX4d::Zero(params.camera_chain.dof(), 4);
    camera_chain_param_labels = params.camera_chain.getParamLabels();
  }
  else
  {
    camera_chain_dh_offsets = Eigen::MatrixX4d::Zero(1, 4);
    camera_chain_param_labels = { { "camera_chain_placeholder_d", "camera_chain_placeholder_theta",
                                    "camera_chain_placeholder_r", "camera_chain_placeholder_alpha" } };
  }

  Eigen::MatrixX4d target_chain_dh_offsets;
  std::vector<std::array<std::string, 4>> target_chain_param_labels;
  if (params.target_chain.dof() != 0)
  {
    target_chain_dh_offsets = Eigen::MatrixX4d::Zero(params.target_chain.dof(), 4);
    target_chain_param_labels = params.target_chain.getParamLabels();
  }
  else
  {
    target_chain_dh_offsets = Eigen::MatrixX4d::Zero(1, 4);
    target_chain_param_labels = { { "target_chain_placeholder_d", "target_chain_placeholder_theta",
                                    "target_chain_placeholder_r", "target_chain_placeholder_alpha" } };
  }

  // Create a vector of the pointers to the optimization variables in the order that the cost function expects them
  std::vector<double*> parameters =
      DualDHChain2D3DCost::constructParameters(camera_chain_dh_offsets, target_chain_dh_offsets, t_cm_to_c, aa_cm_to_c,
                                               t_tm_to_t, aa_tm_to_t, t_ccb_to_tcb, aa_ccb_to_tcb);

  std::vector<std::vector<std::string>> param_labels = DualDHChain2D3DCost::constructParameterLabels(
      camera_chain_param_labels, target_chain_param_labels, t_cm_to_c_labels, aa_cm_to_c_labels, t_tm_to_t_labels,
      aa_tm_to_t_labels, t_ccb_to_tcb_labels, aa_ccb_to_tcb_labels);

  // Set up the problem
  ceres::Problem problem;

  for (const auto& observation : params.observations)
  {
    for (const auto& correspondence : observation.correspondence_set)
    {
      // Allocate Ceres data structures - ownership is taken by the ceres
      // Problem data structure
      auto* cost_fn = new DualDHChain2D3DCost(correspondence.in_image, correspondence.in_target, params.intr,
                                              params.camera_chain, params.target_chain, observation.camera_chain_joints,
                                              observation.target_chain_joints);

      auto* cost_block = new ceres::DynamicAutoDiffCostFunction<DualDHChain2D3DCost>(cost_fn);

      // Add the optimization parameters
      // DH parameters for both kinematic chains
      cost_block->AddParameterBlock(camera_chain_dh_offsets.size());
      cost_block->AddParameterBlock(target_chain_dh_offsets.size());

      // Camera mount to camera transform
      cost_block->AddParameterBlock(3);
      cost_block->AddParameterBlock(3);

      // Target mount to target transform
      cost_block->AddParameterBlock(3);
      cost_block->AddParameterBlock(3);

      // Camera kinematic chain to target kinematic chain transform
      cost_block->AddParameterBlock(3);
      cost_block->AddParameterBlock(3);

      // Residual error
      cost_block->SetNumResiduals(2);

      // Add the residual block to the problem
      problem.AddResidualBlock(cost_block, nullptr, parameters);
    }
  }

  // Tell the optimization to keep constant the dummy DH offsets that might have been added to the 0-DoF chains
  if (params.camera_chain.dof() == 0) problem.SetParameterBlockConstant(camera_chain_dh_offsets.data());
  if (params.target_chain.dof() == 0) problem.SetParameterBlockConstant(target_chain_dh_offsets.data());

  // Add subset parameterization to mask variables that shouldn't be optimized
  std::array<double*, 8> tmp;
  std::copy_n(parameters.begin(), tmp.size(), tmp.begin());
  addSubsetParameterization(problem, params.mask, tmp);

  // Add a cost to drive the camera chain DH parameters towards an expected mean
  if (params.camera_chain.dof() != 0 && !problem.IsParameterBlockConstant(parameters[0]))
  {
    Eigen::ArrayXXd mean(Eigen::ArrayXXd::Zero(camera_chain_dh_offsets.rows(), camera_chain_dh_offsets.cols()));

    Eigen::ArrayXXd stdev(Eigen::ArrayXXd::Constant(camera_chain_dh_offsets.rows(), camera_chain_dh_offsets.cols(),
                                                    params.camera_chain_offset_stdev));

    auto* fn = new MaximumLikelihood(mean, stdev);
    auto* cost_block = new ceres::DynamicAutoDiffCostFunction<MaximumLikelihood>(fn);
    cost_block->AddParameterBlock(camera_chain_dh_offsets.size());
    cost_block->SetNumResiduals(camera_chain_dh_offsets.size());

    problem.AddResidualBlock(cost_block, nullptr, camera_chain_dh_offsets.data());
  }

  // Add a cost to drive the target chain DH parameters towards an expected mean
  if (params.target_chain.dof() != 0 && !problem.IsParameterBlockConstant(parameters[1]))
  {
    Eigen::ArrayXXd mean(Eigen::ArrayXXd::Zero(target_chain_dh_offsets.rows(), target_chain_dh_offsets.cols()));

    Eigen::ArrayXXd stdev(Eigen::ArrayXXd::Constant(target_chain_dh_offsets.rows(), target_chain_dh_offsets.cols(),
                                                    params.target_chain_offset_stdev));

    auto* fn = new MaximumLikelihood(mean, stdev);
    auto* cost_block = new ceres::DynamicAutoDiffCostFunction<MaximumLikelihood>(fn);
    cost_block->AddParameterBlock(target_chain_dh_offsets.size());
    cost_block->SetNumResiduals(target_chain_dh_offsets.size());

    problem.AddResidualBlock(cost_block, nullptr, target_chain_dh_offsets.data());
  }

  // Setup the Ceres optimization parameters
  ceres::Solver::Options options;
  options.max_num_iterations = 150;
  options.num_threads = 4;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;

  // Solve the optimization
  ceres::Solve(options, &problem, &summary);

  // Report and save the results
  KinematicCalibrationResult result;
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;

  // Save the transforms
  result.camera_mount_to_camera = createTransform(t_cm_to_c, aa_cm_to_c);
  result.target_mount_to_target = createTransform(t_tm_to_t, aa_tm_to_t);
  result.camera_base_to_target_base = createTransform(t_ccb_to_tcb, aa_ccb_to_tcb);

  // Save the DH parameter offsets
  if (params.camera_chain.dof() == 0)
    result.camera_chain_dh_offsets = Eigen::MatrixX4d::Zero(params.camera_chain.dof(), 4);
  else
    result.camera_chain_dh_offsets = camera_chain_dh_offsets;

  if (params.target_chain.dof() == 0)
    result.target_chain_dh_offsets = Eigen::MatrixX4d::Zero(params.target_chain.dof(), 4);
  else
    result.target_chain_dh_offsets = target_chain_dh_offsets;

  ceres::Covariance::Options cov_options = DefaultCovarianceOptions();
  cov_options.null_space_rank = -1;  // automatically drop terms below min_reciprocal_condition_number
  result.covariance = computeCovariance(problem, std::vector<const double*>(parameters.begin(), parameters.end()),
                                        param_labels, cov_options);

  return result;
}

KinematicCalibrationResult optimize(const KinematicCalibrationProblemPose6D& params, const double orientation_weight,
                                    const ceres::Solver::Options& options)
{
  // Initialize the optimization variables
  // Camera mount to camera (cm_to_c) quaternion and translation
  Eigen::Vector3d t_cm_to_c(params.camera_mount_to_camera_guess.translation());
  Eigen::AngleAxisd rot_cm_to_c(params.camera_mount_to_camera_guess.rotation());
  Eigen::Vector3d aa_cm_to_c(rot_cm_to_c.angle() * rot_cm_to_c.axis());

  std::array<std::string, 3> t_cm_to_c_labels({ params.label_camera_mount_to_camera + "_x",
                                                params.label_camera_mount_to_camera + "_y",
                                                params.label_camera_mount_to_camera + "_z" });
  std::array<std::string, 3> aa_cm_to_c_labels({ params.label_camera_mount_to_camera + "_rx",
                                                 params.label_camera_mount_to_camera + "_ry",
                                                 params.label_camera_mount_to_camera + "_rz" });

  // Target mount to target (tm_to_t) quaternion and translation
  Eigen::Vector3d t_tm_to_t(params.target_mount_to_target_guess.translation());
  Eigen::AngleAxisd rot_tm_to_t(params.target_mount_to_target_guess.rotation());
  Eigen::Vector3d aa_tm_to_t(rot_tm_to_t.angle() * rot_tm_to_t.axis());

  std::array<std::string, 3> t_tm_to_t_labels({ params.label_target_mount_to_target + "_x",
                                                params.label_target_mount_to_target + "_y",
                                                params.label_target_mount_to_target + "_z" });
  std::array<std::string, 3> aa_tm_to_t_labels({ params.label_target_mount_to_target + "_rx",
                                                 params.label_target_mount_to_target + "_ry",
                                                 params.label_target_mount_to_target + "_rz" });

  // Camera chain base to target_chain_base (ccb_to_tcb) quaternion and translation
  Eigen::Vector3d t_ccb_to_tcb(params.camera_base_to_target_base_guess.translation());
  Eigen::AngleAxisd rot_ccb_to_tcb(params.camera_base_to_target_base_guess.rotation());
  Eigen::Vector3d aa_ccb_to_tcb(rot_ccb_to_tcb.angle() * rot_ccb_to_tcb.axis());

  std::array<std::string, 3> t_ccb_to_tcb_labels({ params.label_camera_base_to_target + "_x",
                                                   params.label_camera_base_to_target + "_y",
                                                   params.label_camera_base_to_target + "_z" });
  std::array<std::string, 3> aa_ccb_to_tcb_labels({ params.label_camera_base_to_target + "_rx",
                                                    params.label_camera_base_to_target + "_ry",
                                                    params.label_camera_base_to_target + "_rz" });

  // Create containers for the kinematic chain DH offsets
  // Ceres will not work with parameter blocks of size zero, so create a dummy set of DH offsets for chains with DoF ==
  // 0
  Eigen::MatrixX4d camera_chain_dh_offsets;
  std::vector<std::array<std::string, 4>> camera_chain_param_labels;
  if (params.camera_chain.dof() != 0)
  {
    camera_chain_dh_offsets = Eigen::MatrixX4d::Zero(params.camera_chain.dof(), 4);
    camera_chain_param_labels = params.camera_chain.getParamLabels();
  }
  else
  {
    camera_chain_dh_offsets = Eigen::MatrixX4d::Zero(1, 4);
    camera_chain_param_labels = { { "camera_chain_placeholder_d", "camera_chain_placeholder_theta",
                                    "camera_chain_placeholder_r", "camera_chain_placeholder_alpha" } };
  }

  Eigen::MatrixX4d target_chain_dh_offsets;
  std::vector<std::array<std::string, 4>> target_chain_param_labels;
  if (params.target_chain.dof() != 0)
  {
    target_chain_dh_offsets = Eigen::MatrixX4d::Zero(params.target_chain.dof(), 4);
    target_chain_param_labels = params.target_chain.getParamLabels();
  }
  else
  {
    target_chain_dh_offsets = Eigen::MatrixX4d::Zero(1, 4);
    target_chain_param_labels = { { "target_chain_placeholder_d", "target_chain_placeholder_theta",
                                    "target_chain_placeholder_r", "target_chain_placeholder_alpha" } };
  }

  // Create a vector of the pointers to the optimization variables in the order that the cost function expects them
  std::vector<double*> parameters =
      DualDHChainMeasurementCost::constructParameters(camera_chain_dh_offsets, target_chain_dh_offsets, t_cm_to_c,
                                                      aa_cm_to_c, t_tm_to_t, aa_tm_to_t, t_ccb_to_tcb, aa_ccb_to_tcb);

  std::vector<std::vector<std::string>> param_labels = DualDHChainMeasurementCost::constructParameterLabels(
      camera_chain_param_labels, target_chain_param_labels, t_cm_to_c_labels, aa_cm_to_c_labels, t_tm_to_t_labels,
      aa_tm_to_t_labels, t_ccb_to_tcb_labels, aa_ccb_to_tcb_labels);

  // Set up the problem
  ceres::Problem problem;

  for (const auto& observation : params.observations)
  {
    // Allocate Ceres data structures - ownership is taken by the ceres
    // Problem data structure
    auto* cost_fn =
        new DualDHChainMeasurementCost(observation, params.camera_chain, params.target_chain, orientation_weight);

    auto* cost_block = new ceres::DynamicAutoDiffCostFunction<DualDHChainMeasurementCost>(cost_fn);

    // Add the optimization parameters
    // DH parameters for both kinematic chains
    cost_block->AddParameterBlock(camera_chain_dh_offsets.size());
    cost_block->AddParameterBlock(target_chain_dh_offsets.size());

    // Camera mount to camera transform
    cost_block->AddParameterBlock(3);
    cost_block->AddParameterBlock(3);

    // Target mount to target transform
    cost_block->AddParameterBlock(3);
    cost_block->AddParameterBlock(3);

    // Camera kinematic chain to target kinematic chain transform
    cost_block->AddParameterBlock(3);
    cost_block->AddParameterBlock(3);

    // Residual error
    cost_block->SetNumResiduals(4);

    // Add the residual block to the problem
    problem.AddResidualBlock(cost_block, nullptr, parameters);
  }

  // Tell the optimization to keep constant the dummy DH offsets that might have been added to the 0-DoF chains
  if (params.camera_chain.dof() == 0) problem.SetParameterBlockConstant(camera_chain_dh_offsets.data());
  if (params.target_chain.dof() == 0) problem.SetParameterBlockConstant(target_chain_dh_offsets.data());

  // Add subset parameterization to mask variables that shouldn't be optimized
  std::array<double*, 8> tmp;
  std::copy_n(parameters.begin(), tmp.size(), tmp.begin());
  addSubsetParameterization(problem, params.mask, tmp);

  // Add a cost to drive the camera chain DH parameters towards an expected mean
  if (params.camera_chain.dof() != 0 && !problem.IsParameterBlockConstant(parameters[0]))
  {
    Eigen::ArrayXXd mean(Eigen::ArrayXXd::Zero(camera_chain_dh_offsets.rows(), camera_chain_dh_offsets.cols()));

    Eigen::ArrayXXd stdev(Eigen::ArrayXXd::Constant(camera_chain_dh_offsets.rows(), camera_chain_dh_offsets.cols(),
                                                    params.camera_chain_offset_stdev));

    auto* fn = new MaximumLikelihood(mean, stdev);
    auto* cost_block = new ceres::DynamicAutoDiffCostFunction<MaximumLikelihood>(fn);
    cost_block->AddParameterBlock(camera_chain_dh_offsets.size());
    cost_block->SetNumResiduals(camera_chain_dh_offsets.size());

    problem.AddResidualBlock(cost_block, nullptr, camera_chain_dh_offsets.data());
  }

  // Add a cost to drive the target chain DH parameters towards an expected mean
  if (params.target_chain.dof() != 0 && !problem.IsParameterBlockConstant(parameters[1]))
  {
    Eigen::ArrayXXd mean(Eigen::ArrayXXd::Zero(target_chain_dh_offsets.rows(), target_chain_dh_offsets.cols()));

    Eigen::ArrayXXd stdev(Eigen::ArrayXXd::Constant(target_chain_dh_offsets.rows(), target_chain_dh_offsets.cols(),
                                                    params.target_chain_offset_stdev));

    auto* fn = new MaximumLikelihood(mean, stdev);
    auto* cost_block = new ceres::DynamicAutoDiffCostFunction<MaximumLikelihood>(fn);
    cost_block->AddParameterBlock(target_chain_dh_offsets.size());
    cost_block->SetNumResiduals(target_chain_dh_offsets.size());

    problem.AddResidualBlock(cost_block, nullptr, target_chain_dh_offsets.data());
  }

  // Solve the optimization
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Report and save the results
  KinematicCalibrationResult result;
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;

  // Save the transforms
  result.camera_mount_to_camera = createTransform(t_cm_to_c, aa_cm_to_c);
  result.target_mount_to_target = createTransform(t_tm_to_t, aa_tm_to_t);
  result.camera_base_to_target_base = createTransform(t_ccb_to_tcb, aa_ccb_to_tcb);

  // Save the DH parameter offsets
  if (params.camera_chain.dof() == 0)
    result.camera_chain_dh_offsets = Eigen::MatrixX4d::Zero(params.camera_chain.dof(), 4);
  else
    result.camera_chain_dh_offsets = camera_chain_dh_offsets;

  if (params.target_chain.dof() == 0)
    result.target_chain_dh_offsets = Eigen::MatrixX4d::Zero(params.target_chain.dof(), 4);
  else
    result.target_chain_dh_offsets = target_chain_dh_offsets;

  ceres::Covariance::Options cov_options = DefaultCovarianceOptions();
  cov_options.null_space_rank = -1;  // automatically drop terms below min_reciprocal_condition_number
  result.covariance = computeCovariance(problem, std::vector<const double*>(parameters.begin(), parameters.end()),
                                        param_labels, cov_options);

  return result;
}

}  // namespace industrial_calibration
