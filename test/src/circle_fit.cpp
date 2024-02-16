#include <industrial_calibration_tests/circle_fit.h>
#include <industrial_calibration/optimizations/covariance_analysis.h>

#include <ceres/ceres.h>

namespace industrial_calibration
{
CircleFitResult optimize(const CircleFitProblem& params)
{
  double x = params.x_center_initial;
  double y = params.y_center_initial;
  double r = params.radius_initial;

  double r_sqrt = sqrt(r);

  std::vector<double> circle_params(3, 0.0);
  circle_params[0] = x;
  circle_params[1] = y;
  circle_params[2] = r_sqrt;

  CircleFitResult result;

  ceres::Problem problem;

  ceres::LossFunction* loss_fn = nullptr;

  for (auto obs : params.observations)
  {
    auto* cost_fn = new CircleDistCost(obs.x(), obs.y());

    auto* cost_block = new ceres::AutoDiffCostFunction<CircleDistCost, 1, 3>(cost_fn);
    problem.AddResidualBlock(cost_block, loss_fn, circle_params.data());
  }

  ceres::Solver::Options options;
  options.max_num_iterations = 500;
  options.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << std::endl;

  std::map<const double*, std::vector<std::string>> param_block_labels;
  param_block_labels[circle_params.data()] = params.labels;

  result.converged = summary.termination_type == ceres::TerminationType::CONVERGENCE;
  result.x_center = circle_params[0];
  result.y_center = circle_params[1];
  result.radius = pow(circle_params[2], 2);
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;
  result.covariance = computeCovariance(problem, param_block_labels);

  return result;
}

}  // namespace industrial_calibration
