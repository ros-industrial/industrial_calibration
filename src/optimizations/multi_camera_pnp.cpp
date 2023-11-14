#include <ical_core/optimizations/multi_camera_pnp.h>
#include <ical_core/cost_functions/multi_camera_pnp.h>
#include <ical_core/optimizations/utils/ceres_math_utilities.h>
#include <ical_core/types.h>

#include <ceres/ceres.h>

namespace industrial_calibration
{
MultiCameraPnPResult optimize(const MultiCameraPnPProblem& params)
{
  Pose6d internal_base_to_target = poseEigenToCal(params.base_to_target_guess);

  ceres::Problem problem;

  for (std::size_t c = 0; c < params.base_to_camera.size(); ++c)  // For each camera
  {
    for (std::size_t i = 0; i < params.image_observations[c].size(); ++i)  // For each 3D point seen in the 2D image
    {
      // Define
      const auto& img_obs = params.image_observations[c][i].in_image;
      const auto& point_in_target = params.image_observations[c][i].in_target;
      const auto& base_to_camera = params.base_to_camera[c];
      const auto& intr = params.intr[c];

      // Allocate Ceres data structures - ownership is taken by the ceres
      // Problem data structure
      auto* cost_fn = new MultiCameraPnPCost(img_obs, intr, base_to_camera.inverse(), point_in_target);

      auto* cost_block = new ceres::AutoDiffCostFunction<MultiCameraPnPCost, 2, 6>(cost_fn);

      problem.AddResidualBlock(cost_block, NULL, internal_base_to_target.values.data());
    }
  }  // end for each camera

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  MultiCameraPnPResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.base_to_target = poseCalToEigen(internal_base_to_target);
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;
  return result;
}

}  // namespace industrial_calibration
