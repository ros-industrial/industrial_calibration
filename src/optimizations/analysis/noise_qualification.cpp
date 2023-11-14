#include <ical_core/optimizations/analysis/noise_qualification.h>
#include <ical_core/optimizations/pnp.h>
#include <ical_core/exceptions.h>

namespace industrial_calibration
{
PnPNoiseStat qualifyNoise2D(const std::vector<PnPProblem>& params)
{
  PnPNoiseStat output;
  std::size_t count = params.size();

  std::vector<Eigen::Isometry3d> solution_transforms;
  solution_transforms.reserve(count);

  std::vector<Eigen::Vector3d> translations;
  translations.reserve(count);

  std::vector<Eigen::Quaterniond> orientations;
  orientations.reserve(count);

  std::vector<double> x_acc, y_acc, z_acc;
  x_acc.reserve(params.size());
  y_acc.reserve(params.size());
  z_acc.reserve(params.size());

  for (auto& prob : params)
  {
    PnPResult result;

    result = optimize(prob);

    if (result.converged)
    {
      // we will save the full result here for debugging purposes
      solution_transforms.push_back(result.camera_to_target);
      translations.push_back(solution_transforms.back().translation());

      x_acc.push_back(result.camera_to_target.translation()(0));
      y_acc.push_back(result.camera_to_target.translation()(1));
      z_acc.push_back(result.camera_to_target.translation()(2));

      orientations.push_back(Eigen::Quaterniond(result.camera_to_target.rotation()));
    }
  }

  std::tie(output.p_stat.mean.x(), output.p_stat.stdev.x()) = computeStats(x_acc);
  std::tie(output.p_stat.mean.y(), output.p_stat.stdev.y()) = computeStats(y_acc);
  std::tie(output.p_stat.mean.z(), output.p_stat.stdev.z()) = computeStats(z_acc);
  output.q_stat = computeQuaternionStats(orientations);

  return output;
}

PnPNoiseStat qualifyNoise3D(const std::vector<PnPProblem3D>& params)
{
  PnPNoiseStat output;
  std::size_t count = params.size();

  std::vector<Eigen::Isometry3d> solution_transforms;
  solution_transforms.reserve(count);

  std::vector<Eigen::Vector3d> translations;
  translations.reserve(count);

  std::vector<Eigen::Quaterniond> orientations;
  orientations.reserve(count);

  std::vector<double> x_acc, y_acc, z_acc;
  x_acc.reserve(params.size());
  y_acc.reserve(params.size());
  z_acc.reserve(params.size());

  for (auto& prob : params)
  {
    PnPResult result;

    result = optimize(prob);

    if (result.converged)
    {
      // we will save the full result here for debugging purposes
      solution_transforms.push_back(result.camera_to_target);
      translations.push_back(solution_transforms.back().translation());

      x_acc.push_back(result.camera_to_target.translation()(0));
      y_acc.push_back(result.camera_to_target.translation()(1));
      z_acc.push_back(result.camera_to_target.translation()(2));

      orientations.push_back(Eigen::Quaterniond(result.camera_to_target.rotation()));
    }
  }

  std::tie(output.p_stat.mean.x(), output.p_stat.stdev.x()) = computeStats(x_acc);
  std::tie(output.p_stat.mean.y(), output.p_stat.stdev.y()) = computeStats(y_acc);
  std::tie(output.p_stat.mean.z(), output.p_stat.stdev.z()) = computeStats(z_acc);
  output.q_stat = computeQuaternionStats(orientations);

  return output;
}

}  // namespace industrial_calibration
