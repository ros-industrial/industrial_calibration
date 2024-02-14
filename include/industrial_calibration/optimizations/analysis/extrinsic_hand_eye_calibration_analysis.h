#pragma once

namespace industrial_calibration
{
class ExtrinsicHandEyeProblem2D3D;
class ExtrinsicHandEyeResult;

/**
 * @brief Position and orientation difference/standard deviation between the location of the camera as determined by the
 * extrinsic calibration vs the per-observation PnP estimations
 */
struct ExtrinsicHandEyeAnalysisStats
{
  double pos_diff_mean;
  double pos_diff_stdev;
  double ori_diff_mean;
  double ori_diff_stdev;
};

/**
 * @brief Analyzes the results of the hand eye calibration by measuring the difference between the calibrated camera to
 * target transform and a PnP optimization estimation of the same transform
 */
ExtrinsicHandEyeAnalysisStats analyzeResults(const ExtrinsicHandEyeProblem2D3D& problem,
                                             const ExtrinsicHandEyeResult& opt_result);

/**
 * @brief 3D reprojection error statistics (m)
 */
struct ExtrinsicHandEye3dProjectionStats
{
  double min;
  double max;
  double mean;
  double stdev;
};

ExtrinsicHandEye3dProjectionStats analyze3dProjectionError(const ExtrinsicHandEyeProblem2D3D& problem,
                                                           const ExtrinsicHandEyeResult& opt_result);

}  // namespace industrial_calibration
