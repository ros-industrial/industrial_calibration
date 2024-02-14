#pragma once

#include <industrial_calibration/optimizations/extrinsic_hand_eye.h>

namespace industrial_calibration
{
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

}  // namespace industrial_calibration
