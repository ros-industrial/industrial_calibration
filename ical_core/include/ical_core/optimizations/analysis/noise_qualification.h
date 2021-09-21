#pragma once

#include <ical_core/types.h>
#include <ical_core/optimizations/pnp.h>
#include <ical_core/optimizations/analysis/statistics.h>

namespace industrial_calibration
{
/**
 * @brief This function qualifies 2D sensor noise by
 * comparing PnP results from images taken at same pose.
 * Sensor noise can be understood by inspecting the returned standard
 * deviations.
 * @param Sets of PnP 2D problem parameters
 * @return Noise Statistics: a vector of means & std devs
 */
PnPNoiseStat qualifyNoise2D(const std::vector<PnPProblem>& params);

/**
 * @brief This function qualifies 3D sensor noise by
 * comparing PnP results from scans taken at the same pose.
 * Sensor noise can be understood by inspecting the returned standard
 * deviations.
 * @param params 3D image parameters
 * @return Noise Statiscics: a vector of standard deviations and the mean pos
 */
PnPNoiseStat qualifyNoise3D(const std::vector<PnPProblem3D>& params);

}  // namespace industrial_calibration
