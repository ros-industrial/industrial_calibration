#pragma once

#include <industrial_calibration/core/target_finder.h>

#include <opencv2/core/mat.hpp>

namespace industrial_calibration
{
using TargetFinder2D3D = TargetFinder<2, 3, cv::Mat>;
using TargetFinderFactory2D3D = TargetFinderFactory<2, 3, cv::Mat>;

}  // namespace industrial_calibration
