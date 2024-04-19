#pragma once

#include <industrial_calibration/core/target_finder.h>

#include <opencv2/core/mat.hpp>

namespace industrial_calibration
{
/** @brief Typedef for a target finder based on OpenCV that finds 2D targets in an RGB image */
using TargetFinderOpenCV = TargetFinder<2, 3, cv::Mat>;

/** @brief Plugin interface for generating OpenCV-based target finders */
struct TargetFinderFactoryOpenCV
{
  using Ptr = std::shared_ptr<TargetFinderFactoryOpenCV>;
  using ConstPtr = std::shared_ptr<const TargetFinderFactoryOpenCV>;

  TargetFinderFactoryOpenCV() = default;
  virtual ~TargetFinderFactoryOpenCV() = default;

  virtual TargetFinderOpenCV::ConstPtr create(const YAML::Node& config) const = 0;

  static std::string getSection() { return TARGET_FINDER_SECTION; }
};

}  // namespace industrial_calibration
