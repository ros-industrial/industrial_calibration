#pragma once

#include <industrial_calibration/core/target_finder.h>

#include <opencv2/core/mat.hpp>

namespace industrial_calibration
{
/**
 * @brief Typedef for a target finder based on OpenCV that finds 2D targets in an RGB image
 * @ingroup target_finders_opencv
 */
class TargetFinderOpenCV : public TargetFinder<2, 3, cv::Mat>
{
public:
  /**
   * @brief Draws the target origin X and Y axes on the image
   * @details This method computes the homography matrix between the detected target features and the known target
   * features and uses it to project the X and Y axes of the target origin onto the image.
   */
  void drawTargetOrigin(const cv::Mat& image, const TargetFeatures2D& features, const double axis_length = 0.1) const;
};

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
