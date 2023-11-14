#include <industrial_calibration/target_finders/target_finder.h>

#include <iostream>

namespace industrial_calibration
{
/**
 * @brief Finds correspondences from an image
 */
Correspondence2D3D::Set TargetFinder::findCorrespondences(const cv::Mat& image) const
{
  TargetFeatures features = findTargetFeatures(image);
  if (features.empty()) throw std::runtime_error("Failed to find any target features");
  std::cout << "Found " << features.size() << " target features" << std::endl;

  return target().createCorrespondences(findTargetFeatures(image));
}

/**
 * @brief Finds correspondences from a set of images
 */
Correspondence2D3D::Set TargetFinder::findCorrespondences(const std::vector<cv::Mat>& images) const
{
  Correspondence2D3D::Set correspondences;

  for (const cv::Mat& image : images)
  {
    auto corrs = findCorrespondences(image);
    correspondences.insert(correspondences.end(), corrs.begin(), corrs.end());
  }

  return correspondences;
}

}  // namespace industrial_calibration
