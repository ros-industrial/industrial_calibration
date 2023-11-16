#pragma once

#include <industrial_calibration/serialization.h>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

namespace industrial_calibration
{
struct CircleDetectorParams
{
  /** @brief The minimum grayscale pixel intensity value at which to start the image thresholding (inclusive) */
  float minThreshold = 50;
  /** @brief The maximum grayscale pixel intensity value at which to stop the image thresholding (inclusive) */
  float maxThreshold = 220;
  /** @brief The number of thresholding steps to apply */
  std::size_t nThresholds = 18;

  /** @brief The number of times a particular circle must be identified to be considered valid (must be <= the number of
   * threshold steps) */
  size_t minRepeatability = 3;
  /** @brief The radius (pixels) around an identified circle within which new detected blobs will be considered to be
   * the same feature as the circle */
  float circleInclusionRadius = 5;
  /** @brief The maximum difference in radius (pixels) between an identified circle and a detected blob, above which
   * the blob will not be considered to be the same feature as the previously identified circle */
  float maxRadiusDiff = 5;

  /** @brief The maximum average deviation of the contour of a blob from its calculated ellipse fit (percentage) */
  float maxAverageEllipseError = 0.02f;

  /** @brief Flag for color filtering */
  bool filterByColor = true;
  /** @brief Color intensity of circle center in the binary image (value must be 0 or 255) */
  uchar circleColor = 0;

  /** @brief Flag for filtering by area */
  bool filterByArea = true;
  /** @brief Minimum blob area (px^2) */
  float minArea = 25.0f;
  /** @brief Maximum blob area (px^2) */
  float maxArea = 5000.0f;

  /** @brief Flag for circularity filtering */
  bool filterByCircularity = false;
  /** @brief Minimum blob circularity ratio - for a perfect circle this value is 1.0 / PI (~0.333) */
  float minCircularity = 0.8f;
  /**@ brief Maximum blob circularity ratio */
  float maxCircularity = std::numeric_limits<float>::max();

  /** @brief Flag for inertia filtering */
  bool filterByInertia = false;
  /** @brief Minimum blob inertia ratio */
  float minInertiaRatio = 0.3f;
  /** @brief Maximum blob inertia ratio - for a perfect circle, this value is 1.0 */
  float maxInertiaRatio = std::numeric_limits<float>::max();

  /** @brief Flag for convexity filtering */
  bool filterByConvexity = true;
  /** @brief Minimum blob convexity */
  float minConvexity = 0.95f;
  /** @brief Maximum blob convexity */
  float maxConvexity = std::numeric_limits<float>::max();
};

class CircleDetector : public cv::FeatureDetector
{
public:
  CircleDetector(const CircleDetectorParams& parameters);

  struct CV_EXPORTS Center
  {
    cv::Point2d location;
    double radius;
    double confidence;
  };

  /**
   * @brief Detects circle keypoints in an image
   * @param image
   * @param keypoints
   * @param mask
   */
  virtual void detect(cv::InputArray image, std::vector<cv::KeyPoint>& keypoints,
                      cv::InputArray mask = cv::noArray()) override;

  /**
   * @brief Draws the contours and keypoints of detected circles
   * @param image
   * @return
   */
  cv::Mat drawDetectedCircles(const cv::Mat& image);

  /**
   * @brief Creates a circle detector pointer from a parameter structure
   * @param params
   * @return
   */
  static cv::Ptr<CircleDetector> create(const CircleDetectorParams& params = CircleDetectorParams());

protected:
  const CircleDetectorParams params;
};

}  // namespace industrial_calibration

using namespace industrial_calibration;

namespace YAML
{
template<>
struct convert<CircleDetectorParams>
{
  using T = CircleDetectorParams;
  inline static bool decode(const Node& node, T& val)
  {
    val.nThresholds = getMember<decltype(val.nThresholds)>(node, "nThresholds");
    val.minThreshold = getMember<decltype(val.minThreshold)>(node, "minThreshold");
    val.maxThreshold = getMember<decltype(val.maxThreshold)>(node, "maxThreshold");
    val.minRepeatability = getMember<decltype(val.minRepeatability)>(node, "minRepeatability");
    val.circleInclusionRadius = getMember<decltype(val.circleInclusionRadius)>(node, "circleInclusionRadius");
    val.maxRadiusDiff = getMember<decltype(val.maxRadiusDiff)>(node, "maxRadiusDiff");
    val.maxAverageEllipseError = getMember<decltype(val.maxAverageEllipseError)>(node, "maxAverageEllipseError");

    val.filterByColor = getMember<decltype(val.filterByColor)>(node, "filterByColor");
    val.circleColor = static_cast<unsigned short>(getMember<int>(node, "circleColor"));

    val.filterByArea = getMember<decltype(val.filterByArea)>(node, "filterByArea");
    val.minArea = getMember<decltype(val.minArea)>(node, "minArea");
    val.maxArea = getMember<decltype(val.maxArea)>(node, "maxArea");

    val.filterByCircularity = getMember<decltype(val.filterByCircularity)>(node, "filterByCircularity");
    val.minCircularity = getMember<decltype(val.minCircularity)>(node, "minCircularity");
    val.maxCircularity = getMember<decltype(val.maxCircularity)>(node, "maxCircularity");

    val.filterByInertia = getMember<decltype(val.filterByInertia)>(node, "filterByInertia");
    val.minInertiaRatio = getMember<decltype(val.minInertiaRatio)>(node, "minInertiaRatio");
    val.maxInertiaRatio = getMember<decltype(val.maxInertiaRatio)>(node, "maxInertiaRatio");

    val.filterByConvexity = getMember<decltype(val.filterByConvexity)>(node, "filterByConvexity");
    val.minConvexity = getMember<decltype(val.minConvexity)>(node, "minConvexity");
    val.maxConvexity = getMember<decltype(val.maxConvexity)>(node, "maxConvexity");

    return true;
  }
};

} // namespace YAML
