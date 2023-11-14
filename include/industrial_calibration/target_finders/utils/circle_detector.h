#pragma once

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
