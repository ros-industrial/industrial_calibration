#pragma once

#include <industrial_calibration/target_finders/target_finder.h>
#include <industrial_calibration/target_finders/utils/circle_detector.h>

namespace industrial_calibration
{
/**
 * @brief Structure containing the necessary data to represent a modified circle grid target
 */
struct ModifiedCircleGridTarget : Target
{
  /**
   * @brief Constructor
   * @param rows - Number of rows in the target
   * @param cols - Number of columns in the target
   * @param spacing - The spacing between adjacent circle centers (m)
   */
  ModifiedCircleGridTarget(const unsigned rows, const unsigned cols, const double spacing);

  bool operator==(const ModifiedCircleGridTarget& other) const;

  virtual Correspondence2D3D::Set createCorrespondences(const TargetFeatures& target_features) const override;

  std::vector<Eigen::Vector3d> createPoints() const;

  unsigned rows;
  unsigned cols;
  double spacing;
};

/**
 * @brief This class finds 2D features (circle centers) from images of a known ModifiedCircleGridTarget.
 * All points must be seen or it will fail. Features are returned in the same order as points are defined in the target.
 */
class ModifiedCircleGridTargetFinder : public TargetFinder
{
public:
  ModifiedCircleGridTargetFinder(const ModifiedCircleGridTarget& target);
  ModifiedCircleGridTargetFinder(const ModifiedCircleGridTarget& target, const CircleDetectorParams& params);

  /**
   * @brief Finds target features in an input image
   * @param image
   * @return
   */
  virtual TargetFeatures findTargetFeatures(const cv::Mat& image) const override;

  /**
   * @brief A debugging utility that will draw target features onto an image for display purposes.
   * Usually you want to call findTargetFeatures() above then this with the result.
   */
  virtual cv::Mat drawTargetFeatures(const cv::Mat& image, const TargetFeatures& target_features) const override;

  virtual const Target& target() const override { return target_; }

  inline const CircleDetectorParams& getCircleDetectorParams() const { return params_; }

protected:
  const ModifiedCircleGridTarget target_;
  const CircleDetectorParams params_;
};

struct ModifiedCircleGridTargetFinderFactory : public TargetFinderFactory
{
public:
  TargetFinder::ConstPtr create(const YAML::Node& config) const override;
};

}  // namespace industrial_calibration
