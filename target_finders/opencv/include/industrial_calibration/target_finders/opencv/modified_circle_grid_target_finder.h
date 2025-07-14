#pragma once

#include <industrial_calibration/target_finders/opencv/target_finder.h>
#include <industrial_calibration/target_finders/opencv/circle_detector.h>

namespace industrial_calibration
{
/**
 * @brief Structure containing the necessary data to represent a modified circle grid target
 * @image html static/mod_circle_target_annotated.png
 * @details
 * - The one big dot allows us to disambiguate the orientation of symmetrical targets.
 * - The big dot is the "origin" or (0,0,0) of the target. The +Z axis comes out of the page, the +X axis runs along the
 * bottom of the page, left to right (the last row if your big dot is in the bottom left). The +Y runs up the page from
 * the big dot.
 * - When using this target finder, the points are ordered left to right, top to bottom as if reading a book.
 * The big dot, or origin, is in the bottom left. So the top left point is `0`, the top right point is `cols - 1`, the
 * second row first column is point `cols`, etc. See the image.
 *
 * @note You can create targets with custom size and spacing using the handy script, `calibration_target.py` found in
 * `target_finders/opencv/script`. Thanks to Jeremy Zoss for making this.
 *
 * - Pros:
 *   - Theoretically more accurate since the
 * - Cons:
 *   - Entire target must be visible
 *   - Detection requires tuning of many parameters
 *   - The grid can frequently be identified in an incorrect order
 * @ingroup target_finders_opencv
 */
struct ModifiedCircleGridTarget : Target2D3D
{
  /**
   * @brief Constructor
   * @param rows - Number of rows in the target
   * @param cols - Number of columns in the target
   * @param spacing - The spacing between adjacent circle centers (m)
   */
  ModifiedCircleGridTarget(const unsigned rows, const unsigned cols, const double spacing);

  bool operator==(const ModifiedCircleGridTarget& other) const;

  virtual Correspondence2D3D::Set createCorrespondences(const TargetFeatures2D& target_features) const override;

  std::vector<Eigen::Vector3d> createPoints() const;

  unsigned rows;
  unsigned cols;
  double spacing;
};

/**
 * @brief This class finds 2D features (circle centers) from images of a known ModifiedCircleGridTarget.
 * All points must be seen or it will fail. Features are returned in the same order as points are defined in the target.
 * @ingroup target_finders_opencv
 */
class ModifiedCircleGridTargetFinder : public TargetFinderOpenCV
{
public:
  ModifiedCircleGridTargetFinder(const ModifiedCircleGridTarget& target);
  ModifiedCircleGridTargetFinder(const ModifiedCircleGridTarget& target, const CircleDetectorParams& params);

  /**
   * @brief Finds target features in an input image
   * @param image
   * @return
   */
  virtual TargetFeatures2D findTargetFeatures(const cv::Mat& image) const override;

  /**
   * @brief A debugging utility that will draw target features onto an image for display purposes.
   * Usually you want to call findTargetFeatures() above then this with the result.
   */
  virtual cv::Mat drawTargetFeatures(const cv::Mat& image, const TargetFeatures2D& target_features) const override;

  virtual const Target2D3D& target() const override { return target_; }

  inline const CircleDetectorParams& getCircleDetectorParams() const { return params_; }

protected:
  const ModifiedCircleGridTarget target_;
  const CircleDetectorParams params_;
};

struct ModifiedCircleGridTargetFinderFactory : public TargetFinderFactoryOpenCV
{
public:
  TargetFinderOpenCV::ConstPtr create(const YAML::Node& config) const override;
};

}  // namespace industrial_calibration
