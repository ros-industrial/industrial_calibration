/**
 * ChAruco gridboard detector, following the same pattern as ModifiedCircleGridTargetFinder.
 * Author: John Berkebile
 */
#pragma once

#include <industrial_calibration/target_finders/target_finder.h>

#include <Eigen/Dense>
#include <map>
#include <opencv2/aruco/charuco.hpp>

namespace industrial_calibration
{
/**
 * @brief Structure containing relevant data for a ChArUco grid target
 */
struct CharucoGridTarget : public Target
{
  /**
   * @brief Constructor
   * @param rows - number of rows in the target
   * @param cols - number of columns in the target
   * @param chessboard_dim - The length of the side of one chessboard square (m)
   * @param aruco_marker_dim - The length of the side of one ArUco marker (m)
   * @param dictionary_id - The enumeration ID of the dictionary of ArUco markers to use.
   * <a
   * href="https://github.com/opencv/opencv_contrib/blob/6a18431421087aaf5c2a579269c72da4de54d5bf/modules/aruco/include/opencv2/aruco/dictionary.hpp#L141-L163">ArUco
   * dictionary enumerations</a>
   */
  CharucoGridTarget(const int rows, const int cols, const float chessboard_dim, const float aruco_marker_dim,
                    const int dictionary_id = cv::aruco::DICT_6X6_250);

  CharucoGridTarget(const cv::Ptr<cv::aruco::CharucoBoard>& board_in);

  bool operator==(const CharucoGridTarget& other) const;

  /**
   * @brief Creates a set of correspondences between chessboard intersections observed in an image and their
   * counterparts in the target (matched by ID)
   * @param target_features - Map of observed chessboard intersections and their IDs
   * @return Set of corresponding features in the image to features in the ChArUco target
   */
  virtual Correspondence2D3D::Set createCorrespondences(const TargetFeatures& target_features) const override;

  /** @brief Representation of the ChArUco board target */
  cv::Ptr<cv::aruco::CharucoBoard> board;
  /** @brief Map of 3D chessboard corners with corresponding IDs */
  std::map<unsigned, Eigen::Vector3d> points;
};

/**
 * @brief This class finds 2D features from images of a specified ChArUco gridboard target.
 * The main advantage of this kind of target is that partial views still provide usable correspondences.
 */
class CharucoGridBoardTargetFinder : public TargetFinder
{
public:
  CharucoGridBoardTargetFinder(const CharucoGridTarget& target);

  /**
   * @brief Detects chessboard intersection coordinates in the provided image.
   * @param image - Input image, ideally containing a ChArUco grid target.
   * @return Map matching marker ID numbers to the 2D position of the chessboard intersections
   */
  virtual TargetFeatures findTargetFeatures(const cv::Mat& image) const override;

  /**
   * @brief A debugging utility that will draw target features set onto an input image for display purposes
   * @param image - Input image, ideally containing a ChArUco grid target
   * @param target_features - Chessboard intersections (obtained by calling @ref findTargetFeatures)
   * @return An image with the chessboard intersections and IDs overlaid on the input image
   */
  virtual cv::Mat drawTargetFeatures(const cv::Mat& image, const TargetFeatures& target_features) const override;

  virtual const Target& target() const override { return target_; }

protected:
  const CharucoGridTarget target_;
};

struct CharucoGridTargetFinderFactory : public TargetFinderFactory
{
public:
  TargetFinder::ConstPtr create(const YAML::Node& config) const override;
};

}  // namespace industrial_calibration
