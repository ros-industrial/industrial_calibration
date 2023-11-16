#pragma once

#include <industrial_calibration/target_finders/target_finder.h>

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

namespace industrial_calibration
{
/**
 * @brief Structure containing relevant data for a ArUco grid target
 */
struct ArucoGridTarget : Target
{
  /**
   * @brief Constructor
   * @param rows - number of rows in the target
   * @param cols - number of columns in the target
   * @param aruco_marker_dim - The length of the side of one ArUco marker (m)
   * @param marker_gap - The size of the gap between adjacent arUco markers (m)
   * @param dictionary_id - The dictionary of ArUco markers to use
   */
  ArucoGridTarget(const int rows, const int cols, const float aruco_marker_dim, const float marker_gap,
                  const int dictionary_id = cv::aruco::DICT_6X6_250);

  /**
   * @brief Constructor
   * @param board_in - OpenCV ArUco GridBoard object defining rows, columns, marker size, and marker spacing
   */
  ArucoGridTarget(const cv::Ptr<cv::aruco::GridBoard>& board_in);

  bool operator==(const ArucoGridTarget& other) const;

  /**
   * @brief Creates a set of correspondences between the corners of each ArUco tag observed in an image (ordered
   * clockwise from the "origin" corner) and their counterparts in the target (matched by ID)
   * @param target_features - Map of ArUco tag corners observed in an image
   * @return Set of corresponding features in the image to features in the ArUco grid target
   */
  virtual Correspondence2D3D::Set createCorrespondences(const TargetFeatures& target_features) const override;

  /** @brief Representation of the ArUco grid target */
  cv::Ptr<cv::aruco::GridBoard> board;
  /** @brief Map of 3D ArUco tag corners with corresponding IDs */
  std::map<int, std::vector<Eigen::Vector3d>> points;
};

/**
 * @brief This class finds 2D target features from images of a specified ArUco gridboard target.
 * The main advantage of this kind of target is that partial views still provide usable correspondences.
 * Target features are returned as a map where the marker ID is the key and the image coordinates of the
 * marker corners are the mapped value.
 */
class ArucoGridBoardTargetFinder : public TargetFinder
{
public:
  ArucoGridBoardTargetFinder(const ArucoGridTarget& target);

  /**
   * @brief Detect marker corner coordinates in the provided image.
   * @param image - Input image, ideally containing an ArUco gridboard.
   * @return Map matching marker ID numbers to a vector of marker corner coordinates. The vector will contain
   * four corners defined in the same order as in the output of the function cv::aruco::DetectMarkers()
   * (e.g. clockwise from the "origin" corner).
   */
  virtual TargetFeatures findTargetFeatures(const cv::Mat& image) const override;

  /**
   * @brief A debugging utility that will draw a set of target features onto an image for
   * display purposes. Usually you want to call findTargetFeatures() above then this with the result.
   * @param image - The image of the target
   * @param target_features - The target features identified in the input image
   */
  virtual cv::Mat drawTargetFeatures(const cv::Mat& image, const TargetFeatures& target_features) const override;

  virtual const Target& target() const override { return target_; }

protected:
  const ArucoGridTarget target_;
};

struct ArucoGridTargetFinderFactory : public TargetFinderFactory
{
public:
  TargetFinder::ConstPtr create(const YAML::Node& config) const override;
};

}  // namespace industrial_calibration
