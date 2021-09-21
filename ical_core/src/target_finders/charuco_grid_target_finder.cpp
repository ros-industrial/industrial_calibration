#include <ical_core/target_finders/charuco_grid_target_finder.h>
#include <ical_core/exceptions.h>

namespace
{
/**
 * @brief For a given ChArUco board, create a map of chessboard intersection coordinates keyed to marker IDs.
 * @param board - ChArUco board to use when generating the map
 * @return Resulting map. Keys are the indices of chessboard intersections in the board. Values are board-relative
 * coordinates for each intersection.
 */
std::map<unsigned, Eigen::Vector3d> mapCharucoIdsToObjPts(const cv::Ptr<cv::aruco::CharucoBoard>& board)
{
  std::map<unsigned, Eigen::Vector3d> points;
  for (std::size_t i = 0; i < board->chessboardCorners.size(); i++)
  {
    const auto& corner = board->chessboardCorners.at(i);
    Eigen::Vector3f pt(corner.x, corner.y, corner.z);
    points.emplace(i, pt.cast<double>());
  }
  return points;
}

}  // namespace

namespace industrial_calibration
{
CharucoGridTarget::CharucoGridTarget(const int rows, const int cols, const float chessboard_dim,
                                     const float aruco_marker_dim, const int dictionary_id)
  : CharucoGridTarget::CharucoGridTarget(cv::aruco::CharucoBoard::create(
        cols, rows, chessboard_dim, aruco_marker_dim, cv::aruco::getPredefinedDictionary(dictionary_id)))
{
}

CharucoGridTarget::CharucoGridTarget(const cv::Ptr<cv::aruco::CharucoBoard>& board_in)
  : board(board_in), points(mapCharucoIdsToObjPts(board))
{
}

bool CharucoGridTarget::operator==(const CharucoGridTarget& other) const
{
  auto board_size = board->getChessboardSize();
  auto other_board_size = other.board->getChessboardSize();
  bool equal = true;
  equal &= other.points == points;
  equal &=
      (std::abs(other.board->getMarkerLength() - board->getMarkerLength()) < std::numeric_limits<float>::epsilon());
  equal &=
      (std::abs(other.board->getSquareLength() - board->getSquareLength()) < std::numeric_limits<float>::epsilon());
  equal &= other_board_size.width == board_size.width;
  equal &= other_board_size.height == board_size.height;
  return equal;
}

std::vector<Correspondence2D3D> CharucoGridTarget::createCorrespondences(const TargetFeatures& target_features) const
{
  std::vector<Correspondence2D3D> correspondences;
  correspondences.reserve(target_features.size());

  for (auto it = target_features.begin(); it != target_features.end(); it++)
  {
    Correspondence2D3D corr;
    corr.in_target = points.at(it->first);
    // Get the first (and only) feature from the current iterator
    corr.in_image = target_features.at(it->first).at(0);
    correspondences.push_back(corr);
  }
  return correspondences;
}

CharucoGridBoardTargetFinder::CharucoGridBoardTargetFinder(const CharucoGridTarget& target)
  : TargetFinder(), target_(target)
{
}

TargetFeatures CharucoGridBoardTargetFinder::findTargetFeatures(const cv::Mat& image) const
{
  // Create a generic set of parameters
  // TODO: expose the setting of these parameters
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

  // Detect the ArUco markers
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  cv::aruco::detectMarkers(image, target_.board->dictionary, marker_corners, marker_ids, parameters);

  if (marker_ids.empty())
  {
    throw ICalException("No ArUco markers were detected");
  }

  // Detect the chessboard intersections given the observed ArUco markers
  std::vector<cv::Point2f> charuco_corners;
  std::vector<int> charuco_ids;
  int detected_corners = cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, image, target_.board,
                                                              charuco_corners, charuco_ids);

  // Create the map of observed features
  TargetFeatures target_features;
  for (unsigned i = 0; i < static_cast<unsigned>(detected_corners); i++)
  {
    const cv::Point2f& corner = charuco_corners.at(i);
    VectorEigenVector<2> v_obs;
    v_obs.push_back(Eigen::Vector2d(corner.x, corner.y));
    target_features.emplace(static_cast<unsigned>(charuco_ids[i]), v_obs);
  }

  return target_features;
}

cv::Mat CharucoGridBoardTargetFinder::drawTargetFeatures(const cv::Mat& image,
                                                         const TargetFeatures& target_features) const
{
  std::vector<int> charuco_ids;
  charuco_ids.reserve(target_features.size());

  std::vector<cv::Point2f> charuco_corners;
  charuco_corners.reserve(target_features.size());

  for (auto it = target_features.begin(); it != target_features.end(); ++it)
  {
    // Add the ID
    charuco_ids.push_back(static_cast<int>(it->first));

    // Add the image coordinates
    const Eigen::Vector2f& pt = it->second.at(0).cast<float>();
    charuco_corners.push_back(cv::Point2f(pt.x(), pt.y()));
  }

  // Draw the detected corners
  cv::aruco::drawDetectedCornersCharuco(image, charuco_corners, charuco_ids, cv::Scalar(255, 0, 0));

  return image;
}

}  // namespace industrial_calibration
