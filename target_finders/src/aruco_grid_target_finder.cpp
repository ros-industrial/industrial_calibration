/**
 * ArUco gridboard detector, following the same pattern as ModifiedCircleGridTargetFinder.
 * Author: Joseph Schornak
 */
#include <industrial_calibration/target_finders/aruco_grid_target_finder.h>
#include <industrial_calibration/core/serialization.h>

namespace
{
/**
 * @brief For a given ArUco GridBoard, create a map of marker corner coordinates keyed to marker IDs.
 * @param board - ArUco GridBoard to use when generating the map
 * @return Resulting map. Keys are the IDs for the ArUco markers in the board. Values are vectors containing the four
 * board-relative coordinates for the corners of the marker.
 */
std::map<int, std::vector<Eigen::Vector3d>> mapArucoIdsToObjPts(const cv::Ptr<cv::aruco::GridBoard>& board)
{
  std::map<int, std::vector<Eigen::Vector3d>> map_ids_to_corners;
  for (std::size_t i = 0; i < board->ids.size(); i++)
  {
    std::vector<Eigen::Vector3d> obj_pts(board->objPoints[i].size());
    std::transform(
        board->objPoints[i].begin(), board->objPoints[i].end(), obj_pts.begin(),
        [](const cv::Point3f& o) -> Eigen::Vector3d { return Eigen::Vector3f(o.x, o.y, o.z).cast<double>(); });

    map_ids_to_corners.insert(std::make_pair(board->ids[i], obj_pts));
  }
  return map_ids_to_corners;
}

}  // namespace

namespace industrial_calibration
{
ArucoGridTarget::ArucoGridTarget(const int rows, const int cols, const float aruco_marker_dim, const float marker_gap,
                                 const int dictionary_id)
  : ArucoGridTarget(cv::aruco::GridBoard::create(cols, rows, aruco_marker_dim, marker_gap,
                                                 cv::aruco::getPredefinedDictionary(dictionary_id)))
{
}

ArucoGridTarget::ArucoGridTarget(const cv::Ptr<cv::aruco::GridBoard>& board_in)
  : board(board_in), points(mapArucoIdsToObjPts(board))
{
}

bool ArucoGridTarget::operator==(const ArucoGridTarget& other) const
{
  auto board_size = board->getGridSize();
  auto other_board_size = other.board->getGridSize();
  bool equal = true;
  equal &= other_board_size.width == board_size.width;
  equal &= other_board_size.height == board_size.height;
  equal &=
      (std::abs(other.board->getMarkerLength() - board->getMarkerLength()) < std::numeric_limits<float>::epsilon());
  equal &= (std::abs(other.board->getMarkerSeparation() - board->getMarkerSeparation()) <
            std::numeric_limits<float>::epsilon());
  equal &= other.points == points;
  return equal;
}

Correspondence2D3D::Set ArucoGridTarget::createCorrespondences(const TargetFeatures& target_features) const
{
  Correspondence2D3D::Set correspondences;
  correspondences.reserve(target_features.size());

  for (auto it = target_features.begin(); it != target_features.end(); ++it)
  {
    for (std::size_t i = 0; i < it->second.size(); ++i)
    {
      Correspondence2D3D corr;
      corr.in_target = points.at(it->first).at(i);
      corr.in_image = target_features.at(it->first).at(i);
      correspondences.push_back(corr);
    }
  }
  return correspondences;
}

ArucoGridBoardTargetFinder::ArucoGridBoardTargetFinder(const ArucoGridTarget& target) : TargetFinder(), target_(target)
{
}

TargetFeatures ArucoGridBoardTargetFinder::findTargetFeatures(const cv::Mat& image) const
{
  TargetFeatures map_ids_to_obs_corners;

  std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
  std::vector<int> marker_ids;
  cv::Ptr<cv::aruco::DetectorParameters> parameters(new cv::aruco::DetectorParameters);

  cv::aruco::detectMarkers(image, target_.board->dictionary, marker_corners, marker_ids, parameters,
                           rejected_candidates);
  cv::aruco::refineDetectedMarkers(image, target_.board, marker_corners, marker_ids, rejected_candidates);

  for (unsigned i = 0; i < marker_ids.size(); i++)
  {
    std::vector<cv::Point2f> corner_pts = marker_corners[i];
    VectorEigenVector<2> obs_pts(4);

    for (unsigned j = 0; j < corner_pts.size(); j++)
    {
      obs_pts[j] = Eigen::Vector2d(corner_pts[j].x, corner_pts[j].y).cast<double>();
    }
    map_ids_to_obs_corners.emplace(marker_ids[i], obs_pts);
  }
  return map_ids_to_obs_corners;
}

cv::Mat ArucoGridBoardTargetFinder::drawTargetFeatures(const cv::Mat& image,
                                                       const TargetFeatures& target_features) const
{
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  for (auto it = target_features.begin(); it != target_features.end(); ++it)
  {
    marker_ids.push_back(it->first);
    std::vector<cv::Point2f> cv_obs(it->second.size());
    std::transform(it->second.begin(), it->second.end(), cv_obs.begin(),
                   [](const Eigen::Vector2d& o) { return cv::Point2d(o.x(), o.y()); });
    marker_corners.push_back(cv_obs);
  }
  cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);
  return image;
}

TargetFinder::ConstPtr ArucoGridTargetFinderFactory::create(const YAML::Node& config) const
{
  auto cols = getMember<int>(config, "cols");
  auto rows = getMember<int>(config, "rows");
  auto aruco_marker_dim = getMember<float>(config, "aruco_marker_dim");
  auto marker_gap = getMember<float>(config, "marker_gap");
  auto dictionary = getMember<int>(config, "dictionary");

  ArucoGridTarget target(rows, cols, aruco_marker_dim, marker_gap, dictionary);
  return std::make_shared<const ArucoGridBoardTargetFinder>(target);
}

}  // namespace industrial_calibration
