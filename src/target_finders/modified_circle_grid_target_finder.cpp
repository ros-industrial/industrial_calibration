#include <industrial_calibration/target_finders/modified_circle_grid_target_finder.h>
#include <industrial_calibration/target_finders/utils/circle_detector.h>
#include <industrial_calibration/exceptions.h>

#include <opencv2/calib3d.hpp>
#include <memory>

namespace industrial_calibration
{
static void drawPointLabel(const std::string& label, const cv::Point2d& position, const cv::Scalar& color,
                           cv::Mat& image)
{
  const double font_scale = 0.75;
  const int line_thickness = 2;

  cv::Point label_origin = cv::Point(static_cast<int>(position.x), static_cast<int>(position.y));

  cv::putText(image, label, label_origin, cv::FONT_HERSHEY_SIMPLEX, font_scale, color, line_thickness);

  // Draw dot
  const int radius = 5.0;
  cv::circle(image, position, radius, color, -1);
}

static cv::Mat renderObservations(const cv::Mat& input, const std::vector<cv::Point2d>& observation_points,
                                  const ModifiedCircleGridTarget& target)
{
  cv::Mat output_image;
  input.copyTo(output_image);

  if (!observation_points.empty())
  {
    cv::Size pattern_size(static_cast<int>(target.cols), static_cast<int>(target.rows));

    cv::Mat center_image = cv::Mat(observation_points);
    cv::Mat center_converted;
    center_image.convertTo(center_converted, CV_32F);
    cv::drawChessboardCorners(output_image, pattern_size, center_converted, true);

    // Draw point labels // TODO
    drawPointLabel("First Point", observation_points[0], cv::Scalar(0, 255, 0), output_image);
    drawPointLabel("Origin", observation_points[target.rows * target.cols - target.cols], cv::Scalar(255, 0, 0),
                   output_image);
    drawPointLabel("Last Point", observation_points[observation_points.size() - 1], cv::Scalar(0, 0, 255),
                   output_image);
  }

  return output_image;
}

template <typename DETECTOR_PTR>
static std::vector<cv::Point2d> extractKeyPoints(const cv::Mat& image, const std::vector<cv::Point2d>& centers,
                                                 cv::Ptr<DETECTOR_PTR>& detector_ptr, const std::size_t rows,
                                                 const std::size_t cols, const bool flipped)
{
  /*
    Note(cLewis):
    This is the same method called in the beginning of findCirclesGrid,
    unfortunately, they don't
    return their keypoints. If OpenCV changes, the keypoint locations may not
    match, which has the
    risk of failing with updates to OpenCV.
  */

  // Extract KeyPoints
  std::vector<cv::KeyPoint> keypoints;
  cv::Point large_point;

  std::size_t start_first_row = 0;
  std::size_t end_first_row = cols - 1;
  std::size_t start_last_row = rows * cols - cols;
  std::size_t end_last_row = rows * cols - 1;

  for (double alpha = 1.0; alpha <= 3.0; alpha += 0.01)
  {
    bool found = false;
    for (int beta = 0; beta <= 100; beta++)
    {
      cv::Mat altered_image;
      image.convertTo(altered_image, -1, alpha, beta);
      detector_ptr->detect(altered_image, keypoints);
      if (keypoints.size() >= rows * cols)
      {
        found = true;
        break;
      }
    }
    if (found)
    {
      break;
    }
  }

  // OpenCV creates duplicate centers sometimes.  https://github.com/opencv/opencv/issues/4775
  // Make sure every center has a matching keypoint.  It may be sufficient just to check centers for duplicates.
  std::vector<cv::Point2d> centers_tmp = centers;
  for (auto&& keypoint : keypoints)
  {
    auto it = std::find_if(centers_tmp.begin(), centers_tmp.end(),
                           [&](const cv::Point2f& pt) { return (pt.x == keypoint.pt.x && pt.y == keypoint.pt.y); });

    if (it != centers_tmp.end())
    {
      centers_tmp.erase(it);
    }
  }

  if (!centers_tmp.empty())
    throw ICalException("Centers and keypoints did not match. Check this issue: "
                        "'https://github.com/opencv/opencv/issues/4775'");

  // If a flipped pattern is found, flip the rows/columns
  std::size_t temp_rows = flipped ? cols : rows;
  std::size_t temp_cols = flipped ? rows : cols;

  // Determine which circle is the largest
  double start_first_row_size = -1.0;
  double start_last_row_size = -1.0;
  double end_first_row_size = -1.0;
  double end_last_row_size = -1.0;

  cv::Point2d start_last_row_pt = centers[start_last_row];
  cv::Point2d end_last_row_pt = centers[end_last_row];
  cv::Point2d start_first_row_pt = centers[start_first_row];
  cv::Point2d end_first_row_pt = centers[end_first_row];

  for (std::size_t i = 0; i < keypoints.size(); i++)
  {
    double x = keypoints[i].pt.x;
    double y = keypoints[i].pt.y;
    double ksize = keypoints[i].size;

    if (x == start_last_row_pt.x && y == start_last_row_pt.y)
    {
      start_last_row_size = ksize;
    }
    if (x == end_last_row_pt.x && y == end_last_row_pt.y)
    {
      end_last_row_size = ksize;
    }
    if (x == start_first_row_pt.x && y == start_first_row_pt.y)
    {
      start_first_row_size = ksize;
    }
    if (x == end_first_row_pt.x && y == end_first_row_pt.y)
    {
      end_first_row_size = ksize;
    }
  }

  // No keypoint match for one or more corners
  if (start_last_row_size < 0.0 || start_first_row_size < 0.0 || end_last_row_size < 0.0 || end_first_row_size < 0.0)
    throw ICalException("No keypoint match for one or more centers");

  /*
    Note(cLewis):
    Determine if ordering is usual by computing cross product of two vectors.
    Normal ordering has
    z-axis positive in cross. The most common ordering is with points going from
    left to right then
    top to bottom.
  */
  bool usual_ordering = true;
  double v1x, v1y, v2x, v2y;

  v1x = end_last_row_pt.x - start_last_row_pt.x;
  v1y = -end_last_row_pt.y + start_last_row_pt.y;
  v2x = end_first_row_pt.x - end_last_row_pt.x;
  v2y = -end_first_row_pt.y + end_last_row_pt.y;

  double cross = v1x * v2y - v1y * v2x;
  if (cross < 0.0)
  {
    usual_ordering = false;
  }

  // Construct the output object
  std::vector<cv::Point2d> observation_points;
  observation_points.reserve(centers.size());

  /*
    Note(cLewis): Largest circle at start of last row
    ......
    ......
    ......
    o.....
  */
  if (start_last_row_size > start_first_row_size && start_last_row_size > end_first_row_size &&
      start_last_row_size > end_last_row_size)
  {
    large_point.x = start_last_row_pt.x;
    large_point.y = start_last_row_pt.y;
    if (usual_ordering)
    {
      for (std::size_t j = 0; j < centers.size(); j++)
      {
        observation_points.push_back(centers[j]);
      }
    }
    else  // unusual ordering
    {
      for (int j = temp_cols - 1; j >= 0; j--)
      {
        for (int k = temp_rows - 1; k >= 0; k--)
        {
          observation_points.push_back(centers[static_cast<std::size_t>(k) * temp_cols + static_cast<std::size_t>(j)]);
        }
      }
    }
  }

  /*
    Note(cLewis): Largest circle at end of first row
    .....o
    ......
    ......
    ......
  */
  else if (end_first_row_size > end_last_row_size && end_first_row_size > start_last_row_size &&
           end_first_row_size > start_first_row_size)
  {
    large_point.x = end_first_row_pt.x;
    large_point.y = end_first_row_pt.y;
    if (usual_ordering)
    {
      for (int j = (centers.size() - 1); j >= 0; j--)
      {
        observation_points.push_back(centers[static_cast<std::size_t>(j)]);
      }
    }
    else  // unusual ordering
    {
      for (std::size_t j = 0; j < temp_cols; j++)
      {
        for (std::size_t k = 0; k < temp_rows; k++)
        {
          observation_points.push_back(centers[k * temp_cols + j]);
        }
      }
    }
  }

  /*
    Note(cLewis): Largest circle at end of last row
    ......
    ......
    ......
    .....o
  */
  else if (end_last_row_size > start_last_row_size && end_last_row_size > end_first_row_size &&
           end_last_row_size > start_first_row_size)
  {
    large_point.x = end_last_row_pt.x;
    large_point.y = end_last_row_pt.y;

    if (usual_ordering)
    {
      for (std::size_t j = 0; j < temp_cols; j++)
      {
        for (int k = temp_rows - 1; k >= 0; k--)
        {
          observation_points.push_back(centers[static_cast<std::size_t>(k) * temp_cols + j]);
        }
      }
    }
    else  // unusual ordering
    {
      for (std::size_t j = 0; j < temp_cols; j++)
      {
        for (std::size_t k = 0; k < temp_rows; k++)
        {
          observation_points.push_back(centers[k * temp_cols + j]);
        }
      }
    }
  }

  /*
    Note(cLewis): Largest circle at start of first row
    o.....
    ......
    ......
    ......
  */
  else if (start_first_row_size > end_last_row_size && start_first_row_size > end_first_row_size &&
           start_first_row_size > start_last_row_size)
  {
    large_point.x = start_first_row_pt.x;
    large_point.y = start_first_row_pt.y;
    if (usual_ordering)
    {
      for (int j = temp_cols - 1; j >= 0; j--)
      {
        for (std::size_t k = 0; k < temp_rows; k++)
        {
          observation_points.push_back(centers[k * temp_cols + static_cast<std::size_t>(j)]);
        }
      }
    }
    else  // unusual ordering
    {
      for (int j = temp_cols - 1; j >= 0; j--)
      {
        for (int k = temp_rows - 1; k >= 0; k--)
        {
          observation_points.push_back(centers[static_cast<std::size_t>(k) * temp_cols + static_cast<std::size_t>(j)]);
        }
      }
    }
  }

  else
  {
    throw ICalException("No matching configuration");
  }

  return observation_points;
}

template <typename PARAMS, typename DETECTOR_PTR, typename DETECTOR>
static std::vector<cv::Point2d> extractModifiedCircleGrid(const cv::Mat& image, const ModifiedCircleGridTarget& target,
                                                          const PARAMS& params)
{
  cv::Ptr<DETECTOR_PTR> detector_ptr;
  detector_ptr = DETECTOR::create(params);

  bool flipped = false;

  std::size_t cols = target.cols;
  std::size_t rows = target.rows;

  cv::Size pattern_size(cols, rows);
  cv::Size pattern_size_flipped(rows, cols);

  std::vector<cv::Point2d> centers;

  bool regular_pattern_found = cv::findCirclesGrid(image, pattern_size, centers,
                                                   cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, detector_ptr);
  if (regular_pattern_found && (centers.size() == rows * cols))
  {
    // Do nothing, there used to be something in here, may have
    // to re-structure this in the future.
  }
  else  // Try flipped pattern size
  {
    bool flipped_pattern_found = cv::findCirclesGrid(
        image, pattern_size_flipped, centers, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, detector_ptr);
    if (flipped_pattern_found && (centers.size() == rows * cols))
    {
      flipped = true;
    }
  }

  if (centers.size() == 0) throw ICalException("Failed to find circle centers");

  return extractKeyPoints(image, centers, detector_ptr, rows, cols, flipped);
}

ModifiedCircleGridTarget::ModifiedCircleGridTarget(const unsigned rows_, const unsigned cols_, const double spacing_)
  : rows(rows_), cols(cols_), spacing(spacing_)
{
}

bool ModifiedCircleGridTarget::operator==(const ModifiedCircleGridTarget& other) const
{
  bool equal = true;
  equal &= (rows == other.rows);
  equal &= (cols == other.cols);
  equal &= (std::abs(spacing - other.spacing) < std::numeric_limits<double>::epsilon());

  return equal;
}

Correspondence2D3D::Set ModifiedCircleGridTarget::createCorrespondences(const TargetFeatures& target_features) const
{
  std::vector<Eigen::Vector3d> target_points = createPoints();
  assert(target_features.size() == target_points.size());

  Correspondence2D3D::Set correspondences;
  correspondences.reserve(target_features.size());

  for (std::size_t i = 0; i < target_points.size(); ++i)
  {
    Correspondence2D3D corr;
    corr.in_target = target_points.at(i);
    // Get the first (and only) element of the features at the current index; increment the index
    corr.in_image = target_features.at(static_cast<const unsigned>(i)).at(0);
    correspondences.push_back(corr);
  }

  return correspondences;
}

std::vector<Eigen::Vector3d> ModifiedCircleGridTarget::createPoints() const
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(rows * cols);

  for (std::size_t i = 1; i < (rows + 1); i++)
  {
    double y = (rows - i) * spacing;
    for (std::size_t j = 0; j < cols; j++)
    {
      double x = j * spacing;
      Eigen::Vector3d point(x, y, 0.0);
      points.push_back(point);
    }
  }

  assert(points.size() == (rows * cols));

  return points;
}

ModifiedCircleGridTargetFinder::ModifiedCircleGridTargetFinder(const ModifiedCircleGridTarget& target,
                                                               const CircleDetectorParams& params)
  : TargetFinder(), target_(target), params_(params)
{
  assert(target_.cols > 0);
  assert(target_.rows > 0);
}

ModifiedCircleGridTargetFinder::ModifiedCircleGridTargetFinder(const ModifiedCircleGridTarget& target)
  : ModifiedCircleGridTargetFinder(target, CircleDetectorParams())
{
}

TargetFeatures ModifiedCircleGridTargetFinder::findTargetFeatures(const cv::Mat& image) const
{
  // Call modified circle finder
  std::vector<cv::Point2d> points =
      extractModifiedCircleGrid<CircleDetectorParams, CircleDetector, CircleDetector>(image, target_, params_);

  // Construct the output map
  TargetFeatures observations;
  for (unsigned i = 0; i < points.size(); ++i)
  {
    const cv::Point2d& pt = points.at(i);
    VectorEigenVector<2> v_obs;
    v_obs.push_back(Eigen::Vector2d(pt.x, pt.y));
    observations.emplace(i, v_obs);
  }

  return observations;
}

cv::Mat ModifiedCircleGridTargetFinder::drawTargetFeatures(const cv::Mat& image,
                                                           const TargetFeatures& target_features) const
{
  // Draw all detected circles
  CircleDetector circle_detector_(params_);
  cv::Mat out_image = circle_detector_.drawDetectedCircles(image);

  // Draw the detected circle centers
  std::vector<cv::Point2d> cv_obs;
  cv_obs.reserve(target_features.size());
  for (auto it = target_features.begin(); it != target_features.end(); ++it)
  {
    const Eigen::Vector2d& pt = it->second.at(0);
    cv_obs.push_back(cv::Point2d(pt.x(), pt.y()));
  }

  return renderObservations(out_image, cv_obs, target_);
}

}  // namespace industrial_calibration
