#include <industrial_calibration/target_finders/opencv/target_finder.h>
#include <industrial_calibration/core/utils.h>

#include <opencv2/opencv.hpp>

namespace industrial_calibration
{
void TargetFinderOpenCV::drawTargetOrigin(const cv::Mat& image, const TargetFeatures2D& target_features,
                                          const double axis_length) const
{
  // Compute the correspondences for the given target features
  Correspondence2D3D::Set correspondences = target().createCorrespondences(target_features);

  // At least 4 correspondences required to compute homography
  if (correspondences.size() < 4)
    return;

  // Compute the homography matrix
  auto H = calculateHomography(correspondences);

  // Create the set of target 2D points to project into the image: one at the origin, one along the +X direction, and
  // one in the +Y direction
  Eigen::Matrix2Xd pts(2, 3);
  pts.col(0) = Eigen::Vector2d::Zero();
  pts.col(1) = Eigen::Vector2d::UnitX() * axis_length;
  pts.col(2) = Eigen::Vector2d::UnitY() * axis_length;

  // Project the points into UV coordinates
  Eigen::Matrix2Xd uv = projectHomography(H, pts);

  // Draw a line from the origin to the +X point (in red)
  cv::line(image, cv::Point2f(uv(0, 0), uv(1, 0)), cv::Point2f(uv(0, 1), uv(1, 1)), cv::Vec3d(0, 0, 255), 3);

  // Draw a line from the origin to the +Y point (in green)
  cv::line(image, cv::Point2f(uv(0, 0), uv(1, 0)), cv::Point2f(uv(0, 2), uv(1, 2)), cv::Vec3d(0, 255, 0), 3);
}

}  // namespace industrial_calibration
