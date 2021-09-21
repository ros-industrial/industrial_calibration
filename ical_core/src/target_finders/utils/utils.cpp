#include <ical_core/target_finders/utils/utils.h>
#include <ical_core/exceptions.h>
#include <ical_core/optimizations/utils/ceres_math_utilities.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

namespace industrial_calibration
{
std::vector<cv::Point2d> getReprojections(const Eigen::Isometry3d& camera_to_target, const CameraIntrinsics& intr,
                                          const std::vector<Eigen::Vector3d>& target_points)
{
  std::vector<cv::Point2d> reprojections;
  for (const auto& point_in_target : target_points)
  {
    Eigen::Vector3d in_camera = camera_to_target * point_in_target;

    double uv[2];
    projectPoint(intr, in_camera.data(), uv);

    reprojections.push_back(cv::Point2d(uv[0], uv[1]));
  }
  return reprojections;
}

void drawReprojections(const std::vector<cv::Point2d>& reprojections, int size, cv::Scalar color, cv::Mat& image)
{
  for (const auto& pt : reprojections)
  {
    cv::circle(image, pt, size, color);
  }
}

cv::Mat readImageOpenCV(const std::string& path)
{
  cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
  if (image.data == nullptr) throw BadFileException("Failed to load file at: '" + path + "'");

  return image;
}

}  // namespace industrial_calibration
