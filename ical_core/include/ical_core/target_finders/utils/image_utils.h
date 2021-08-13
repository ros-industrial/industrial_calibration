#pragma once

#include <ical_core/optimizations/utils/ceres_math_utilities.h>

#include <Eigen/Dense>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace industrial_calibration
{
/**
 * @brief Get the uv coordinates of a point reprojected into the image frame
 * @param camera_to_target The transformation from the camera frame to the target frame
 * @param intr The intrinsic values of the camera
 * @param target_points A vector of target points
 * @return A vector of uv values in the image frame
 */
inline std::vector<cv::Point2d> getReprojections(const Eigen::Isometry3d& camera_to_target,
                                                 const CameraIntrinsics& intr,
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

/**
 * @brief Draw a set of reprojections on an image
 * @param reprojections A vector of reprojections to be drawn on the image
 * @param size The size of circle to drawn
 * @param color The color of the circle drawn
 * @param image The image to draw the reprojections on
 */
inline void drawReprojections(const std::vector<cv::Point2d>& reprojections, int size, cv::Scalar color, cv::Mat& image)
{
  for (const auto& pt : reprojections)
  {
    cv::circle(image, pt, size, color);
  }
}

}  // namespace industrial_calibration
