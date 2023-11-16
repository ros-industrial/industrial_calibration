#pragma once

#include <industrial_calibration/camera_intrinsics.h>

#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>
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
std::vector<cv::Point2d> getReprojections(const Eigen::Isometry3d& camera_to_target, const CameraIntrinsics& intr,
                                          const std::vector<Eigen::Vector3d>& target_points);

void drawReprojections(const std::vector<cv::Point2d>& reprojections, int size, cv::Scalar color, cv::Mat& image);

/**
 * @brief Reads and image from file into an OpenCV matrix
 * @param path
 * @return
 */
cv::Mat readImageOpenCV(const std::string& path);

}  // namespace industrial_calibration
