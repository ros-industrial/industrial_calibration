#pragma once

#include <industrial_calibration/core/camera_intrinsics.h>
#include <industrial_calibration/core/types.h>

#include <opencv2/core/mat.hpp>

namespace industrial_calibration
{
using VectorVector2d = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;

/**
 * @brief Draws reprojected points onto an image
 */
void drawReprojections(const VectorVector2d& reprojections, int size, cv::Scalar color, cv::Mat& image);

/**
 * @brief Reads and image from file into an OpenCV matrix
 */
cv::Mat readImageOpenCV(const std::string& path);

}  // namespace industrial_calibration
