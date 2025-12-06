#pragma once

#include <industrial_calibration/core/types.h>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace industrial_calibration
{
/**
 * @brief Gets the weights of the four pixels surrounding a floating point location in an image based on the XY
 * distance of the pixels to the feature
 * @details Returns the weights for the points starting with the upper-left pixel and moving clockwise
 */
Eigen::Vector4d getWeightedAverageWeights(const double row, const double col)
{
  // Separate the decimal component from the rest of the float, such that (dx, dy) represents the vector from the upper
  // left pixel to the feature
  double o_row, o_col;
  const double d_row = std::modf(row, &o_row);
  const double d_col = std::modf(col, &o_col);

  // Calculate the vector normal from each of the four pixels nearest to the feature, starting from the upper left and
  // moving clockwise. The weight is the inverse of this distance: smaller distance -> larger weight
  Eigen::Vector4d out;
  out[0] = 1.0 / Eigen::Vector2d(0.0 - d_row, 0.0 - d_col).norm();
  out[1] = 1.0 / Eigen::Vector2d(1.0 - d_row, 0.0 - d_col).norm();
  out[2] = 1.0 / Eigen::Vector2d(1.0 - d_row, 1.0 - d_col).norm();
  out[3] = 1.0 / Eigen::Vector2d(0.0 - d_row, 1.0 - d_col).norm();

  // Return the normalized weights, such that they sum to 1
  return out / out.sum();
}

Correspondence3D3D::Set get3DCorrespondences(
    const pcl::PointCloud<pcl::PointXYZ>& cloud, const Correspondence2D3D::Set& correspondences, int width)
{
  // Initialize the output variable
  Correspondence3D3D::Set out;
  out.reserve(correspondences.size());

  // Loop over each observation and calculate the 3D point from the point cloud that most closely matches the center of
  // the circle observation
  for (const Correspondence2D3D& corr : correspondences)
  {
    // Get the center of the circle
    const double col = corr.in_image.x();
    const double row = corr.in_image.y();

    // Assumption: the point cloud is organized and each point cloud point corresponds to a pixel
    // Since the center of the circle is a real number (i.e. a double) there is a discrete pixel on both sides of that
    // point in both X and Y Let's get the index of those 4 surrounding pixels
    const auto col_lower = static_cast<int>(std::floor(col));
    const auto row_lower = static_cast<int>(std::floor(row));
    const auto col_upper = static_cast<int>(std::ceil(col));
    const auto row_upper = static_cast<int>(std::ceil(row));

    // Get the weights for a weighted average of these four points based on their distance from the observed feature in
    // image space
    Eigen::Vector4d weights = getWeightedAverageWeights(row, col);

    /* Initialize variables to hold the 3D locations of the 4 pixels in a column-wise matrix
     * Points:
     *   ul (upper left, corresponds to the pixel indices x (column) low, y (row) low)
     *   ur (upper right, corresponds to the pixel indices x (column) high, y (row) low)
     *   lr (lower_right, corresponds to the pixel indices x (column) high, y (row) high)
     *   ll (lower left, corresponds to the pixel indices x (column) low, y (row) high)
     */
    Eigen::Matrix<double, 3, 4> pts;
    for (std::size_t i = 0; i < 4; ++i)
    {
      int idx;
      switch (i)
      {
      case 0:
        idx = row_lower * width + col_lower;
        break;
      case 1:
        idx = row_lower * width + col_upper;
        break;
      case 2:
        idx = row_upper * width + col_upper;
        break;
      case 3:
        idx = row_upper * width + col_lower;
        break;
      default:
        throw std::runtime_error("Invalid case value for point creation");
      }
      pts.col(i) = cloud.at(idx).getArray3fMap().cast<double>();
    }

    // Create a mask for a weighted average based on whether the point has finite values and the point norm is non-zero
    Eigen::Array<bool, 1, 4> finite_mask = pts.array().isFinite().colwise().all();
    Eigen::Array<bool, 1, 4> nonzero_mask = pts.colwise().norm().array() > 0.001;
    Eigen::Array<bool, 1, 4> mask = finite_mask && nonzero_mask;

    // Skip if all the points are masked
    if (mask.count() == 0)
    {
      std::cout << "No valid points found around feature" << std::endl;
      continue;
    }

    // Apply the boolean mask and renormalize the weights so they sum to 1
    weights.array() *= mask.cast<double>();
    weights /= weights.sum();

    Correspondence3D3D corr_3d;
    corr_3d.in_target = corr.in_target;

    // Apply the weighted average to the points
    corr_3d.in_image = pts * weights;
    // corr_3d.in_image = cloud.at(row_lower * width + col_lower).getArray3fMap().cast<double>();

    Eigen::IOFormat fmt(4, 0, ",", "\n", "[", "]");
    std::cout << "Point: " << corr_3d.in_image.transpose().format(fmt) << " (weighted average of " << mask.count()
              << "/4 nearest points)" << std::endl;

    out.push_back(corr_3d);
  }

  return out;
}

}  // namespace industrial_calibration
