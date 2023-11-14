#pragma once

#include <ical_core/types.h>
#include <ical_core/dh_chain.h>

namespace industrial_calibration
{
namespace test
{
/**
 * @brief a test pin-hole camera that has intrinsics, but also image size data needed for generating simulated
 * information
 */
struct Camera
{
  CameraIntrinsics intr;
  int width;
  int height;
};

/**
 * @brief Create a test camera set with kinect-like parameters
 * @return
 */
Camera makeKinectCamera();

/**
 * @brief A sample grid target for test purposes
 * Looking down at the target
 *   - The origin is in the lower left corner
 *   - The x coordinate defines the feature column
 *   - The y coordinate defines the feature row
 *   - The points are ordered row-wise from the top left corner to the bottom right corner
 */
struct Target
{
  Target() = default;

  Target(const unsigned rows, const unsigned cols, const double spacing)
    : origin_idx((rows - 1) * cols), center(double(rows - 1) * spacing / 2.0, double(cols - 1) * spacing / 2.0, 0.0)
  {
    points.reserve(rows * cols);

    for (unsigned i = 1; i < (rows + 1); i++)
    {
      double y = (rows - i) * spacing;
      for (unsigned j = 0; j < cols; j++)
      {
        double x = j * spacing;
        Eigen::Vector3d point(x, y, 0.0);
        points.push_back(point);
      }
    }

    origin_idx = (rows - 1) * cols;
  }

  std::vector<Eigen::Vector3d> points;
  std::size_t origin_idx;
  Eigen::Vector3d center;
};

/**
 * @brief Creates a pose that is perturbed with the input noise levels relative to the input pose
 * @param pose - reference pose
 * @param spatial_noise - translational noise standard deviation (m)
 * @param angle_noise - angular noise standard deviation (rad)
 * @return
 */
Eigen::Isometry3d perturbPose(const Eigen::Isometry3d& pose, double spatial_noise, double angle_noise);

/**
 * @brief Creates a DH parameter-based robot representation of an ABB IRB2400
 * @return
 */
DHChain createABBIRB2400();

/**
 * @brief Creates a DH parameter-based robot representation of a generic two-axis part positioner with arbitrary base
 * offset
 * @return
 */
DHChain createTwoAxisPositioner();

/**
 * @brief Creates a kinematic chain whose DH parameters are pertubed with Gaussian noise relative to the reference
 * kinematic chain
 * @param in - reference kinematic chain
 * @param stddev - standard deviation to apply to all DH parameters individually
 * @return
 */
DHChain perturbDHChain(const DHChain& in, const double stddev);

}  // namespace test
}  // namespace industrial_calibration
