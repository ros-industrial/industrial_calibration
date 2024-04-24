#pragma once

#include <industrial_calibration/core/types.h>

#include <Eigen/Core>
#include <map>
#include <memory>
#include <vector>

namespace YAML
{
class Node;
}

namespace industrial_calibration
{
/** @brief Typedef for an STL vector of Eigen vector objects. Use a specialized allocator in the case that types
 * divisible by 16 bytes are used, specifically Eigen::Vector2d */
template <Eigen::Index DIM>
using VectorEigenVector =
    std::vector<Eigen::Matrix<double, DIM, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, DIM, 1>>>;

/** @brief Typedef for a container of target features from a calibration target.
 *  This definition allows for multiple features to be associated with a single unique identifier
 *  (such as the corners of an ArUco tag)
 */
template <Eigen::Index DIM>
using TargetFeatures = std::map<unsigned, VectorEigenVector<DIM>>;

/** @brief Typedef for target features that exist in 2 dimensions (e.g., pixels from a 2D image) */
using TargetFeatures2D = TargetFeatures<2>;
/** @brief Typedef for target features that exist in 3 dimensions (e.g., points from a point cloud) */
using TargetFeatures3D = TargetFeatures<3>;

/**
 * @brief Base class for calibration target definitions
 */
template <Eigen::Index SENSOR_DIM, Eigen::Index WORLD_DIM>
struct Target
{
  Target() = default;
  virtual ~Target() = default;

  /**
   * @brief Creates a set of correspondences between an input set of target features (e.g., identified in a 2D image)
   * and the same features from the known geometry of the target
   * @param target_features - map of target features identified in a sensor data measurement (e.g., 2D image)
   * @return
   */
  virtual typename Correspondence<SENSOR_DIM, WORLD_DIM>::Set
  createCorrespondences(const TargetFeatures<SENSOR_DIM>& target_features) const = 0;
};

/** @brief Target mapping 2-dimensional sensor measurements (e.g., from a 2D camera) to a 3-dimensional world space */
using Target2D3D = Target<2, 3>;
/** @brief Target mapping 3-dimensional sensor measurements (e.g., from a 3D sensor) to a 3-dimensional world space */
using Target3D3D = Target<3, 3>;

/**
 * @brief Base class for target finders
 */
template <Eigen::Index SENSOR_DIM, Eigen::Index WORLD_DIM, typename SensorDataT>
class TargetFinder
{
public:
  using Ptr = std::shared_ptr<TargetFinder>;
  using ConstPtr = std::shared_ptr<const TargetFinder>;

  TargetFinder() = default;
  virtual ~TargetFinder() = default;

  /**
   * @brief Finds the features of the target in a sensor data measurement (e.g., 2D image)
   */
  virtual TargetFeatures<SENSOR_DIM> findTargetFeatures(const SensorDataT& measurement) const = 0;

  /**
   * @brief Draws the target features on an input sensor data measurement (e.g., a 2D image)
   */
  virtual SensorDataT drawTargetFeatures(const SensorDataT& sensor_data,
                                         const TargetFeatures<SENSOR_DIM>& target_features) const
  {
    return sensor_data;
  }

  /**
   * @brief Returns the definition of the target used by the finder
   */
  virtual const Target<SENSOR_DIM, WORLD_DIM>& target() const = 0;

  /**
   * @brief Finds correspondences from a sensor data measurement (e.g., 2D image)
   */
  typename Correspondence<SENSOR_DIM, WORLD_DIM>::Set findCorrespondences(const SensorDataT& measurement) const
  {
    TargetFeatures<SENSOR_DIM> features = findTargetFeatures(measurement);
    if (features.empty()) throw std::runtime_error("Failed to find any target features");

    return target().createCorrespondences(findTargetFeatures(measurement));
  }

  /**
   * @brief Finds correspondences from a set of sensor data measurements (e.g., 2D images)
   */
  typename Correspondence<SENSOR_DIM, WORLD_DIM>::Set
  findCorrespondences(const std::vector<SensorDataT>& measurements) const
  {
    Correspondence2D3D::Set correspondences;

    for (const SensorDataT& measurement : measurements)
    {
      auto corrs = findCorrespondences(measurement);
      correspondences.insert(correspondences.end(), corrs.begin(), corrs.end());
    }

    return correspondences;
  }
};

}  // namespace industrial_calibration
