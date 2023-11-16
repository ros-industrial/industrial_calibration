#pragma once

#include <industrial_calibration/types.h>

#include <Eigen/Core>
#include <map>
#include <opencv2/core/mat.hpp>
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

/** Typedef for an STL map with an unsigned integer key and STL vector of Eigen vector value */
template <Eigen::Index DIM>
using MapVectorEigenVector = std::map<unsigned, VectorEigenVector<DIM>>;

/** @brief Typedef for a container of target features from a calibration target.
 *  This definition allows for multiple features to be associated with a single unique identifier
 *  (such as the corners of an ArUco tag)
 */
using TargetFeatures = MapVectorEigenVector<2>;

/**
 * @brief Base class for calibration target definitions
 */
struct Target
{
  Target() = default;
  virtual ~Target() = default;

  /**
   * @brief Creates a set of correspondences between an input set of target features (identified in a 2D image) and the
   * same features from the known geometry of the target
   * @param target_features - map of target features identified in a 2D image
   * @return
   */
  virtual Correspondence2D3D::Set createCorrespondences(const TargetFeatures& target_features) const = 0;
};

/**
 * @brief Base class for target finders
 */
class TargetFinder
{
public:
  using Ptr = std::shared_ptr<TargetFinder>;
  using ConstPtr = std::shared_ptr<const TargetFinder>;

  TargetFinder() = default;
  virtual ~TargetFinder() = default;

  /**
   * @brief Finds the features of the target in an image
   */
  virtual TargetFeatures findTargetFeatures(const cv::Mat& image) const = 0;

  /**
   * @brief Draws the target features on an input image
   */
  virtual cv::Mat drawTargetFeatures(const cv::Mat& image, const TargetFeatures& target_features) const = 0;

  /**
   * @brief Returns the definition of the target used by the finder
   */
  virtual const Target& target() const = 0;

  /**
   * @brief Finds correspondences from an image
   */
  Correspondence2D3D::Set findCorrespondences(const cv::Mat& image) const;

  /**
   * @brief Finds correspondences from a set of images
   */
  Correspondence2D3D::Set findCorrespondences(const std::vector<cv::Mat>& images) const;
};

/** @brief Plugin interface for generating target finders */
struct TargetFinderFactory
{
  using Ptr = std::shared_ptr<TargetFinderFactory>;
  using ConstPtr = std::shared_ptr<const TargetFinderFactory>;

  TargetFinderFactory() = default;
  virtual ~TargetFinderFactory() = default;

  virtual TargetFinder::ConstPtr create(const YAML::Node& config) const = 0;

  static std::string getSection() { return TARGET_FINDER_SECTION; }
};

}  // namespace industrial_calibration
