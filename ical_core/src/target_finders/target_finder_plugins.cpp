#include <ical_core/target_finders/target_finder_plugin.h>
#include <ical_core/exceptions.h>
#include <ical_core/target_finders/aruco_grid_target_finder.h>
#include <ical_core/target_finders/charuco_grid_target_finder.h>
#include <ical_core/target_finders/modified_circle_grid_target_finder.h>

#include <boost/core/demangle.hpp>
#include <iostream>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace industrial_calibration
{
template <typename T>
static T getMember(const YAML::Node& n, const std::string& key)
{
  try
  {
    T value = n[key].as<T>();
    return value;
  }
  catch (const YAML::BadConversion&)
  {
    throw BadFileException("Failed to convert parameter '" + key + "' to type '" +
                           boost::core::demangle(typeid(T).name()) + "'");
  }
  catch (const YAML::Exception&)
  {
    throw BadFileException("Failed to find '" + key + "' parameter of type '" +
                           boost::core::demangle(typeid(T).name()) + "'");
  }
}

/**
 * @brief Load CircleDetector parameters from a yaml node.
 * @param node
 * @throw std::runtime_error if a parameter in the file exists and fails to be parsed.  Putting a string instead of a
 * float, etc.
 */
CircleDetectorParams loadCircleDetectorParams(const YAML::Node& node)
{
  CircleDetectorParams p;

  p.nThresholds = getMember<decltype(p.nThresholds)>(node, "nThresholds");
  p.minThreshold = getMember<decltype(p.minThreshold)>(node, "minThreshold");
  p.maxThreshold = getMember<decltype(p.maxThreshold)>(node, "maxThreshold");
  p.minRepeatability = getMember<decltype(p.minRepeatability)>(node, "minRepeatability");
  p.circleInclusionRadius = getMember<decltype(p.circleInclusionRadius)>(node, "circleInclusionRadius");
  p.maxRadiusDiff = getMember<decltype(p.maxRadiusDiff)>(node, "maxRadiusDiff");
  p.maxAverageEllipseError = getMember<decltype(p.maxAverageEllipseError)>(node, "maxAverageEllipseError");

  p.filterByColor = getMember<decltype(p.filterByColor)>(node, "filterByColor");
  p.circleColor = static_cast<unsigned short>(getMember<int>(node, "circleColor"));

  p.filterByArea = getMember<decltype(p.filterByArea)>(node, "filterByArea");
  p.minArea = getMember<decltype(p.minArea)>(node, "minArea");
  p.maxArea = getMember<decltype(p.maxArea)>(node, "maxArea");

  p.filterByCircularity = getMember<decltype(p.filterByCircularity)>(node, "filterByCircularity");
  p.minCircularity = getMember<decltype(p.minCircularity)>(node, "minCircularity");
  p.maxCircularity = getMember<decltype(p.maxCircularity)>(node, "maxCircularity");

  p.filterByInertia = getMember<decltype(p.filterByInertia)>(node, "filterByInertia");
  p.minInertiaRatio = getMember<decltype(p.minInertiaRatio)>(node, "minInertiaRatio");
  p.maxInertiaRatio = getMember<decltype(p.maxInertiaRatio)>(node, "maxInertiaRatio");

  p.filterByConvexity = getMember<decltype(p.filterByConvexity)>(node, "filterByConvexity");
  p.minConvexity = getMember<decltype(p.minConvexity)>(node, "minConvexity");
  p.maxConvexity = getMember<decltype(p.maxConvexity)>(node, "maxConvexity");

  return p;
}

void ModifiedCircleGridTargetFinderPlugin::init(const YAML::Node& config)
{
  int rows = config["rows"].as<int>();
  int cols = config["cols"].as<int>();
  double spacing = config["spacing"].as<double>();
  ModifiedCircleGridTarget target(rows, cols, spacing);

  try
  {
    CircleDetectorParams circle_detector_params = loadCircleDetectorParams(config["circle_detector_params"]);
    std::cout << "Successfully loaded circle detector parameters" << std::endl;

    finder_ = std::make_unique<const ModifiedCircleGridTargetFinder>(target, circle_detector_params);
  }
  catch (const std::exception& ex)
  {
    std::cout << "Failed to load circle detector parameters: '" << ex.what() << "'. Using default values" << std::endl;
    finder_ = std::make_unique<const ModifiedCircleGridTargetFinder>(target);
  }
}

void CharucoGridTargetFinderPlugin::init(const YAML::Node& config)
{
  int cols = config["cols"].as<int>();
  int rows = config["rows"].as<int>();
  float chessboard_dim = config["chessboard_dim"].as<float>();
  float aruco_marker_dim = config["aruco_marker_dim"].as<float>();
  int dictionary = config["dictionary"].as<int>();

  CharucoGridTarget target(rows, cols, chessboard_dim, aruco_marker_dim, dictionary);
  finder_ = std::make_unique<const CharucoGridBoardTargetFinder>(target);
}

void ArucoGridTargetFinderPlugin::init(const YAML::Node& config)
{
  int cols = config["cols"].as<int>();
  int rows = config["rows"].as<int>();
  float aruco_marker_dim = config["aruco_marker_dim"].as<float>();
  float marker_gap = config["marker_gap"].as<float>();
  int dictionary = config["dictionary"].as<int>();

  ArucoGridTarget target(rows, cols, aruco_marker_dim, marker_gap, dictionary);
  finder_ = std::make_unique<const ArucoGridBoardTargetFinder>(target);
}

}  // namespace industrial_calibration
