#pragma once

#include <ical_core/target_finders/target_finder.h>

#include <yaml-cpp/node/node.h>

namespace industrial_calibration
{
/**
 * @brief Plugin class for target finders
 */
class TargetFinderPlugin : public TargetFinder
{
public:
  using TargetFinder::TargetFinder;

  TargetFeatures findTargetFeatures(const cv::Mat& image) const override final
  {
    return finder_->findTargetFeatures(image);
  }

  cv::Mat drawTargetFeatures(const cv::Mat& image, const TargetFeatures& target_features) const override final
  {
    return finder_->drawTargetFeatures(image, target_features);
  }

  const Target& target() const override final
  {
    return finder_->target();
  }

  virtual void init(const YAML::Node& config) = 0;

protected:
  std::unique_ptr<const TargetFinder> finder_;
};

struct ModifiedCircleGridTargetFinderPlugin : public TargetFinderPlugin
{
public:
  void init(const YAML::Node& config) override;
};

struct CharucoGridTargetFinderPlugin : public TargetFinderPlugin
{
public:
  void init(const YAML::Node& config) override;
};

struct ArucoGridTargetFinderPlugin : public TargetFinderPlugin
{
public:
  void init(const YAML::Node& config) override;
};

}  // namespace industrial_calibration
