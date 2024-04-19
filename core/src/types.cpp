#include <industrial_calibration/core/types.h>
#include <industrial_calibration/core/serialization.h>

using namespace industrial_calibration;

namespace YAML
{
template <Eigen::Index SENSOR_DIM, Eigen::Index WORLD_DIM>
Node convert<Correspondence<SENSOR_DIM, WORLD_DIM>>::encode(const Correspondence<SENSOR_DIM, WORLD_DIM>& corr)
{
  YAML::Node node;
  node["in_image"] = corr.in_image;
  node["in_target"] = corr.in_target;
  return node;
}

template <Eigen::Index SENSOR_DIM, Eigen::Index WORLD_DIM>
bool convert<Correspondence<SENSOR_DIM, WORLD_DIM>>::decode(const YAML::Node& node,
                                                            Correspondence<SENSOR_DIM, WORLD_DIM>& rhs)
{
  rhs.in_image = getMember<decltype(rhs.in_image)>(node, "in_image");
  rhs.in_target = getMember<decltype(rhs.in_target)>(node, "in_target");

  return true;
}

template <Eigen::Index SENSOR_DIM, Eigen::Index WORLD_DIM>
Node convert<Observation<SENSOR_DIM, WORLD_DIM>>::encode(const Observation<SENSOR_DIM, WORLD_DIM>& obs)
{
  YAML::Node node;
  node["correspondences"] = obs.correspondence_set;
  node["to_target_mount"] = obs.to_target_mount;
  node["to_camera_mount"] = obs.to_camera_mount;
  return node;
}

template <Eigen::Index SENSOR_DIM, Eigen::Index WORLD_DIM>
bool convert<Observation<SENSOR_DIM, WORLD_DIM>>::decode(const YAML::Node& node,
                                                         Observation<SENSOR_DIM, WORLD_DIM>& obs)
{
  obs.correspondence_set = getMember<decltype(obs.correspondence_set)>(node, "correspondences");
  obs.to_target_mount = getMember<decltype(obs.to_target_mount)>(node, "to_target_mount");
  obs.to_camera_mount = getMember<decltype(obs.to_camera_mount)>(node, "to_camera_mount");

  return true;
}

// Explicit template instantiation
template struct convert<Correspondence<2, 3>>;
template struct convert<Correspondence<3, 3>>;
template struct convert<Observation<2, 3>>;
template struct convert<Observation<3, 3>>;

}  // namespace YAML
