#include <industrial_calibration/core/camera_intrinsics.h>
#include <industrial_calibration/core/serialization.h>

using namespace industrial_calibration;

namespace YAML
{
Node convert<CameraIntrinsics>::encode(const CameraIntrinsics& rhs)
{
  YAML::Node node;
  node["cx"] = rhs.cx();
  node["cy"] = rhs.cy();
  node["fx"] = rhs.fx();
  node["fy"] = rhs.fy();
  return node;
}

bool convert<CameraIntrinsics>::decode(const YAML::Node& node, CameraIntrinsics& rhs)
{
  rhs.cx() = getMember<double>(node, "cx");
  rhs.cy() = getMember<double>(node, "cy");
  rhs.fx() = getMember<double>(node, "fx");
  rhs.fy() = getMember<double>(node, "fy");

  return true;
}

}  // namespace YAML
