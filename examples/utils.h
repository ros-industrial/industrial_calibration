#pragma once

#include <industrial_calibration/target_finders/utils/utils.h>

#if __GNUC__ >= 8
#include <filesystem>
using path = std::filesystem::path;
#else
#include <experimental/filesystem>
using path = std::experimental::filesystem::path;
#endif
#include <Eigen/Geometry>
#include <iostream>
#include <opencv2/imgproc/types_c.h>
#include <yaml-cpp/yaml.h>

using VectorEigenIsometry = std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>;

namespace industrial_calibration
{
std::tuple<VectorEigenIsometry, std::vector<cv::Mat>> loadPoseImagePairs(const path& data_dir)
{
  VectorEigenIsometry poses;
  std::vector<cv::Mat> images;

  // Load the images and poses
  YAML::Node data = YAML::LoadFile((data_dir / "cal_data.yaml").string());
  poses.reserve(data.size());
  images.reserve(data.size());

  for (std::size_t i = 0; i < data.size(); ++i)
  {
    const std::string pose_file = data[i]["pose"].as<std::string>();
    poses.push_back(YAML::LoadFile((data_dir / pose_file).string()).as<Eigen::Isometry3d>());

    const std::string image_file = (data_dir / data[i]["image"].as<std::string>()).string();
    images.push_back(readImageOpenCV(image_file));
  }

  return std::make_tuple(poses, images);
}

inline std::string getStringRPY(const Eigen::Vector3d& rpy)
{
  std::stringstream s;
  s << "rpy=\"" << rpy(2) << "(" << rpy(2) * 180 / M_PI << " deg) " << rpy(1) << "(" << rpy(1) * 180 / M_PI << " deg) "
    << rpy(0) << "(" << rpy(0) * 180 / M_PI << " deg)\"";
  return s.str();
}

inline std::string getStringXYZ(const Eigen::Vector3d& xyz)
{
  std::stringstream s;
  s << "xyz=\"" << xyz(0) << " " << xyz(1) << " " << xyz(2) << "\"";
  return s.str();
}

inline std::string getStringQuaternion(const Eigen::Quaterniond& q)
{
  std::stringstream s;
  s << "qxyzw=\"" << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\"";
  return s.str();
}

inline std::string getStringIntrinsics(const std::array<double, 4>& values)
{
  std::stringstream s;
  s << "Intr:\nfx = " << values[0] << "\tfy = " << values[1] << "\ncx = " << values[2] << "\tcy = " << values[3];
  return s.str();
}

inline std::string getStringDistortion(const std::array<double, 5>& values)
{
  std::stringstream s;
  s << "Distortions:\n"
    << "k1 = " << values[0] << "\tk2 = " << values[1] << "\tp1 = " << values[2] << "\tp2 = " << values[3]
    << "\tk3 = " << values[4];
  return s.str();
}

inline void printTitle(const std::string& title, int width = 80)
{
  int length = title.length() + 8;
  if (length < width) length = width;

  std::string full;
  std::string mid;
  int delta = (length - title.length()) / 2;
  if ((int)(delta + delta + title.length()) == length)
  {
    full = std::string(length, '*');
    mid = std::string(delta - 1, '*');
  }
  else
  {
    full = std::string(length + 1, '*');
    mid = std::string(delta, '*');
  }
  std::cout << full << std::endl;
  std::cout << mid << " " << title << " " << mid << std::endl;
  std::cout << full << std::endl;
}

inline void printTransform(const Eigen::Isometry3d& transform, const std::string& parent_frame,
                           const std::string& child_frame, const std::string& description)
{
  std::cout << description << ":" << std::endl << transform.matrix() << std::endl << std::endl;
  std::cout << "--- URDF Format " << parent_frame << " to " << child_frame << " ---" << std::endl;
  Eigen::Vector3d rpy = transform.rotation().eulerAngles(2, 1, 0);
  Eigen::Quaterniond q(transform.rotation());
  std::cout << getStringXYZ(transform.translation()) << std::endl;
  std::cout << getStringRPY(rpy) << std::endl;
  std::cout << getStringQuaternion(q) << std::endl;
}

inline void printTransformDiff(const Eigen::Isometry3d& transform1, const Eigen::Isometry3d& transform2,
                               const std::string& parent_frame, const std::string& child_frame,
                               const std::string& description)
{
  Eigen::Isometry3d delta = transform1.inverse() * transform2;
  Eigen::AngleAxisd aa(delta.linear());
  Eigen::Vector3d rpy = delta.rotation().eulerAngles(2, 1, 0);

  std::cout << description << ":" << std::endl;
  std::cout << "--- " << parent_frame << " to " << child_frame << " Diff ---" << std::endl;

  Eigen::Vector3d trans = transform2.translation() - transform1.translation();
  std::cout << "DELTA S: " << trans.norm() << " at " << getStringXYZ(trans) << std::endl;
  std::cout << "DELTA A: " << (180.0 * aa.angle() / M_PI) << " and " << getStringRPY(rpy) << std::endl << std::endl;

  std::cout << "--- " << parent_frame << " to " << child_frame << " Diff Relative To Transform 1 ---" << std::endl;
  std::cout << delta.matrix() << std::endl << std::endl;

  std::cout << "DELTA S: " << delta.translation().norm() << " at " << getStringXYZ(delta.translation()) << std::endl;
  std::cout << "DELTA A: " << (180.0 * aa.angle() / M_PI) << " and " << getStringRPY(rpy) << std::endl;
}

inline void printOptResults(bool converged, double initial_cost_per_obs, double final_cost_per_obs)
{
  std::cout << "Did converge?: " << converged << std::endl;
  std::cout << "Initial cost?: " << std::sqrt(initial_cost_per_obs) << " (pixels per feature)" << std::endl;
  std::cout << "Final cost?: " << std::sqrt(final_cost_per_obs) << " (pixels per feature)" << std::endl;
}

inline void printCameraIntrinsics(const std::array<double, 4>& values, const std::string& description)
{
  std::cout << description << ":" << std::endl;
  std::cout << getStringIntrinsics(values) << std::endl;
}

inline void printCameraDistortion(const std::array<double, 5>& values, const std::string& description)
{
  std::cout << description << ":" << std::endl;
  std::cout << getStringDistortion(values) << std::endl;
}

}  // namespace industrial_calibration
