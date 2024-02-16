#pragma once

#include <industrial_calibration/core/camera_intrinsics.h>

#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/types_c.h>
#include <vector>
#if __GNUC__ >= 8
#include <filesystem>
using path = std::filesystem::path;
#else
#include <experimental/filesystem>
using path = std::experimental::filesystem::path;
#endif

namespace YAML
{
class Node;
}

namespace industrial_calibration
{
using VectorEigenIsometry = std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>;

/**
 * @brief Get the uv coordinates of a point reprojected into the image frame
 * @param camera_to_target The transformation from the camera frame to the target frame
 * @param intr The intrinsic values of the camera
 * @param target_points A vector of target points
 * @return A vector of uv values in the image frame
 */
std::vector<cv::Point2d> getReprojections(const Eigen::Isometry3d& camera_to_target, const CameraIntrinsics& intr,
                                          const std::vector<Eigen::Vector3d>& target_points);

void drawReprojections(const std::vector<cv::Point2d>& reprojections, int size, cv::Scalar color, cv::Mat& image);

/**
 * @brief Reads and image from file into an OpenCV matrix
 * @param path
 * @return
 */
cv::Mat readImageOpenCV(const std::string& path);

/**
 * @brief Loads pairs of images and poses from a directory and a YAML configuration
 */
std::tuple<VectorEigenIsometry, std::vector<cv::Mat>> loadPoseImagePairs(const path& data_dir, const YAML::Node& data);

std::string getStringRPY(const Eigen::Vector3d& rpy);
std::string getStringXYZ(const Eigen::Vector3d& xyz);
std::string getStringQuaternion(const Eigen::Quaterniond& q);
std::string getStringIntrinsics(const std::array<double, 4>& values);
std::string getStringDistortion(const std::array<double, 5>& values);

void printTitle(const std::string& title, int width = 80);
void printTransform(const Eigen::Isometry3d& transform, const std::string& parent_frame, const std::string& child_frame,
                    const std::string& description);
void printTransformDiff(const Eigen::Isometry3d& transform1, const Eigen::Isometry3d& transform2,
                        const std::string& parent_frame, const std::string& child_frame,
                        const std::string& description);
void printOptResults(bool converged, double initial_cost_per_obs, double final_cost_per_obs);
void printCameraIntrinsics(const std::array<double, 4>& values, const std::string& description);
void printCameraDistortion(const std::array<double, 5>& values, const std::string& description);

}  // namespace industrial_calibration
