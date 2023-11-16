#include <industrial_calibration/optimizations/pnp.h>
#include <industrial_calibration/target_finders/target_finder_plugin.h>
#include <industrial_calibration/target_finders/utils/utils.h>
// Utilities
#include "utils.h"

#if __GNUC__ >= 8
#include <filesystem>
using path = std::filesystem::path;
#else
#include <experimental/filesystem>
using path = std::experimental::filesystem::path;
#endif

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <yaml-cpp/yaml.h>

using namespace industrial_calibration;

std::string WINDOW = "window";

/**
 * @brief Solves PnP optimization using OpenCV
 */
static Eigen::Isometry3d solveCVPnP(const CameraIntrinsics& intr, const Correspondence2D3D::Set& correspondences)
{
  cv::Mat cam_matrix(3, 3, cv::DataType<double>::type);
  cv::setIdentity(cam_matrix);

  cam_matrix.at<double>(0, 0) = intr.fx();
  cam_matrix.at<double>(1, 1) = intr.fy();
  cam_matrix.at<double>(0, 2) = intr.cx();
  cam_matrix.at<double>(1, 2) = intr.cy();

  std::vector<cv::Point2d> image_points;
  std::vector<cv::Point3d> target_points;
  image_points.reserve(correspondences.size());
  target_points.reserve(correspondences.size());
  for (const Correspondence2D3D& corr : correspondences)
  {
    image_points.push_back(cv::Point2d(corr.in_image.x(), corr.in_image.y()));
    target_points.push_back(cv::Point3d(corr.in_target.x(), corr.in_target.y(), corr.in_target.z()));
  }

  cv::Mat rvec(3, 1, cv::DataType<double>::type);
  cv::Mat tvec(3, 1, cv::DataType<double>::type);
  cv::solvePnP(target_points, image_points, cam_matrix, cv::noArray(), rvec, tvec);

  Eigen::Vector3d rr(Eigen::Vector3d(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)));
  Eigen::Isometry3d result(Eigen::AngleAxisd(rr.norm(), rr.normalized()));
  result.translation() = Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));

  return result;
}

/**
 * @brief Performs a PnP optimization using the algorithms in this repository and OpenCV and returns the
 * position/orientation difference bewteen the two results
 * @return a tuple containing the industrial calibration and OpenCV PnP optimization results, respectively
 */
std::tuple<PnPResult, Eigen::Isometry3d> run()
{
  const path data_path = path(EXAMPLE_DATA_DIR) / path("test_set_10x10");

  YAML::Node data_node = YAML::LoadFile((data_path / "cal_data.yaml").string());

  // Load the image
  const std::string image_path = data_node[0]["image"].as<std::string>();
  const cv::Mat image = cv::imread((data_path / "images" / "0.png").string());

  // Load the camera intrinsics
  YAML::Node intr_config = YAML::LoadFile((data_path / "camera_intr.yaml").string());

  // Load the target finder
  YAML::Node target_finder_config = YAML::LoadFile((data_path / "target_finder.yaml").string());
  auto target_finder = std::make_unique<ModifiedCircleGridTargetFinderPlugin>();
  target_finder->init(target_finder_config["target_finder"]);

  // Solve
  PnPProblem params;
  params.intr = intr_config["intrinsics"].as<CameraIntrinsics>();

  // Create an initial guess for the camera to target transform
  {
    const std::string pose_path = data_node[0]["pose"].as<std::string>();
    const Eigen::Isometry3d base_to_wrist = YAML::LoadFile((data_path / pose_path).string()).as<Eigen::Isometry3d>();

    YAML::Node pose_guess_config = YAML::LoadFile((data_path / "pose_initial_guesses.yaml").string());
    const Eigen::Isometry3d wrist_to_camera = pose_guess_config["wrist_to_camera_guess"].as<Eigen::Isometry3d>();
    const Eigen::Isometry3d base_to_target = pose_guess_config["base_to_target_guess"].as<Eigen::Isometry3d>();

    params.camera_to_target_guess = (base_to_wrist * wrist_to_camera).inverse() * base_to_target;
  }

  // Find correspondences between the known target and the features in the image
  params.correspondences = target_finder->findCorrespondences(image);

#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING
  // Display the features
  cv::namedWindow(WINDOW, cv::WINDOW_NORMAL);

  TargetFeatures target_features = target_finder->findTargetFeatures(image);
  cv::imshow(WINDOW, target_finder->drawTargetFeatures(image, target_features));
  cv::waitKey();
#endif

  PnPResult pnp_result = optimize(params);

  printOptResults(pnp_result.converged, pnp_result.initial_cost_per_obs, pnp_result.final_cost_per_obs);
  std::cout << std::endl;

  printTransform(pnp_result.camera_to_target, "Camera", "Target", "INDUSTRIAL CALIBRATION: CAMERA TO TARGET");
  std::cout << std::endl;

  // Solve with OpenCV for comparison
  const Eigen::Isometry3d camera_to_target_cv = solveCVPnP(params.intr, params.correspondences);

  printTransform(camera_to_target_cv, "Camera", "Target", "OPENCV: CAMERA TO TARGET");
  std::cout << std::endl;

  return std::make_tuple(pnp_result, camera_to_target_cv);
}

#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING

int main(int argc, char** argv)
{
  try
  {
    run();
  }
  catch (const std::exception& ex)
  {
    std::cerr << ex.what() << std::endl;
    return -1;
  }

  return 0;
}

#else

#include <gtest/gtest.h>

TEST(PnPExample, CalibratePnP)
{
  PnPResult result;
  Eigen::Isometry3d camera_to_target_cv;
  ASSERT_NO_THROW(std::tie(result, camera_to_target_cv) = run());

  // Expect that the optimization converged with low residual error (in pixels)
  ASSERT_TRUE(result.converged);
  ASSERT_LT(std::sqrt(result.final_cost_per_obs), 0.5);  // pixels

  // Compute the difference between the industrial calibration and OpenCV PnP transforms
  const Eigen::Isometry3d diff = camera_to_target_cv.inverse() * result.camera_to_target;
  const double pos_diff = diff.translation().norm();
  const double ori_diff = Eigen::Quaterniond(camera_to_target_cv.linear())
                              .angularDistance(Eigen::Quaterniond(result.camera_to_target.linear()));

  // Expect that the difference between the OpenCV and Industrial Calibration PnP results is very small
  ASSERT_LT(pos_diff, 1.0e-4);              // meters
  ASSERT_LT(ori_diff, 0.1 * M_PI / 180.0);  // radians
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#endif
