#include "utils.h"
#include <industrial_calibration/optimizations/pnp.h>
#include <industrial_calibration/target_finders/target_finder.h>
#include <industrial_calibration/core/serialization.h>

#include <boost_plugin_loader/plugin_loader.hpp>
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
std::tuple<PnPResult, Eigen::Isometry3d> run(const path& calibration_file)
{
  YAML::Node config = YAML::LoadFile(calibration_file.string());

  auto data = getMember<YAML::Node>(config, "data");

  // Load the image
  auto image_relative_path = getMember<std::string>(data[0], "image");
  const cv::Mat image = cv::imread((calibration_file.parent_path() / image_relative_path).string());

  // Load the target finder
  boost_plugin_loader::PluginLoader loader;
  loader.search_libraries.insert(INDUSTRIAL_CALIBRATION_PLUGIN_LIBRARIES);
  loader.search_libraries_env = INDUSTRIAL_CALIBRATION_SEARCH_LIBRARIES_ENV;

  auto target_finder_config = getMember<YAML::Node>(config, "target_finder");
  auto factory = loader.createInstance<TargetFinderFactory>(getMember<std::string>(target_finder_config, "type"));
  auto target_finder = factory->create(target_finder_config);

  // Solve
  PnPProblem problem;

  // Load the camera intrinsics
  problem.intr = getMember<CameraIntrinsics>(config, "intrinsics");

  // Create an initial guess for the camera to target transform
  {
    auto pose_relative_path = getMember<std::string>(data[0], "pose");
    const Eigen::Isometry3d target_mount_to_camera_mount =
        YAML::LoadFile((calibration_file.parent_path() / pose_relative_path).string()).as<Eigen::Isometry3d>();
    auto camera_mount_to_camera = getMember<Eigen::Isometry3d>(config, "camera_mount_to_camera_guess");
    auto target_mount_to_target = getMember<Eigen::Isometry3d>(config, "target_mount_to_target_guess");

    problem.camera_to_target_guess =
        (target_mount_to_camera_mount * camera_mount_to_camera).inverse() * target_mount_to_target;
  }

  // Find correspondences between the known target and the features in the image
  problem.correspondences = target_finder->findCorrespondences(image);

#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING
  // Display the features
  cv::namedWindow(WINDOW, cv::WINDOW_NORMAL);

  TargetFeatures target_features = target_finder->findTargetFeatures(image);
  cv::imshow(WINDOW, target_finder->drawTargetFeatures(image, target_features));
  cv::waitKey();
#endif

  PnPResult pnp_result = optimize(problem);

  printOptResults(pnp_result.converged, pnp_result.initial_cost_per_obs, pnp_result.final_cost_per_obs);
  std::cout << std::endl;

  printTransform(pnp_result.camera_to_target, "Camera", "Target", "INDUSTRIAL CALIBRATION: CAMERA TO TARGET");
  std::cout << std::endl;

  // Solve with OpenCV for comparison
  const Eigen::Isometry3d camera_to_target_cv = solveCVPnP(problem.intr, problem.correspondences);

  printTransform(camera_to_target_cv, "Camera", "Target", "OPENCV: CAMERA TO TARGET");
  std::cout << std::endl;

  return std::make_tuple(pnp_result, camera_to_target_cv);
}

#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING

int main(int argc, char** argv)
{
  try
  {
    const path calibration_file = path(EXAMPLE_DATA_DIR) / path("test_set_10x10") / "cal_data.yaml";
    run(calibration_file);
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
  const path calibration_file = path(EXAMPLE_DATA_DIR) / path("test_set_10x10") / "cal_data.yaml";
  PnPResult result;
  Eigen::Isometry3d camera_to_target_cv;
  ASSERT_NO_THROW(std::tie(result, camera_to_target_cv) = run(calibration_file));

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
