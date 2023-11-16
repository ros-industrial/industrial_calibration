#include <industrial_calibration/optimizations/analysis/noise_qualification.h>
#include <industrial_calibration/target_finders/target_finder_plugin.h>
#include <industrial_calibration/target_finders/utils/utils.h>

#if __GNUC__ >= 8
#include <filesystem>
using path = std::filesystem::path;
#else
#include <experimental/filesystem>
using path = std::experimental::filesystem::path;
#endif

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <yaml-cpp/yaml.h>

using namespace industrial_calibration;

std::string WINDOW = "window";

PnPNoiseStat run()
{
  // Parse parameters
  path data_path = path(EXAMPLE_DATA_DIR) / path("noise_qualification");

  // Load the target finder
  YAML::Node target_finder_config = YAML::LoadFile((data_path / "target_finder.yaml").string());
  auto target_finder = std::make_unique<ModifiedCircleGridTargetFinderPlugin>();
  target_finder->init(target_finder_config["target_finder"]);

  // Load camera intrinsics
  YAML::Node camera_params = YAML::LoadFile((data_path / "camera_intr.yaml").string());
  CameraIntrinsics camera = camera_params["intrinsics"].as<CameraIntrinsics>();

  // Load an initial guess for the camera to target transformation
  YAML::Node pose_params = YAML::LoadFile((data_path / "camera_to_target_guess.yaml").string());
  Eigen::Isometry3d camera_to_target_guess = pose_params["camera_to_target_guess"].as<Eigen::Isometry3d>();

  // Load the data file which specifies the location of the images on which to perform the noise qualification
  YAML::Node root = YAML::LoadFile((data_path / "data.yaml").string());

#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING
  cv::namedWindow(WINDOW, cv::WINDOW_NORMAL);
#endif

  // Set up the noise qualification inputs
  std::vector<PnPProblem> problem_set;
  problem_set.reserve(root.size());
  for (std::size_t i = 0; i < root.size(); ++i)
  {
    // Each entry should have an image path. This path is relative to the root_path directory!
    const auto img_path = root[i]["image"].as<std::string>();
    static cv::Mat image = readImageOpenCV((data_path / img_path).string());

    // Find the observations in the image
    TargetFeatures target_features;
    try
    {
      target_features = target_finder->findTargetFeatures(image);
      if (target_features.empty()) throw std::runtime_error("Failed to find any target features");
      std::cout << "Found " << target_features.size() << " target features" << std::endl;

#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING
      // Show the points we detected
      cv::imshow(WINDOW, target_finder->drawTargetFeatures(image, target_features));
      cv::waitKey();
#endif
    }
    catch (const std::runtime_error& ex)
    {
      std::cout << "Image " << i << ": '" << ex.what() << "'" << std::endl;
#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING
      cv::imshow(WINDOW, image);
      cv::waitKey();
#endif
      continue;
    }

    // Set up the PnP problem for this image
    PnPProblem problem;
    problem.intr = camera;
    problem.camera_to_target_guess = camera_to_target_guess;

    // Add the detected correspondences
    problem.correspondences = target_finder->target().createCorrespondences(target_features);

    problem_set.push_back(problem);
  }

  // Perform the noise qualification
  PnPNoiseStat result = qualifyNoise2D(problem_set);

  // Print the results
  Eigen::IOFormat fmt(4, 0, ",", "\n", "[", "]");
  std::cout << "Camera to Target Noise Results" << std::endl;
  std::cout << "Position mean (m): " << result.p_stat.mean.transpose().format(fmt) << std::endl;
  std::cout << "Position standard deviation (m): " << result.p_stat.stdev.transpose().format(fmt) << std::endl;
  std::cout << "Quaternion mean (qx, qy, qz, qw): " << result.q_stat.mean.coeffs().transpose().format(fmt) << std::endl;
  std::cout << "Quaternion standard deviation: " << result.q_stat.stdev << std::endl;

  return result;
}

#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING

int main(int argc, char** argv)
{
  try
  {
    run();
    return 0;
  }
  catch (const std::exception& ex)
  {
    std::cerr << ex.what() << std::endl;
    return -1;
  }
}

#else

#include <gtest/gtest.h>

TEST(NoiseQualification, NoiseQualification2D)
{
  PnPNoiseStat stats;
  ASSERT_NO_THROW(stats = run());

  ASSERT_LT(stats.p_stat.stdev.norm(), 0.0005);       // meters
  ASSERT_LT(stats.q_stat.stdev, 0.1 * M_PI / 180.0);  // radians
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#endif
