#include <industrial_calibration/optimizations/analysis/noise_qualification.h>
#include <industrial_calibration/target_finders/target_finder.h>
#include <industrial_calibration/utils.h>
#include <industrial_calibration/serialization.h>

#include <boost_plugin_loader/plugin_loader.hpp>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <yaml-cpp/yaml.h>

using namespace industrial_calibration;

std::string WINDOW = "window";

PnPNoiseStat run(const path& calibration_file)
{
  // Parse parameters
  YAML::Node config = YAML::LoadFile(calibration_file.string());

  // Load the target finder
  boost_plugin_loader::PluginLoader loader;
  loader.search_libraries.insert(INDUSTRIAL_CALIBRATION_PLUGIN_LIBRARIES);
  loader.search_libraries_env = INDUSTRIAL_CALIBRATION_SEARCH_LIBRARIES_ENV;

  auto target_finder_config = getMember<YAML::Node>(config, "target_finder");
  auto factory = loader.createInstance<TargetFinderFactory>(getMember<std::string>(target_finder_config, "type"));
  auto target_finder = factory->create(target_finder_config);

  // Load camera intrinsics
  auto camera = getMember<CameraIntrinsics>(config, "intrinsics");

  // Load an initial guess for the camera to target transformation
  auto camera_to_target_guess = getMember<Eigen::Isometry3d>(config, "camera_to_target_guess");

  // Load the data file which specifies the location of the images on which to perform the noise qualification
  auto data = getMember<YAML::Node>(config, "data");

#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING
  cv::namedWindow(WINDOW, cv::WINDOW_NORMAL);
#endif

  // Set up the noise qualification inputs
  std::vector<PnPProblem> problem_set;
  problem_set.reserve(data.size());
  for (std::size_t i = 0; i < data.size(); ++i)
  {
    // Each entry should have an image path. This path is relative to the root_path directory!
    const auto img_path = data[i]["image"].as<std::string>();
    static cv::Mat image = readImageOpenCV((calibration_file.parent_path() / img_path).string());

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
    const path calibration_file = path(EXAMPLE_DATA_DIR) / path("noise_qualification") / "cal_data.yaml";
    run(calibration_file);
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
  const path calibration_file = path(EXAMPLE_DATA_DIR) / path("noise_qualification") / "cal_data.yaml";

  PnPNoiseStat stats;
  ASSERT_NO_THROW(stats = run(calibration_file));

  ASSERT_LT(stats.p_stat.stdev.norm(), 0.0005);       // meters
  ASSERT_LT(stats.q_stat.stdev, 0.1 * M_PI / 180.0);  // radians
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#endif
