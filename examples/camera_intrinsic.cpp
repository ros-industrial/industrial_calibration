#include <industrial_calibration/optimizations/camera_intrinsic.h>
#include <industrial_calibration/optimizations/ceres_math_utilities.h>
#include <industrial_calibration/analysis/homography_analysis.h>
#include <industrial_calibration/analysis/camera_intrinsic_calibration_analysis.h>
#include <industrial_calibration/analysis/statistics.h>
#include <industrial_calibration/target_finders/opencv/target_finder.h>
#include <industrial_calibration/target_finders/opencv/utils.h>
#include <industrial_calibration/core/serialization.h>

#include <boost_plugin_loader/plugin_loader.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#if __has_include(<filesystem>)
#include <filesystem>
using path = std::filesystem::path;
#else
#include <experimental/filesystem>
using path = std::experimental::filesystem::path;
#endif

static const std::string WINDOW = "window";
static const unsigned RANDOM_SEED = 1;

using namespace industrial_calibration;

std::tuple<VectorEigenIsometry, std::vector<cv::Mat>> loadPoseImagePairs(const path& data_dir, const YAML::Node& data)
{
  VectorEigenIsometry poses;
  std::vector<cv::Mat> images;

  // Load the images and poses
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

/**
 * @brief Reprojects target points into the image using the calibrated transform
 * @param camera_to_target - calibrated transformation from the camera frame to the target frame
 * @param correspondence_set
 * @param intr
 * @param image
 * @param window_name
 * @return
 */
void reproject(const Eigen::Isometry3d& camera_to_target, const Correspondence2D3D::Set& correspondence_set,
               const CameraIntrinsics& intr, const cv::Mat& image)
{
  VectorVector3<double> target_points;
  target_points.reserve(correspondence_set.size());
  std::transform(correspondence_set.begin(), correspondence_set.end(), std::back_inserter(target_points),
                 [](const Correspondence2D3D& corr) { return corr.in_target; });
  VectorVector2<double> reprojections = projectPoints<double>(camera_to_target, intr, target_points);

  cv::Mat frame = image.clone();
  drawReprojections(reprojections, 3, cv::Scalar(0, 0, 255), frame);
  cv::imshow(WINDOW, frame);
  cv::waitKey();
}

std::pair<CameraIntrinsicResult, IntrinsicCalibrationAccuracyResult> run(const path& calibration_file)
{
  // Now we create our calibration problem
  CameraIntrinsicProblem problem;

  YAML::Node config = YAML::LoadFile(calibration_file.string());

  // Load the homography threshold
  auto homography_threshold = getMember<double>(config, "homography_threshold");

  // Load the images and poses
  std::vector<cv::Mat> images;
  std::tie(problem.extrinsic_guesses, images) =
      loadPoseImagePairs(calibration_file.parent_path(), getMember<YAML::Node>(config, "data"));

  // Load the camera intrinsics guess
  problem.intrinsics_guess = getMember<CameraIntrinsics>(config, "intrinsics");

  // Flag to indicate whether or not to utilize the estimates of the camera to target transforms
  problem.use_extrinsic_guesses = getMember<bool>(config, "use_extrinsic_guesses");

  // Load the target finder
  boost_plugin_loader::PluginLoader loader;
  loader.search_libraries.insert(INDUSTRIAL_CALIBRATION_PLUGIN_LIBRARIES);
  loader.search_libraries_env = INDUSTRIAL_CALIBRATION_SEARCH_LIBRARIES_ENV;

  YAML::Node target_finder_config = getMember<YAML::Node>(config, "target_finder");
  auto factory = loader.createInstance<TargetFinderFactoryOpenCV>(getMember<std::string>(target_finder_config, "type"));
  TargetFinderOpenCV::ConstPtr target_finder = factory->create(target_finder_config);

  // Create a named OpenCV window for viewing the images
#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING
  cv::namedWindow(WINDOW, cv::WINDOW_NORMAL);
#endif

  // Finally, we need to process our images into correspondence sets: for each feature in the
  // target this will be where that feature is in the target and where it was seen in the image.
  problem.image_observations.reserve(images.size());

  // The target may not be identified in all images, so let's keep track the indices of the images for which the
  // target was identified
  std::vector<std::size_t> found_images;
  found_images.reserve(images.size());

  for (std::size_t i = 0; i < images.size(); ++i)
  {
    // For each image we need to:
    try
    {
      // Try to find the correspondences with the target features in this image:
      Correspondence2D3D::Set correspondence_set = target_finder->findCorrespondences(images[i]);

      // Check that a homography matrix can accurately reproject the observed points onto the expected target points
      // within a defined threshold
      RandomCorrespondenceSampler random_sampler(correspondence_set.size(), correspondence_set.size() / 2, RANDOM_SEED);
      Eigen::VectorXd homography_error = calculateHomographyError(correspondence_set, random_sampler);
      if (homography_error.array().mean() > homography_threshold)
        throw std::runtime_error("Homography error exceeds threshold (" +
                                 std::to_string(homography_error.array().mean()) + ")");

      // Add the observations to the problem
      problem.image_observations.push_back(correspondence_set);
      found_images.push_back(i);

#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING
      // Show the points we detected
      TargetFeatures2D target_features = target_finder->findTargetFeatures(images[i]);
      cv::imshow(WINDOW, target_finder->drawTargetFeatures(images[i], target_features));
      cv::waitKey();
#endif
    }
    catch (const std::runtime_error& ex)
    {
      std::cerr << "Image " << i << ": '" << ex.what() << "'" << std::endl;
#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING
      cv::imshow(WINDOW, images[i]);
      cv::waitKey();
#endif
      continue;
    }
  }

  // Now we have a defined problem, run optimization:
  CameraIntrinsicResult opt_result = optimize(problem);

  // Analyze the results
  IntrinsicCalibrationAccuracyResult accuracy_result;
  {
    // Create accumulators for mean and variance
    std::vector<double> pos_acc, ang_acc;
    pos_acc.reserve(problem.image_observations.size());
    ang_acc.reserve(problem.image_observations.size());

    // Compute the virtual target difference for each observation
    for (std::size_t i = 0; i < problem.image_observations.size(); ++i)
    {
      VirtualCorrespondenceResult vcr = measureVirtualTargetDiff(problem.image_observations[i], opt_result.intrinsics,
                                                                 opt_result.target_transforms[i], 1.0);
      pos_acc.push_back(vcr.positional_error);
      ang_acc.push_back(vcr.angular_error);
    }

    // Calculate the mean and variance of the measurements
    // Theoretically each transform from virtual target 1 to virtual target 2 should be zero; thus the mean should be
    // zero In practice the mean represents bias from the ideal zero state and the variance is the amount of change
    // around the mean
    std::tie(accuracy_result.pos_error.first, accuracy_result.pos_error.second) = computeStats(pos_acc);
    std::tie(accuracy_result.ang_error.first, accuracy_result.ang_error.second) = computeStats(ang_acc);
  }

  // Report results
  std::cout << std::endl << opt_result << std::endl;
  std::cout << std::setprecision(4) << "\n" << accuracy_result << std::endl;
  std::cout << opt_result.covariance.printCorrelationCoeffAboveThreshold(0.5) << std::endl;

#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING
  // Reproject the target points into the image using the results of the calibration and visualize
  for (std::size_t i = 0; i < problem.image_observations.size(); ++i)
  {
    reproject(opt_result.target_transforms[i], problem.image_observations[i], opt_result.intrinsics,
              images.at(found_images.at(i)));
  }
#endif

  return std::make_pair(opt_result, accuracy_result);
}

#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING

int main(int argc, char** argv)
{
  try
  {
    // Modified circle grid target
    {
      std::cout << std::endl << "Camera intrinsic calibration, modified circle grid target" << std::endl << std::endl;
      const path calibration_file = path(EXAMPLE_DATA_DIR) / path("test_set_10x10") / "cal_data.yaml";
      run(calibration_file.string());
    }

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

TEST(ExtrinsicHandEyeCalibration, ModifiedCircleGridTarget)
{
  const path calibration_file = path(EXAMPLE_DATA_DIR) / path("test_set_10x10") / "cal_data.yaml";

  CameraIntrinsicResult result;
  IntrinsicCalibrationAccuracyResult accuracy;
  std::tie(result, accuracy) = run(calibration_file);

  // Expect the optimization to converge with low* residual error (in pixels)
  EXPECT_TRUE(result.converged);
  EXPECT_LT(std::sqrt(result.final_cost_per_obs), 0.5);  // pixels

  // Expect the PnP-relative accuracy to be somewhat low due to the low quality of the images
  EXPECT_LT(accuracy.pos_error.first, 0.010);   // position error mean
  EXPECT_LT(accuracy.pos_error.second, 0.010);  // position error stdev

  EXPECT_LT(accuracy.ang_error.first, 2.0 * M_PI / 180.0);   // position angular mean
  EXPECT_LT(accuracy.ang_error.second, 2.0 * M_PI / 180.0);  // position angular stdev
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#endif
