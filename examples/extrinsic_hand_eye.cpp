#include <industrial_calibration/optimizations/extrinsic_hand_eye.h>
#include <industrial_calibration/optimizations/ceres_math_utilities.h>
#include <industrial_calibration/analysis/homography_analysis.h>
#include <industrial_calibration/analysis/extrinsic_hand_eye_calibration_analysis.h>
#include <industrial_calibration/target_finders/target_finder.h>
#include <industrial_calibration/target_finders/utils.h>
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

std::tuple<ExtrinsicHandEyeResult, ExtrinsicHandEyeAnalysisStats> run(const path& calibration_file)
{
  // Now we create our calibration problem
  ExtrinsicHandEyeProblem2D3D problem;

  YAML::Node config = YAML::LoadFile(calibration_file.string());

  // Load the flag that indicates whether the camera was static during the calibration
  bool static_camera = getMember<bool>(config, "static_camera");

  // Load the homography threshold
  auto homography_threshold = getMember<double>(config, "homography_threshold");

  // Load the pose guesses
  problem.target_mount_to_target_guess = getMember<Eigen::Isometry3d>(config, "target_mount_to_target_guess");
  problem.camera_mount_to_camera_guess = getMember<Eigen::Isometry3d>(config, "camera_mount_to_camera_guess");

  // Load the images and poses
  VectorEigenIsometry poses;
  std::vector<cv::Mat> images;
  std::tie(poses, images) = loadPoseImagePairs(calibration_file.parent_path(), getMember<YAML::Node>(config, "data"));

  // Load the camera intrinsics
  problem.intr = getMember<CameraIntrinsics>(config, "intrinsics");

  // Load the target finder
  boost_plugin_loader::PluginLoader loader;
  loader.search_libraries.insert(INDUSTRIAL_CALIBRATION_PLUGIN_LIBRARIES);
  loader.search_libraries_env = INDUSTRIAL_CALIBRATION_SEARCH_LIBRARIES_ENV;

  YAML::Node target_finder_config = getMember<YAML::Node>(config, "target_finder");
  auto factory = loader.createInstance<TargetFinderFactory2D3D>(getMember<std::string>(target_finder_config, "type"));
  TargetFinder2D3D::ConstPtr target_finder = factory->create(target_finder_config);

  // Create a named OpenCV window for viewing the images
#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING
  cv::namedWindow(WINDOW, cv::WINDOW_NORMAL);
#endif

  // Finally, we need to process our images into correspondence sets: for each dot in the
  // target this will be where that dot is in the target and where it was seen in the image.
  // Repeat for each image. We also tell where the wrist was when the image was taken.
  problem.observations.reserve(images.size());

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
      Observation2D3D obs;
      if (static_camera)
      {
        obs.to_camera_mount = Eigen::Isometry3d::Identity();
        obs.to_target_mount = poses[i];
      }
      else
      {
        obs.to_camera_mount = poses[i];
        obs.to_target_mount = Eigen::Isometry3d::Identity();
      }
      obs.correspondence_set = target_finder->findCorrespondences(images[i]);

      // Check that a homography matrix can accurately reproject the observed points onto the expected target points
      // within a defined threshold
      RandomCorrespondenceSampler random_sampler(obs.correspondence_set.size(), obs.correspondence_set.size() / 3,
                                                 RANDOM_SEED);
      Eigen::VectorXd homography_error = calculateHomographyError(obs.correspondence_set, random_sampler);
      if (homography_error.array().mean() > homography_threshold)
        throw std::runtime_error("Homography error exceeds threshold (" +
                                 std::to_string(homography_error.array().mean()) + ")");

      // Add the observations to the problem
      problem.observations.push_back(obs);
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
  ExtrinsicHandEyeResult opt_result = optimize(problem);

  // Report results
  std::cout << std::endl << opt_result << std::endl;
  std::cout << opt_result.covariance.printCorrelationCoeffAboveThreshold(0.5) << std::endl;

  // Compute the projected 3D error for comparison
  std::cout << analyze3dProjectionError(problem, opt_result) << std::endl << std::endl;

  // Now let's compare the results of our extrinsic calibration with a PnP optimization for every observation.
  // The PnP optimization will give us an estimate of the camera to target transform using our input camera intrinsic
  // parameters We will then see how much this transform differs from the same transform calculated using the results
  // of the extrinsic calibration
  ExtrinsicHandEyeAnalysisStats stats = analyzeResults(problem, opt_result);
  std::cout << stats << std::endl << std::endl;

#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING
  // Reproject the target points into the image using the results of the calibration and visualize
  for (std::size_t i = 0; i < problem.observations.size(); ++i)
  {
    const Observation2D3D& obs = problem.observations.at(i);

    // Calculate the optimized transform from the camera to the target for the ith observation
    Eigen::Isometry3d camera_to_target = opt_result.camera_mount_to_camera.inverse() * obs.to_camera_mount.inverse() *
                                         obs.to_target_mount * opt_result.target_mount_to_target;

    reproject(camera_to_target, obs.correspondence_set, problem.intr, images.at(found_images.at(i)));
  }
#endif

  return std::make_tuple(opt_result, stats);
}

#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING

int main(int argc, char** argv)
{
  try
  {
    // Modified circle grid target
    {
      std::cout << std::endl << "Camera on wrist, modified circle grid target" << std::endl << std::endl;
      const path calibration_file = path(EXAMPLE_DATA_DIR) / path("test_set_10x10") / "cal_data.yaml";
      run(calibration_file.string());
    }

    // ChArUco grid target
    {
      const path calibration_file = path(EXAMPLE_DATA_DIR) / path("test_set_charuco") / "cal_data.yaml";
      std::cout << std::endl << "Target on wrist, ChArUco target" << std::endl << std::endl;
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

  ExtrinsicHandEyeResult result;
  ExtrinsicHandEyeAnalysisStats stats;
  std::tie(result, stats) = run(calibration_file);

  // Expect the optimization to converge with low* residual error (in pixels)
  // Note: the camera parameters used in this calibration were un-calibrated values and the images are somewhat low
  // resolution. Therefore the results of the calibration are not particularly good
  ASSERT_TRUE(result.converged);
  ASSERT_LT(std::sqrt(result.final_cost_per_obs), 1.5);  // pixels

  // Expect a low* average difference between the camera to target pose estimated by this calibration vs. a PnP
  // optimization
  ASSERT_LT(stats.pos_diff_mean, 0.02);                // meters
  ASSERT_LT(stats.ori_diff_mean, 3.0 * M_PI / 180.0);  // radians
}

TEST(ExtrinsicHandEyeCalibration, ChArUcoGridTarget)
{
  const path calibration_file = path(EXAMPLE_DATA_DIR) / path("test_set_charuco") / "cal_data.yaml";

  ExtrinsicHandEyeResult result;
  ExtrinsicHandEyeAnalysisStats stats;
  std::tie(result, stats) = run(calibration_file);

  // Expect the optimization to converge with low residual error (in pixels)
  ASSERT_TRUE(result.converged);
  ASSERT_LT(std::sqrt(result.final_cost_per_obs), 1.5);  // pixels

  // Expect a low average difference between the camera to target pose estimated by this calibration vs. a PnP
  // optimization
  ASSERT_LT(stats.pos_diff_mean, 0.005);               // meters
  ASSERT_LT(stats.ori_diff_mean, 1.5 * M_PI / 180.0);  // radians
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#endif
