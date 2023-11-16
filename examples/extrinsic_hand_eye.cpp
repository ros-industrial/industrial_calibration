#include <industrial_calibration/optimizations/extrinsic_hand_eye.h>
#include <industrial_calibration/optimizations/analysis/homography_analysis.h>
#include <industrial_calibration/optimizations/analysis/statistics.h>
#include <industrial_calibration/optimizations/pnp.h>
#include <industrial_calibration/target_finders/charuco_grid_target_finder.h>
#include <industrial_calibration/target_finders/modified_circle_grid_target_finder.h>
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
#include <memory>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

static const std::string WINDOW = "window";
static const unsigned RANDOM_SEED = 1;

using namespace industrial_calibration;
using VectorEigenIsometry = std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>;

/**
 * @brief Performs a PnP optimization for comparison to the calibrated camera to target transform
 * @param camera_to_target - calibrated transformation from the camera frame to the target frame
 * @param correspondence_set
 * @param intr
 * @return
 */
Eigen::Isometry3d estimatePnP(const Eigen::Isometry3d& camera_to_target,
                              const Correspondence2D3D::Set& correspondence_set, const CameraIntrinsics& intr)
{
  PnPProblem pb;
  pb.camera_to_target_guess = camera_to_target;
  pb.correspondences = correspondence_set;
  pb.intr = intr;
  PnPResult r = optimize(pb);

  return r.camera_to_target;
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
  std::vector<Eigen::Vector3d> target_points;
  target_points.reserve(correspondence_set.size());
  std::transform(correspondence_set.begin(), correspondence_set.end(), std::back_inserter(target_points),
                 [](const Correspondence2D3D& corr) { return corr.in_target; });
  std::vector<cv::Point2d> reprojections = getReprojections(camera_to_target, intr, target_points);

  cv::Mat frame = image.clone();
  drawReprojections(reprojections, 3, cv::Scalar(0, 0, 255), frame);
  cv::imshow(WINDOW, frame);
  cv::waitKey();
}

struct PnPComparisonStats
{
  double pos_diff_mean;
  double pos_diff_stdev;
  double ori_diff_mean;
  double ori_diff_stdev;
};

/**
 * @brief Analyzes the results of the hand eye calibration by measuring the difference between the calibrated camera to
 * target transform and a PnP optimization estimation of the same transform
 * @param problem
 * @param opt_result
 * @param images
 * @param window_name
 */
PnPComparisonStats analyzeResults(const ExtrinsicHandEyeProblem2D3D& problem, const ExtrinsicHandEyeResult& opt_result,
                                  const std::vector<cv::Mat>& images)
{
  // Create accumulators to more easily calculate the mean and standard deviation of the position and orientation
  // differences
  std::vector<double> pos_diff_acc, ori_diff_acc;
  pos_diff_acc.reserve(images.size());
  ori_diff_acc.reserve(images.size());

  // Iterate over all of the images in which an observation of the target was made
  for (std::size_t i = 0; i < images.size(); ++i)
  {
    // Get the observation
    const Observation2D3D& obs = problem.observations.at(i);

    // Calculate the optimized transform from the camera to the target for the ith observation
    Eigen::Isometry3d camera_to_target = opt_result.camera_mount_to_camera.inverse() * obs.to_camera_mount.inverse() *
                                         obs.to_target_mount * opt_result.target_mount_to_target;

#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING
    // Reproject the target points into the image using the results of the calibration
    reproject(camera_to_target, obs.correspondence_set, problem.intr, images[i]);
#endif

    // Get the same transformation from a PnP optimization with the known camera intrinsic parameters
    Eigen::Isometry3d camera_to_target_pnp = estimatePnP(camera_to_target, obs.correspondence_set, problem.intr);

    // Calculate the difference between the two transforms
    Eigen::Isometry3d diff = camera_to_target.inverse() * camera_to_target_pnp;

    // Accumulate the differences
    pos_diff_acc.push_back(diff.translation().norm());
    ori_diff_acc.push_back(Eigen::Quaterniond(camera_to_target.linear())
                               .angularDistance(Eigen::Quaterniond(camera_to_target_pnp.linear())));
  }

  PnPComparisonStats stats;
  std::tie(stats.pos_diff_mean, stats.pos_diff_stdev) = computeStats(pos_diff_acc);
  std::tie(stats.ori_diff_mean, stats.ori_diff_stdev) = computeStats(ori_diff_acc);

  return stats;
}

struct Params
{
  double homography_threshold{ 2.0 };
  CameraIntrinsics intr;
  TargetFinder::ConstPtr target_finder;
  std::vector<cv::Mat> images;
  VectorEigenIsometry poses;
  Eigen::Isometry3d target_mount_to_target_guess{ Eigen::Isometry3d::Identity() };
  Eigen::Isometry3d camera_mount_to_camera_guess{ Eigen::Isometry3d::Identity() };
};

using ObservationGenerator = std::function<Observation2D3D(const Eigen::Isometry3d&, const Correspondence2D3D::Set&)>;

std::tuple<ExtrinsicHandEyeResult, PnPComparisonStats> run(const Params& params, ObservationGenerator obs_gen)
{
  // Now we create our calibration problem
  ExtrinsicHandEyeProblem2D3D problem;
  problem.intr = params.intr;  // Set the camera properties

  // Our 'base to camera guess': A camera off to the side, looking at a point centered in front of the robot
  problem.target_mount_to_target_guess = params.target_mount_to_target_guess;
  problem.camera_mount_to_camera_guess = params.camera_mount_to_camera_guess;

  // Create a named OpenCV window for viewing the images
#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING
  cv::namedWindow(WINDOW, cv::WINDOW_NORMAL);
#endif

  // Finally, we need to process our images into correspondence sets: for each dot in the
  // target this will be where that dot is in the target and where it was seen in the image.
  // Repeat for each image. We also tell where the wrist was when the image was taken.
  problem.observations.reserve(params.images.size());

  // The target may not be identified in all images, so let's keep track the indices of the images for which the
  // target was identified
  std::vector<cv::Mat> found_images;
  found_images.reserve(params.images.size());

  for (std::size_t i = 0; i < params.images.size(); ++i)
  {
    // For each image we need to:
    try
    {
      // Try to find the correspondences with the target features in this image:
      Observation2D3D obs = obs_gen(params.poses[i], params.target_finder->findCorrespondences(params.images[i]));

      // Check that a homography matrix can accurately reproject the observed points onto the expected target points
      // within a defined threshold
      RandomCorrespondenceSampler random_sampler(obs.correspondence_set.size(), obs.correspondence_set.size() / 3,
                                                 RANDOM_SEED);
      Eigen::VectorXd homography_error = calculateHomographyError(obs.correspondence_set, random_sampler);
      if (homography_error.array().mean() > params.homography_threshold)
        throw std::runtime_error("Homography error exceeds threshold (" +
                                 std::to_string(homography_error.array().mean()) + ")");

      // Add the observations to the problem
      problem.observations.push_back(obs);
      found_images.push_back(params.images[i]);

#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING
      // Show the points we detected
      TargetFeatures target_features = params.target_finder->findTargetFeatures(params.images[i]);
      cv::imshow(WINDOW, params.target_finder->drawTargetFeatures(params.images[i], target_features));
      cv::waitKey();
#endif
    }
    catch (const std::runtime_error& ex)
    {
      std::cerr << "Image " << i << ": '" << ex.what() << "'" << std::endl;
#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING
      cv::imshow(WINDOW, params.images[i]);
      cv::waitKey();
#endif
      continue;
    }
  }

  // Now we have a defined problem, run optimization:
  ExtrinsicHandEyeResult opt_result = optimize(problem);

  // Report results
  std::cout << std::endl;
  printOptResults(opt_result.converged, opt_result.initial_cost_per_obs, opt_result.final_cost_per_obs);
  std::cout << std::endl;

  Eigen::Isometry3d c = opt_result.camera_mount_to_camera;
  printTransform(c, "Camera Mount", "Camera", "CAMERA MOUNT TO CAMERA");
  std::cout << std::endl;

  Eigen::Isometry3d t = opt_result.target_mount_to_target;
  printTransform(t, "Target Mount", "Target", "TARGET MOUNT TO TARGET");
  std::cout << std::endl;

  std::cout << opt_result.covariance.printCorrelationCoeffAboveThreshold(0.5) << std::endl;

  // Now let's compare the results of our extrinsic calibration with a PnP optimization for every observation.
  // The PnP optimization will give us an estimate of the camera to target transform using our input camera intrinsic
  // parameters We will then see how much this transform differs from the same transform calculated using the results
  // of the extrinsic calibration
  PnPComparisonStats stats = analyzeResults(problem, opt_result, found_images);

  std::cout << "Difference in camera to target transform between extrinsic calibration and PnP optimization"
            << std::endl;
  std::cout << "Position:\n\tMean (m): " << stats.pos_diff_mean << "\n\tStd. Dev. (m): " << stats.pos_diff_stdev
            << std::endl;
  std::cout << "Orientation:\n\tMean (deg): " << stats.ori_diff_mean * 180.0 / M_PI
            << "\n\tStd. Dev. (deg): " << stats.ori_diff_stdev * 180.0 / M_PI << std::endl;

  return std::make_tuple(opt_result, stats);
}

Params loadModifiedCircleGridCalibrationData()
{
  Params params;

  const path data_dir = path(EXAMPLE_DATA_DIR) / path("test_set_10x10");

  // Load the pose guesses
  YAML::Node pose_guesses = YAML::LoadFile((data_dir / "pose_initial_guesses.yaml").string());
  params.target_mount_to_target_guess = pose_guesses["base_to_target_guess"].as<Eigen::Isometry3d>();
  params.camera_mount_to_camera_guess = pose_guesses["wrist_to_camera_guess"].as<Eigen::Isometry3d>();

  // Load the images and poses
  std::tie(params.poses, params.images) = loadPoseImagePairs(data_dir);

  // Load the camera intrinsics
  YAML::Node intr = YAML::LoadFile((data_dir / "camera_intr.yaml").string())["intrinsics"];
  params.intr = intr.as<CameraIntrinsics>();

  // Load the target finder
  YAML::Node target_finder_config = YAML::LoadFile((data_dir / "target_finder.yaml").string());
  ModifiedCircleGridTargetFinderFactory factory;
  params.target_finder = factory.create(target_finder_config);

  return params;
}

Params loadCharucoGridCalibrationData()
{
  Params params;

  const path data_dir = path(EXAMPLE_DATA_DIR) / path("test_set_charuco");

  // Load the pose guesses
  YAML::Node pose_guesses = YAML::LoadFile((data_dir / "pose_initial_guesses.yaml").string());
  params.target_mount_to_target_guess = pose_guesses["wrist_to_target_guess"].as<Eigen::Isometry3d>();
  params.camera_mount_to_camera_guess = pose_guesses["base_to_camera_guess"].as<Eigen::Isometry3d>();

  // Load the images and poses
  std::tie(params.poses, params.images) = loadPoseImagePairs(data_dir);

  // Load the camera intrinsics
  YAML::Node intr = YAML::LoadFile((data_dir / "camera_intr.yaml").string())["intrinsics"];
  params.intr = intr.as<CameraIntrinsics>();

  // Load the target finder
  YAML::Node target_finder_config = YAML::LoadFile((data_dir / "target_finder.yaml").string());
  CharucoGridTargetFinderFactory factory;
  params.target_finder = factory.create(target_finder_config);

  return params;
}

Observation2D3D createCameraOnWristObservation(const Eigen::Isometry3d& pose,
                                               const Correspondence2D3D::Set& correspondences)
{
  Observation2D3D obs;
  obs.to_camera_mount = pose;
  obs.to_target_mount = Eigen::Isometry3d::Identity();
  obs.correspondence_set = correspondences;
  return obs;
}

Observation2D3D createTargetOnWristObservation(const Eigen::Isometry3d& pose,
                                               const Correspondence2D3D::Set& correspondences)
{
  Observation2D3D obs;
  obs.to_camera_mount = Eigen::Isometry3d::Identity();
  obs.to_target_mount = pose;
  obs.correspondence_set = correspondences;
  return obs;
}

#ifndef INDUSTRIAL_CALIBRATION_ENABLE_TESTING

int main(int argc, char** argv)
{
  try
  {
    // Modified circle grid target
    printTitle("Camera on wrist, modified circle grid target");
    run(loadModifiedCircleGridCalibrationData(), createCameraOnWristObservation);

    // ChArUco grid target
    printTitle("Target on wrist, ChArUco grid target");
    run(loadCharucoGridCalibrationData(), createTargetOnWristObservation);
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
  ExtrinsicHandEyeResult result;
  PnPComparisonStats stats;
  std::tie(result, stats) = run(loadModifiedCircleGridCalibrationData(), createCameraOnWristObservation);

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
  ExtrinsicHandEyeResult result;
  PnPComparisonStats stats;
  std::tie(result, stats) = run(loadCharucoGridCalibrationData(), createTargetOnWristObservation);

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
