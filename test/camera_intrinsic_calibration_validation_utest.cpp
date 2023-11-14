#include <gtest/gtest.h>
#include <ical_core/optimizations/analysis/camera_intrinsic_calibration_analysis.h>
#include <ical_core_tests/utilities.h>
#include <ical_core_tests/observation_creator.h>
#include <memory>

using namespace industrial_calibration;

TEST(CameraIntrinsicCalibrationValidation, MeasureVirtualTargetDiff)
{
  test::Target target(8, 6, 0.025);
  test::Camera camera = test::makeKinectCamera();

  // Create a camera pose directly above the target
  Eigen::Isometry3d base_to_target(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d base_to_camera =
      base_to_target * Eigen::Translation3d(0.0, 0.0, 1.0) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

  Correspondence2D3D::Set corr_set;
  ASSERT_NO_THROW(corr_set = test::getCorrespondences(base_to_camera, base_to_target, camera, target, true));

  Eigen::Isometry3d camera_to_target = base_to_camera.inverse() * base_to_target;

  /* Since `getInternalTargetTransformation` simply splits the correspondences in half,
   * creating two virtual targets with different features relative to the same origin,
   * the expected transformation from one virtual target to the other should be identity */
  const Eigen::Isometry3d expected(Eigen::Isometry3d::Identity());

  // Perfect camera intrinsics, perfect camera to target guess
  {
    VirtualCorrespondenceResult result;
    EXPECT_NO_THROW(result = measureVirtualTargetDiff(corr_set, camera.intr, camera_to_target));

    // Expect the transform to be almost exactly as expected
    EXPECT_TRUE(result.t1_to_t2.isApprox(expected));
  }

  // Create a slightly noisy camera to target guess (1 cm positional and ~5 degrees orientation noise)
  Eigen::Isometry3d camera_to_target_guess = test::perturbPose(camera_to_target, 0.01, 0.01);

  // Perfect camera intrinsics, imperfect camera to target guess
  {
    VirtualCorrespondenceResult result;
    EXPECT_NO_THROW(result = measureVirtualTargetDiff(corr_set, camera.intr, camera_to_target_guess));

    // Expect the transform to be very close to its expected value, but with a small error from the PnP
    EXPECT_TRUE(result.t1_to_t2.isApprox(expected, 1.0e-8));
  }

  // Bad camera intrinsics, imperfect camera to target guess
  {
    // Tweak the camera intrinsics slightly (+/- 1%)
    camera.intr.fx() += 0.01 * camera.intr.fx();
    camera.intr.fy() -= 0.01 * camera.intr.fy();
    camera.intr.cx() -= 0.01 * camera.intr.cx();
    camera.intr.cy() += 0.01 * camera.intr.cy();

    VirtualCorrespondenceResult result;
    EXPECT_NO_THROW(result = measureVirtualTargetDiff(corr_set, camera.intr, camera_to_target_guess));

    // Expect the transform to not be very close to its expected value
    EXPECT_FALSE(result.t1_to_t2.isApprox(expected, 1.0e-8));
  }
}

TEST(CameraIntrinsicCalibrationValidation, MeasureIntrinsicCalibrationAccuracy)
{
  test::Camera camera = test::makeKinectCamera();
  test::Target target(5, 7, 0.025);
  auto pose_gen = std::make_shared<test::GridPoseGenerator>();

  // Create the relevant calibration transforms
  Eigen::Isometry3d camera_mount_to_camera(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d target_mount_to_target(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d camera_base_to_target_base(Eigen::Isometry3d::Identity());

  // Create some observations
  Observation2D3D::Set observations =
      test::createObservations(camera, target, { pose_gen }, target_mount_to_target, camera_mount_to_camera);

  // Validate the intrinsic calibration
  // Perfect intrinsic parameters, perfect transform guesses
  {
    IntrinsicCalibrationAccuracyResult result = measureIntrinsicCalibrationAccuracy(
        observations, camera.intr, camera_mount_to_camera, target_mount_to_target, camera_base_to_target_base);

    // Expect results to be within 2 sigma (~95%) of the threshold
    {
      double threshold = 1.0e-13;
      EXPECT_LT(result.pos_error.first + 2.0 * result.pos_error.second, threshold);
    }
    {
      double threshold = 1.0e-13;
      EXPECT_LT(result.ang_error.first + 2.0 * result.ang_error.second, threshold);
    }
  }

  // Perturb the calibration transforms slightly
  Eigen::Isometry3d camera_mount_to_camera_guess = test::perturbPose(camera_mount_to_camera, 0.01, 0.01);
  Eigen::Isometry3d target_mount_to_target_guess = test::perturbPose(target_mount_to_target, 0.01, 0.01);

  // Perfect intrinsic parameters, imperfect transform guesses
  {
    IntrinsicCalibrationAccuracyResult result =
        measureIntrinsicCalibrationAccuracy(observations, camera.intr, camera_mount_to_camera_guess,
                                            target_mount_to_target_guess, camera_base_to_target_base);

    // Expect results to be within 2 sigma (~95%) of the threshold
    {
      double threshold = 1.0e-7;
      EXPECT_LT(result.pos_error.first + 2.0 * result.pos_error.second, threshold);
    }
    {
      double threshold = 1.0e-7;
      EXPECT_LT(result.ang_error.first + 2.0 * result.ang_error.second, threshold);
    }
  }

  // Imperfect intrinsic parameters, imperfect transform guesses
  {
    // Changing the intrinsic values by 1% should yield an error of more than 1mm
    camera.intr.fx() += 0.01 * camera.intr.fx();
    camera.intr.fy() -= 0.01 * camera.intr.fy();
    camera.intr.cx() -= 0.01 * camera.intr.cx();
    camera.intr.cx() += 0.01 * camera.intr.cy();

    IntrinsicCalibrationAccuracyResult result =
        measureIntrinsicCalibrationAccuracy(observations, camera.intr, camera_mount_to_camera_guess,
                                            target_mount_to_target_guess, camera_base_to_target_base);

    // Expect results not to be within 2 sigma (~95%) of the threshold because we modified the camera intrinsics
    {
      double threshold = 1.0e-3;
      EXPECT_GT(result.pos_error.first + 2.0 * result.pos_error.second, threshold);
    }
    {
      double threshold = 1.0e-3;
      EXPECT_GT(result.ang_error.first + 2.0 * result.ang_error.second, threshold);
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
