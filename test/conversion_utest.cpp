#include <gtest/gtest.h>
#include <industrial_calibration/optimizations/ceres_math_utilities.h>

using namespace industrial_calibration;

// Test the conversion of a pose from Eigen to internal calibration format and back
TEST(EigenConversions, there_and_back_identity)
{
  const auto e_pose = Eigen::Isometry3d::Identity();
  const auto cal_pose = poseEigenToCal(e_pose);
  const auto recovered_e_pose = poseCalToEigen(cal_pose);

  const Eigen::Matrix4d diff = e_pose.matrix() - recovered_e_pose.matrix();
  EXPECT_TRUE(diff.isZero());
}

// Test the conversion of a pose from Eigen to internal calibration format and back
TEST(EigenConversions, there_and_back)
{
  auto e_pose = Eigen::Isometry3d::Identity();
  e_pose = e_pose * Eigen::Translation3d(0.5, 0.75, 1.0) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
  const auto cal_pose = poseEigenToCal(e_pose);
  const auto recovered_e_pose = poseCalToEigen(cal_pose);

  const Eigen::Matrix4d diff = e_pose.matrix() - recovered_e_pose.matrix();
  EXPECT_TRUE(diff.isZero());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
