#include <industrial_calibration/target_finders/opencv/target_finder.h>
// Implementations
#include <industrial_calibration/target_finders/opencv/modified_circle_grid_target_finder.h>
#include <industrial_calibration/target_finders/opencv/charuco_grid_target_finder.h>
#include <industrial_calibration/target_finders/opencv/aruco_grid_target_finder.h>
// Analysis
#include <industrial_calibration/analysis/homography_analysis.h>

#include <numeric>
#include <gtest/gtest.h>
#include <memory>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

using namespace industrial_calibration;

class TargetFinderTestFixture : public ::testing::Test
{
public:
  TargetFinderTestFixture() : ::testing::Test(), homography_error_threshold(1.0) {}

  void runTest()
  {
    // Load the test image
    cv::Mat m = cv::imread(filename);
    ASSERT_FALSE(m.empty());

    // Find the target features
    TargetFeatures2D target_features = this->finder->findTargetFeatures(m);

    ASSERT_EQ(target_features.size(), expected_ids.size());

    // Iterate through the features to see if they match the ID
    for (const unsigned id : expected_ids)
    {
      EXPECT_TRUE(target_features.find(id) != target_features.end());
    }

    // Check the homography transform between the points in the image and the target points
    // to make sure the correspondences were identified correctly
    if (corr_sampler)
    {
      auto corrs = finder->target().createCorrespondences(target_features);
      Eigen::VectorXd err = calculateHomographyError(corrs, *corr_sampler);
      std::cout << "Homography Error" << std::endl;
      std::cout << "Min: " << err.array().minCoeff() << std::endl;
      std::cout << "Max: " << err.array().maxCoeff() << std::endl;
      std::cout << "Mean: " << err.array().mean() << std::endl;

      EXPECT_LT(err.array().mean(), homography_error_threshold);
    }
  }

  std::shared_ptr<TargetFinderOpenCV> finder;
  std::string filename;
  std::vector<unsigned> expected_ids;
  std::shared_ptr<CorrespondenceSampler> corr_sampler;
  double homography_error_threshold;
};

class ModifiedCircleGridFinderTest : public TargetFinderTestFixture
{
public:
  ModifiedCircleGridFinderTest() : TargetFinderTestFixture(), target(10, 10, 0.025)
  {
    finder = std::make_shared<ModifiedCircleGridTargetFinder>(target);
  }

  virtual void SetUp() override
  {
    filename = std::string(TEST_SUPPORT_DIR) + "modified_circle_grid_10x10_0.025.png";

    // Expect all corners to be detected
    expected_ids.resize(target.rows * target.cols);
    std::iota(expected_ids.begin(), expected_ids.end(), 0);

    // Set up the homography check correspondence sampler
    corr_sampler = std::make_shared<GridCorrespondenceSampler>(target.rows, target.cols, 1);
    homography_error_threshold = 1.0;
  }

  const ModifiedCircleGridTarget target;
};

class CharucoFinderTest : public TargetFinderTestFixture
{
public:
  CharucoFinderTest() : TargetFinderTestFixture(), target(7, 5, 0.02f, 0.01f, cv::aruco::DICT_6X6_250)
  {
    finder = std::make_shared<CharucoGridBoardTargetFinder>(target);
  }

  virtual void SetUp() override
  {
    filename = std::string(TEST_SUPPORT_DIR) + "charuco_unobscured.jpg";

    // Expect all corners to be detected
    expected_ids.resize(target.board->chessboardCorners.size());
    std::iota(expected_ids.begin(), expected_ids.end(), 0);

    // Set up the homography check correspondence sampler
    auto grid_size = target.board->getChessboardSize();
    corr_sampler = std::make_shared<GridCorrespondenceSampler>(grid_size.height - 1, grid_size.width - 1, 1);
    homography_error_threshold = 1.0;
  }

  const CharucoGridTarget target;
};

class CharucoFinderPredefinedBoardTest : public TargetFinderTestFixture
{
public:
  CharucoFinderPredefinedBoardTest()
    : TargetFinderTestFixture()
    , target(cv::aruco::CharucoBoard::create(5, 7, 0.02f, 0.01f,
                                             cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250)))
  {
    finder = std::make_shared<CharucoGridBoardTargetFinder>(target);
  }

  virtual void SetUp() override
  {
    filename = std::string(TEST_SUPPORT_DIR) + "charuco_unobscured.jpg";

    // Expect all corners to be detected
    expected_ids.resize(target.board->chessboardCorners.size());
    std::iota(expected_ids.begin(), expected_ids.end(), 0);

    // Set up the homography check correspondence sampler
    auto grid_size = target.board->getChessboardSize();
    corr_sampler = std::make_shared<GridCorrespondenceSampler>(grid_size.height - 1, grid_size.width - 1, 1);
    homography_error_threshold = 1.0;
  }

  const CharucoGridTarget target;
};

class ObscuredCharucoFinderTest : public CharucoFinderTest
{
public:
  using CharucoFinderTest::CharucoFinderTest;

  virtual void SetUp() override final
  {
    filename = std::string(TEST_SUPPORT_DIR) + "charuco_obscured.jpg";

    // Expect the first seven corners not to be detected
    expected_ids.resize(target.board->chessboardCorners.size() - 7);
    std::iota(expected_ids.begin(), expected_ids.end(), 7);

    // Don't set up the homography check correspondence sampler because of the removed points
  }
};

class OneFeatureCharucoFinderTest : public CharucoFinderTest
{
public:
  using CharucoFinderTest::CharucoFinderTest;

  virtual void SetUp() override final
  {
    filename = std::string(TEST_SUPPORT_DIR) + "charuco_one_corner.jpg";

    // Expect to only see the first corner (ID = 0)
    expected_ids.resize(1);
    expected_ids.front() = 0;

    // Don't check homography because there are not enough points
  }
};

class ArucoFinderTest : public TargetFinderTestFixture
{
public:
  ArucoFinderTest() : TargetFinderTestFixture(), target(20, 20, 0.035f, 0.010f, cv::aruco::DICT_6X6_1000)
  {
    finder = std::make_shared<ArucoGridBoardTargetFinder>(target);
  }

  virtual void SetUp() override
  {
    filename = std::string(TEST_SUPPORT_DIR) + "aruco.jpg";

    // Expect to see the entire board
    expected_ids.resize(static_cast<unsigned>(target.board->getGridSize().area()));
    std::iota(expected_ids.begin(), expected_ids.end(), 0);

    // Set up the homography check correspondence sampler
    // The stride of the correspondence sampler is 4, since there are 4 corners associated with each unique tag
    auto grid_size = target.board->getGridSize();
    corr_sampler = std::make_shared<GridCorrespondenceSampler>(grid_size.height, grid_size.width, 4);
    homography_error_threshold = 3.0;
  }

  ArucoGridTarget target;
};

class ArucoFinderPredefinedBoardTest : public TargetFinderTestFixture
{
public:
  ArucoFinderPredefinedBoardTest()
    : TargetFinderTestFixture()
    , target(cv::aruco::GridBoard::create(20, 20, 0.035f, 0.010f,
                                          cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000)))
  {
    finder = std::make_shared<ArucoGridBoardTargetFinder>(target);
  }

  virtual void SetUp() override
  {
    filename = std::string(TEST_SUPPORT_DIR) + "aruco.jpg";

    // Expect to see the entire board
    expected_ids.resize(static_cast<unsigned>(target.board->getGridSize().area()));
    std::iota(expected_ids.begin(), expected_ids.end(), 0);

    // Set up the homography check correspondence sampler
    // The stride of the correspondence sampler is 4, since there are 4 corners associated with each unique tag
    auto grid_size = target.board->getGridSize();
    corr_sampler = std::make_shared<GridCorrespondenceSampler>(grid_size.height, grid_size.width, 4);
    homography_error_threshold = 3.0;
  }

  ArucoGridTarget target;
};

TEST_F(ModifiedCircleGridFinderTest, findTargetFeatures) { this->runTest(); }

TEST_F(CharucoFinderTest, findTargetFeatures) { this->runTest(); }

TEST_F(CharucoFinderPredefinedBoardTest, findTargetFeatures) { this->runTest(); }

TEST_F(ObscuredCharucoFinderTest, findTargetFeatures) { this->runTest(); }

TEST_F(OneFeatureCharucoFinderTest, findTargetFeatures) { this->runTest(); }

TEST_F(ArucoFinderTest, findTargetFeatures) { this->runTest(); }

TEST_F(ArucoFinderPredefinedBoardTest, findTargetFeatures) { this->runTest(); }

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
