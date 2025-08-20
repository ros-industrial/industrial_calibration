#include <industrial_calibration/analysis/homography_analysis.h>
#include <industrial_calibration/core/types.h>
#include <industrial_calibration/core/exceptions.h>
#include <industrial_calibration/core/utils.h>

#include <Eigen/Dense>

namespace industrial_calibration
{
GridCorrespondenceSampler::GridCorrespondenceSampler(const std::size_t rows_, const std::size_t cols_,
                                                     const std::size_t stride_)
  : rows(rows_), cols(cols_), stride(stride_)
{
}

std::vector<std::size_t> GridCorrespondenceSampler::getSampleCorrespondenceIndices() const
{
  const std::size_t n_samples = 4;

  // Make sure there are at least two times as many points as the number of sample points
  if ((rows * cols / 2) < n_samples)
  {
    std::stringstream ss;
    ss << "Number of correspondences does not exceed minimum of " << n_samples * 2 << " (" << rows * cols
       << " provided)";
    throw ICalException(ss.str());
  }

  std::vector<std::size_t> correspondence_indices;
  correspondence_indices.reserve(n_samples);

  // Sample points should be the corners of the grid, using the first element in the stride
  std::size_t upper_left_idx = 0;
  std::size_t upper_right_idx = (cols - 1) * stride;
  std::size_t lower_left_idx = (rows - 1) * (cols * stride);
  std::size_t lower_right_idx = (rows * cols * stride) - stride;

  correspondence_indices.push_back(upper_left_idx);
  correspondence_indices.push_back(upper_right_idx);
  correspondence_indices.push_back(lower_left_idx);
  correspondence_indices.push_back(lower_right_idx);

  return correspondence_indices;
}

RandomCorrespondenceSampler::RandomCorrespondenceSampler(const std::size_t n_correspondences_,
                                                         const std::size_t n_samples_, const unsigned seed_)
  : n_correspondences(n_correspondences_), n_samples(n_samples_), seed(seed_)
{
  const unsigned min_samples = 4;
  if (n_samples < min_samples)
  {
    std::stringstream ss;
    ss << "Not enough samples specified: " << n_samples << " vs. " << min_samples << " required";
    throw ICalException(ss.str());
  }
  if (n_samples > n_correspondences)
  {
    std::stringstream ss;
    ss << "Number of correspondences (" << n_correspondences << ") must exceed number of samples (" << n_samples << ")";
    throw ICalException(ss.str());
  }
}

std::vector<std::size_t> RandomCorrespondenceSampler::getSampleCorrespondenceIndices() const
{
  // Create a random number generator with a uniform distribution across all indices
  std::mt19937 rand_gen(seed);
  std::uniform_int_distribution<std::size_t> dist(0, n_correspondences - 1);
  auto fn = [&rand_gen, &dist]() -> std::size_t { return dist(rand_gen); };

  // Generate a vector of 4 random correspondence indices
  std::vector<std::size_t> output(n_samples);
  std::generate(output.begin(), output.end(), fn);

  return output;
}

static std::tuple<Eigen::Matrix3Xd, Eigen::Matrix2Xd> toEigen(const Correspondence2D3D::Set& correspondences)
{
  // Convert the correspondence data
  Eigen::Matrix3Xd in_target(3, correspondences.size());
  for (std::size_t i = 0; i < correspondences.size(); ++i)
    in_target.col(i) = correspondences[i].in_target;

  Eigen::Matrix2Xd in_image(2, correspondences.size());
  for (std::size_t i = 0; i < correspondences.size(); ++i)
    in_image.col(i) = correspondences[i].in_image;

  return std::make_tuple(in_target, in_image);
}

Eigen::VectorXd calculateHomographyError(const Correspondence2D3D::Set& correspondences,
                                         const CorrespondenceSampler& correspondence_sampler)
{
  // Select the points that we want to use to create the H matrix
  std::vector<std::size_t> sample_correspondence_indices = correspondence_sampler.getSampleCorrespondenceIndices();
  std::size_t n_samples = sample_correspondence_indices.size();

  // Ensure that there are enough points for testing outside of the sampled set
  if (correspondences.size() < 2 * n_samples)
  {
    std::stringstream ss;
    ss << "Correspondences size is not more than 2x sample size (" << correspondences.size() << " correspondences vs. "
       << n_samples << ")";
    throw ICalException(ss.str());
  }

  Correspondence2D3D::Set sampled_correspondences;
  sampled_correspondences.reserve(n_samples);
  for (std::size_t i : sample_correspondence_indices)
    sampled_correspondences.push_back(correspondences[i]);

  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> H = calculateHomography(sampled_correspondences);

  // Convert the correspondence data into Eigen matrices
  Eigen::Matrix3Xd in_target;
  Eigen::Matrix2Xd in_image;
  std::tie(in_target, in_image) = toEigen(correspondences);

  // Project all of the correspondence target points into image coordinates using the homography matrix
  Eigen::Matrix2Xd uv = projectHomography(H, in_target.block(0, 0, 2, in_target.cols()));

  // Estimate the image locations of all the target observations and compare to the actual image locations
  Eigen::VectorXd error = (in_image - uv).colwise().norm();

  return error;
}

Eigen::VectorXd calculateHomographyError(const Correspondence3D3D::Set& correspondences,
                                         const CorrespondenceSampler& correspondence_sampler)
{
  // Convert the 3D correspondence points into 2D points by scaling the x and y components by the z component
  Correspondence2D3D::Set correspondences_2d;
  correspondences_2d.reserve(correspondences.size());

  // Store the z values for scaling later
  Eigen::VectorXd z_values(correspondences.size());

  for (std::size_t i = 0; i < correspondences.size(); ++i)
  {
    const auto& corr = correspondences[i];

    // Store the z value of the correspondence
    z_values[i] = corr.in_image.z();

    // Generate a scaled 2D correspondence
    Correspondence2D3D corr_2d;
    corr_2d.in_target = corr.in_target;
    corr_2d.in_image = (corr.in_image / corr.in_image.z()).head<2>();
    correspondences_2d.push_back(corr_2d);
  }

  // Calculate the homography error
  Eigen::VectorXd error = calculateHomographyError(correspondences_2d, correspondence_sampler);

  // Scale the errors again by the z values of the original correspondences
  return error.cwiseProduct(z_values);
}

}  // namespace industrial_calibration
