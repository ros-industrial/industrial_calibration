#include <industrial_calibration/analysis/statistics.h>
#include <industrial_calibration/core/exceptions.h>

#include <numeric>
#include <Eigen/SVD>

namespace industrial_calibration
{
std::tuple<double, double> computeStats(const std::vector<double>& v)
{
  double mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
  double var = 0.0;
  std::for_each(std::begin(v), std::end(v), [&](const double d) { var += (d - mean) * (d - mean); });
  var /= (v.size() - 1);
  return std::make_tuple(mean, std::sqrt(std::abs(var)));
}

Eigen::Quaterniond computeQuaternionMean(const std::vector<Eigen::Quaterniond>& quaterns)
{
  Eigen::Matrix4d M = Eigen::Matrix4d::Zero();

  for (const Eigen::Quaterniond& q : quaterns)
  {
    M += q.coeffs() * q.coeffs().transpose();
  }

  // Calculate the SVD of the M matrix
  Eigen::JacobiSVD<Eigen::Matrix4d> svd(M, Eigen::ComputeFullU);

  // The eigenvectors are represented by the columns of the U matrix; the eigenvector corresponding to the largest
  // eigenvalue is in row 0
  Eigen::Quaterniond q;
  q.coeffs() << svd.matrixU().col(0);

  if (q.coeffs().array().hasNaN())
    throw ICalException("Mean quaternion has NaN values");

  return q;
};

QuaternionStats computeQuaternionStats(const std::vector<Eigen::Quaterniond>& quaternions)
{
  QuaternionStats q_stats;
  q_stats.mean = computeQuaternionMean(quaternions);

  double q_var = 0.0;
  for (const Eigen::Quaterniond& q : quaternions)
  {
    q_var += std::pow(q_stats.mean.angularDistance(q), 2.0);
  }
  q_var /= static_cast<double>(quaternions.size() - 1);
  q_stats.stdev = std::sqrt(q_var);

  return q_stats;
}

}  // namespace industrial_calibration
