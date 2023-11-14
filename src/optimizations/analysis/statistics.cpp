#include <ical_core/optimizations/analysis/statistics.h>
#include <ical_core/exceptions.h>

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
  /* Mean quaternion is found using method described by Markley et al: Quaternion Averaging
   * https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
   *
   * M = sum(w_i * q_i * q_i^T)    Eq. 12
   * q_bar = argmax(q^T * M * q)   Eq. 13
   *
   * "The solution of this maximization problem is well known. The average quaternion is
   * the eigenvector of M corresponding to the maximum eigenvalue."
   *
   * In the above equations, w_i is the weight of the ith quaternion.
   * In this case, all quaternions are equally weighted (i.e. w_i = 1)
   */

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

  if (q.coeffs().array().hasNaN()) throw ICalException("Mean quaternion has NaN values");

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
