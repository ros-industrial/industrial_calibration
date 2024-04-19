#include <industrial_calibration/optimizations/covariance_types.h>

#include <algorithm>
#include <cmath>
#include <ostream>
#include <sstream>

namespace industrial_calibration
{
std::string NamedParam::toString() const
{
  std::stringstream ss;
  ss << names.first.c_str() << " " << names.second.c_str() << " " << value;
  return ss.str();
}

std::vector<NamedParam> CovarianceResult::getCorrelationCoeffOutsideThreshold(const std::double_t& threshold) const
{
  std::vector<NamedParam> out;
  for (auto corr : correlation_coeffs)
  {
    if (std::abs(corr.value) > threshold) out.push_back(corr);
  }
  std::sort(out.begin(), out.end(), [](NamedParam a, NamedParam b) { return std::abs(a.value) > std::abs(b.value); });
  return out;
}

std::string CovarianceResult::toString() const
{
  std::string out;
  out.append("Std. Devs.\n");
  for (auto std_dev : standard_deviations)
  {
    out.append(std_dev.toString() + "\n");
  }

  out.append("\nCovariance\n");
  for (auto cov : covariances)
  {
    out.append(cov.toString() + "\n");
  }

  out.append("\nCorrelation Coeffs.\n");
  for (auto corr : correlation_coeffs)
  {
    out.append(corr.toString() + "\n");
  }
  return out;
}

std::string CovarianceResult::printCorrelationCoeffAboveThreshold(const std::double_t& threshold) const
{
  auto above_thresh = getCorrelationCoeffOutsideThreshold(threshold);

  if (above_thresh.size() == 0)
  {
    return std::string("No correlation coefficients with magnitude > " + std::to_string(threshold) + "\n");
  }

  std::string out("\nCorrelation Coeff. > " + std::to_string(threshold) + ":\n");
  for (auto corr : above_thresh)
  {
    out.append(corr.toString() + "\n");
  }
  return out;
}

}  // namespace industrial_calibration
