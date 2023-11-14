#pragma once

#include <industrial_calibration/types.h>
#include <industrial_calibration/optimizations/utils/covariance_types.h>

#include <vector>
#include <Eigen/Dense>

namespace industrial_calibration
{
/**
 * @brief Cost function for the error between a point and a circle.
 */
class CircleDistCost
{
public:
  CircleDistCost(const double& x, const double& y) : x_(x), y_(y)
  {
  }

  /**
   * @brief One formulation of a residual function for error relative to the center of a circle.
   * Presented to the solver as three size-1 double parameters.
   * @param sample A double pointer which contains {x, y, r_sqrt}.
   */
  template <typename T>
  bool operator()(const T* const sample, T* residual) const
  {
    T r = sample[2] * sample[2];

    T x_pos = x_ - sample[0];
    T y_pos = y_ - sample[1];

    residual[0] = r * r - x_pos * x_pos - y_pos * y_pos;
    return true;
  }

  /**
   * @brief An alternative formulation of a residual function for error relative to the center of a circle.
   * Here, the inputs (x, y, r_sqrt) are provided as separate parameters. This is mathematically identical to the
   * previous version, but the number and size of the parameters are presented differently to the solver (one size-3
   * double parameter).
   * @param x_sample The x-coordinate of the center of the circle.
   * @param y_sample The y-coordinate of the center of the circle.
   * @param r_sqrt_sample The square root of the radius of the circle.
   */
  template <typename T>
  bool operator()(const T* const x_sample, const T* const y_sample, const T* const r_sqrt_sample, T* residual) const
  {
    T r = r_sqrt_sample[0] * r_sqrt_sample[0];  // Using sqrt(radius) prevents the radius from ending up negative and
                                                // improves numerical stability.

    T x_pos = x_ - x_sample[0];
    T y_pos = y_ - y_sample[0];

    residual[0] = r * r - x_pos * x_pos - y_pos * y_pos;
    return true;
  }

private:
  double x_, y_;
};

/**
 * @brief Problem setup info for the circle fit optimization.
 */
struct CircleFitProblem
{
  /**
   * @brief Observations around the edge of the circle. 2D points in the form (x, y).
   */
  std::vector<Eigen::Vector2d> observations;

  /**
   * @brief Estimate for the initial x-coordinate of the center of the circle.
   */
  double x_center_initial;

  /**
   * @brief Estimate for the initial y-coordinate of the center of the circle.
   */
  double y_center_initial;

  /**
   * @brief Estimate for the initial radius of the circle.
   */
  double radius_initial;

  const std::vector<std::string> labels = { "x", "y", "r" };
};

/**
 * @brief Results for the circle fit optimization
 */
struct CircleFitResult
{
  /**
   * @brief True if converged, False if any other result.
   */
  bool converged;

  /**
   * @brief Initial average error per observation.
   */
  double initial_cost_per_obs;

  /**
   * @brief Final average error per observation.
   */
  double final_cost_per_obs;

  /**
   * @brief The covariance matrix for the problem. A square 3x3 matrix giving covariance between each pair of elements.
   * Order of elements is [x, y, radius].
   */
  CovarianceResult covariance;

  /**
   * @brief Calculated circle center x-coord.
   */
  double x_center;

  /**
   * @brief Calculated circle center y-coord.
   */
  double y_center;

  /**
   * @brief Calculated circle radius.
   */
  double radius;
};

/**
 * @brief Function that solves the circle fit problem.
 * @param Input observations and guesses.
 * @return Output results.
 */
CircleFitResult optimize(const CircleFitProblem& params);

}  // namespace industrial_calibration
