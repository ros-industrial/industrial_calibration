/*
 * Computes the pose of a "target" seen in one or more intrinsically and extrinsically calibrated
 * cameras. The pose of these cameras must be specified in a common "base" frame, and the guess
 * at the target must be specified in this same base frame. The answer will also be in this common
 * base frame.
 *
 * Note that this function is logically equivalent to the solver available in pnp.h assuming that
 * you specified ONE camera with an identity transform.
 *
 * author: Levi Armstrong
 */
#pragma once

#include <industrial_calibration/types.h>
#include <industrial_calibration/camera_intrinsics.h>

namespace industrial_calibration
{
struct MultiCameraPnPProblem
{
  /** @brief The basic camera intrinsic propeties: fx, fy, cx, cy used to reproject points;
      one for each camera. Should have same length as @e base_to_camera. */
  std::vector<CameraIntrinsics> intr;

  /** @brief The "base frame" to "camera frame" transform; one for each camera. Should have same
      length as @e intr. */
  std::vector<Eigen::Isometry3d> base_to_camera;

  /** @brief A sequence of observation sets corresponding to the image locations.
   * Each observation set consists of a set of correspodences: a 3D position (e.g. a dot) in "target
   * frame" to the image location it was detected at (2D). The outer-most vector is for each camera,
   * the inner vector is the images valid for that camera. */
  std::vector<Correspondence2D3D::Set> image_observations;

  /** @brief Your best guess for transforms, "base to target", for a given observation set taken.*/
  Eigen::Isometry3d base_to_target_guess;
};

struct MultiCameraPnPResult
{
  /**
   * @brief Whether the underlying solver converged. If this is false, your calibration did not go well.
   * If this is true, your calibration MAY have gone well.
   */
  bool converged;

  /**
   * @brief The initial reprojection error (in pixels) per residual based on your input guesses.
   */
  double initial_cost_per_obs;

  /**
   * @brief The final reprojection error (in pixels) per residual after optimization. Note that each circle
   * has two residuals: a U and V error in the image. So a value of 1.2 means that each circle was described
   * to within 1.2 pixels in X and 1.2 pixels in Y.
   *
   * A low value here is encouraging if you had a diversity of images. If you took few images, you can get
   * a low score without getting a calibration that describes your workcell.
   */
  double final_cost_per_obs;

  /** @brief The final location of the target. */
  Eigen::Isometry3d base_to_target;
};

MultiCameraPnPResult optimize(const MultiCameraPnPProblem& params);

}  // namespace industrial_calibration
