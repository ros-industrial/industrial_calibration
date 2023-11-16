#pragma once

#include <Eigen/Geometry>
#include <vector>
#include <random>
#include <memory>

namespace industrial_calibration
{
namespace test
{
/**
 * @brief Defines a camera matrix using a camera origin, a position its looking at, and an up vector hint
 * @param origin - The position of the camera focal point
 * @param eye - A point that the camera is looking at
 * @param up - The upvector in world-space
 */
Eigen::Isometry3d lookAt(const Eigen::Vector3d& origin, const Eigen::Vector3d& eye, const Eigen::Vector3d& up) noexcept;

struct PoseGenerator
{
  /**
   * @brief Generates a set of camera poses
   * @param target_origin The position of the target
   * @return A vector of camera positions & orientations
   */
  virtual std::vector<Eigen::Isometry3d> generate(const Eigen::Isometry3d& target_origin) = 0;
};

/**
 * @brief Generates camera poses in a hemisphere pattern
 */
struct HemispherePoseGenerator : PoseGenerator
{
  inline HemispherePoseGenerator(double r_, unsigned theta_cnt_, unsigned phi_cnt_, double z_rot_,
                                 const Eigen::Isometry3d& target_offset_)
    : r(r_), theta_cnt(theta_cnt_), phi_cnt(phi_cnt_), z_rot(z_rot_), target_offset(target_offset_)
  {
  }

  inline HemispherePoseGenerator(double r_, unsigned theta_cnt_, unsigned phi_cnt_)
    : HemispherePoseGenerator(r_, theta_cnt_, phi_cnt_, 0.0, Eigen::Isometry3d::Identity())
  {
  }

  inline HemispherePoseGenerator() : HemispherePoseGenerator(2.0, 10, 10, 0.0, Eigen::Isometry3d::Identity()) {}

  virtual std::vector<Eigen::Isometry3d> generate(const Eigen::Isometry3d& target_origin) override final;

  double r;                        /** @brief Radius of the hemisphere */
  unsigned theta_cnt;              /** @brief The number of points in the theta-wise direction*/
  unsigned phi_cnt;                /** @brief The number of points in the phi-wise direction */
  double z_rot;                    /** @brief Rotation about camera Z+ */
  Eigen::Isometry3d target_offset; /** @brief Transform from target origin to origin of pose pattern */
};

/**
 * @brief Generates camera positions using a conical template, with the target at the 'point'
 */
struct ConicalPoseGenerator : PoseGenerator
{
  inline ConicalPoseGenerator(double r_, double h_, unsigned n_poses_, double z_rot_,
                              const Eigen::Isometry3d& target_offset_)
    : r(r_), h(h_), n_poses(n_poses_), z_rot(z_rot_), target_offset(target_offset_)
  {
  }

  inline ConicalPoseGenerator(double r_, double h_, unsigned n_poses_)
    : ConicalPoseGenerator(r_, h_, n_poses_, 0.0, Eigen::Isometry3d::Identity())
  {
  }

  inline ConicalPoseGenerator() : ConicalPoseGenerator(1.0, 2.0, 20, 0.0, Eigen::Isometry3d::Identity()) {}

  virtual std::vector<Eigen::Isometry3d> generate(const Eigen::Isometry3d& target_origin) override final;

  double r;                        /** @brief Radius of the cone*/
  double h;                        /** @brief Height of the cone (distance to target) */
  unsigned n_poses;                /** @brief Number of poses to generate */
  double z_rot;                    /** @brief Rotation about camera Z+ */
  Eigen::Isometry3d target_offset; /** @brief Transform from target origin to origin of pose pattern */
};

/**
 * @brief Generates poses in a grid
 */
struct GridPoseGenerator : PoseGenerator
{
  inline GridPoseGenerator(double spacing_, double h_, unsigned grid_side_, double z_rot_,
                           const Eigen::Isometry3d& target_offset_)
    : spacing(spacing_), h(h_), grid_side(grid_side_), z_rot(z_rot_), target_offset(target_offset_)
  {
  }

  inline GridPoseGenerator(double spacing_, double h_, unsigned grid_side_)
    : GridPoseGenerator(spacing_, h_, grid_side_, 0.0, Eigen::Isometry3d::Identity())
  {
  }

  inline GridPoseGenerator() : GridPoseGenerator(0.2, 2.0, 10, 0.0, Eigen::Isometry3d::Identity()) {}

  virtual std::vector<Eigen::Isometry3d> generate(const Eigen::Isometry3d& target_origin) override final;

  double spacing;                  /** @brief Distance between points */
  double h;                        /** @brief Grid distance to target */
  unsigned grid_side;              /** @brief number of columns & rows to go into grid */
  double z_rot;                    /** @brief Rotation to apply around camera Z-axis (radians) */
  Eigen::Isometry3d target_offset; /** @brief Transform from target origin to origin of pose pattern */
};

/**
 * @brief Applies a random rotation around the camera Z+ axis in range [@ref z_rot_min_, @ref z_rot_max_]
 */
struct RandomZRotPoseGenerator : PoseGenerator
{
  inline RandomZRotPoseGenerator(const std::shared_ptr<PoseGenerator>& pg_, double z_rot_min_, double z_rot_max_,
                                 unsigned long seed_ = std::mt19937::default_seed)
    : pg(pg_), mt_rand(seed_), z_sampler(z_rot_min_, z_rot_max_)
  {
  }

  inline RandomZRotPoseGenerator(const std::shared_ptr<PoseGenerator>& pg_,
                                 unsigned long seed_ = std::mt19937::default_seed)
    : RandomZRotPoseGenerator(pg_, 0, 2 * M_PI, seed_)
  {
  }

  virtual std::vector<Eigen::Isometry3d> generate(const Eigen::Isometry3d& target_origin) override final;

  std::shared_ptr<PoseGenerator> pg;

private:
  std::mt19937 mt_rand;                             /** @brief Psuedo-random number generator */
  std::uniform_real_distribution<double> z_sampler; /** @brief Sampler for camera Z+ rotation */
};

}  // namespace test
}  // namespace industrial_calibration
