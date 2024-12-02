#pragma once

#include <industrial_calibration/core/types.h>
#include <industrial_calibration/core/dh_chain.h>
#include <industrial_calibration/optimizations/ceres_math_utilities.h>

#include <ceres/jet.h>
#include <ceres/version.h>
#define CERES_VERSION_LT_2_1 (CERES_VERSION_MAJOR < 2 || (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR < 1))
#include <map>

namespace industrial_calibration
{
/**
 * @brief Base class for cost functions that perform kinematic calibration between two DH chains, one that holds the
 * camera and one that holds the target
 */
class DualDHChainCost
{
public:
  DualDHChainCost(const DHChain& camera_chain, const DHChain& target_chain, const Eigen::VectorXd& camera_chain_joints,
                  const Eigen::VectorXd& target_chain_joints)
    : camera_chain_(camera_chain)
    , target_chain_(target_chain)
    , camera_chain_joints_(camera_chain_joints)
    , target_chain_joints_(target_chain_joints)

  {
  }

  template <typename T>
  static Isometry3<T> createTransform(T const* const* params, const std::size_t idx)
  {
    Eigen::Map<const Vector3<T>> t(params[idx]);
    Eigen::Map<const Vector3<T>> aa(params[idx + 1]);

    Isometry3<T> result = Isometry3<T>::Identity() * Eigen::Translation<T, 3>(t);

    T aa_norm = aa.norm();
    if (aa_norm > std::numeric_limits<T>::epsilon())
    {
      result *= Eigen::AngleAxis<T>(aa_norm, aa.normalized());
    }
    return result;
  }

  static std::vector<double*> constructParameters(Eigen::MatrixX4d& camera_chain_dh_offsets,
                                                  Eigen::MatrixX4d& target_chain_dh_offsets,
                                                  Eigen::Vector3d& camera_mount_to_camera_position,
                                                  Eigen::Vector3d& camera_mount_to_camera_angle_axis,
                                                  Eigen::Vector3d& target_mount_to_target_position,
                                                  Eigen::Vector3d& target_mount_to_target_angle_axis,
                                                  Eigen::Vector3d& camera_chain_base_to_target_chain_base_position,
                                                  Eigen::Vector3d& camera_chain_base_to_target_chain_base_angle_axis)
  {
    std::vector<double*> parameters;
    parameters.push_back(camera_chain_dh_offsets.data());
    parameters.push_back(target_chain_dh_offsets.data());
    parameters.push_back(camera_mount_to_camera_position.data());
    parameters.push_back(camera_mount_to_camera_angle_axis.data());
    parameters.push_back(target_mount_to_target_position.data());
    parameters.push_back(target_mount_to_target_angle_axis.data());
    parameters.push_back(camera_chain_base_to_target_chain_base_position.data());
    parameters.push_back(camera_chain_base_to_target_chain_base_angle_axis.data());
    return parameters;
  }

  static std::map<const double*, std::vector<std::string>> constructParameterLabels(
      const std::vector<double*>& parameters, const std::vector<std::array<std::string, 4>>& camera_chain_labels,
      const std::vector<std::array<std::string, 4>>& target_chain_labels,
      const std::array<std::string, 3>& camera_mount_to_camera_position_labels,
      const std::array<std::string, 3>& camera_mount_to_camera_angle_axis_labels,
      const std::array<std::string, 3>& target_mount_to_target_position_labels,
      const std::array<std::string, 3>& target_mount_to_target_angle_axis_labels,
      const std::array<std::string, 3>& camera_chain_base_to_target_chain_base_position_labels,
      const std::array<std::string, 3>& camera_chain_base_to_target_chain_base_angle_axis_labels)
  {
    // Eigen is column major so need to store labels column major
    std::map<const double*, std::vector<std::string>> param_labels;
    std::vector<std::string> cc_labels_concatenated;
    for (std::size_t c = 0; c < 4; ++c)
    {
      for (auto cc_label : camera_chain_labels)
        cc_labels_concatenated.push_back(cc_label.at(c));
    }
    param_labels[parameters[0]] = cc_labels_concatenated;

    // Eigen is column major so need to store labels column major
    std::vector<std::string> tc_labels_concatenated;
    for (std::size_t c = 0; c < 4; ++c)
    {
      for (auto tc_label : target_chain_labels)
        tc_labels_concatenated.push_back(tc_label.at(c));
    }
    param_labels[parameters[1]] = tc_labels_concatenated;

    param_labels[parameters[2]] = std::vector<std::string>(camera_mount_to_camera_position_labels.begin(),
                                                           camera_mount_to_camera_position_labels.end());
    param_labels[parameters[3]] = std::vector<std::string>(camera_mount_to_camera_angle_axis_labels.begin(),
                                                           camera_mount_to_camera_angle_axis_labels.end());
    param_labels[parameters[4]] = std::vector<std::string>(target_mount_to_target_position_labels.begin(),
                                                           target_mount_to_target_position_labels.end());
    param_labels[parameters[5]] = std::vector<std::string>(target_mount_to_target_angle_axis_labels.begin(),
                                                           target_mount_to_target_angle_axis_labels.end());
    param_labels[parameters[6]] =
        std::vector<std::string>(camera_chain_base_to_target_chain_base_position_labels.begin(),
                                 camera_chain_base_to_target_chain_base_position_labels.end());
    param_labels[parameters[7]] =
        std::vector<std::string>(camera_chain_base_to_target_chain_base_angle_axis_labels.begin(),
                                 camera_chain_base_to_target_chain_base_angle_axis_labels.end());
    return param_labels;
  }

  static std::map<const double*, std::vector<int>> constructParameterMasks(const std::vector<double*>& parameters,
                                                                           const std::array<std::vector<int>, 8>& masks)
  {
    assert(parameters.size() == masks.size());
    std::map<const double*, std::vector<int>> param_masks;
    for (std::size_t i = 0; i < parameters.size(); ++i)
      param_masks[parameters[i]] = masks[i];

    return param_masks;
  }

  static std::map<const double*, std::string> constructParameterNames(const std::vector<double*>& parameters)
  {
    std::map<const double*, std::string> names;
    names[parameters[0]] = "Camera DH Parameters";
    names[parameters[1]] = "Target DH Parameters";
    names[parameters[2]] = "Camera Mount to Camera Position Parameters";
    names[parameters[3]] = "Camera Mount to Camera Orientation Parameters";
    names[parameters[4]] = "Target Mount to Target Position Parameters";
    names[parameters[5]] = "Target Mount to Target Orientation Parameters";
    names[parameters[6]] = "Camera Chain Base to Target Chain Base Position Parameters";
    names[parameters[7]] = "Camera Chain Base to Target Chain Base Orientation Parameters";

    return names;
  }

protected:
  const DHChain& camera_chain_;
  const DHChain& target_chain_;

  const Eigen::VectorXd camera_chain_joints_;
  const Eigen::VectorXd target_chain_joints_;
};

/**
 * @brief Cost function for performing kinematic calibration between two DH chains, one that holds a 2D camera and one
 * that holds the target
 */
class DualDHChain2D3DCost : public DualDHChainCost
{
public:
  DualDHChain2D3DCost(const Eigen::Vector2d& obs, const Eigen::Vector3d& point_in_target, const CameraIntrinsics& intr,
                      const DHChain& camera_chain, const DHChain& target_chain,
                      const Eigen::VectorXd& camera_chain_joints, const Eigen::VectorXd& target_chain_joints)
    : DualDHChainCost(camera_chain, target_chain, camera_chain_joints, target_chain_joints)
    , obs_(obs)
    , target_pt_(point_in_target)
    , intr_(intr)
  {
  }

  template <typename T>
  bool operator()(T const* const* parameters, T* residual) const
  {
    // Step 1: Load the data
    // The first parameter is a pointer to the DH parameter offsets of the camera kinematic chain
    Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 4>> camera_chain_dh_offsets(parameters[0], camera_chain_.dof(),
                                                                                  4);

    // The next parameter is a pointer to the DH parameter offsets of the target kinematic chain
    Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 4>> target_chain_dh_offsets(parameters[1], target_chain_.dof(),
                                                                                  4);

    // The next two parameters are pointers to the position and angle axis of the transform from the camera mount to the
    // camera
    std::size_t cm_to_c_idx = 2;
    const Isometry3<T> camera_mount_to_camera = createTransform(parameters, cm_to_c_idx);

    // The next two parameters are pointers to the position and angle axis of the transform from the target mount to the
    // target
    std::size_t tm_to_t_idx = cm_to_c_idx + 2;
    const Isometry3<T> target_mount_to_target = createTransform(parameters, tm_to_t_idx);

    // The next two parameters are pointers to the position and angle axis of the transform from the camera chain base
    // to the target chain base
    std::size_t cb_to_tb_idx = tm_to_t_idx + 2;
    const Isometry3<T> camera_base_to_target_base = createTransform(parameters, cb_to_tb_idx);

    // Step 2: Transformation math
    // Build the transforms from the camera chain base out to the camera
    Isometry3<T> camera_chain_fk = camera_chain_.getFK<T>(camera_chain_joints_.cast<T>(), camera_chain_dh_offsets);
    Isometry3<T> camera_base_to_camera = camera_chain_fk * camera_mount_to_camera;

    // Build the transforms from the camera chain base out to the target
    Isometry3<T> target_chain_fk = target_chain_.getFK<T>(target_chain_joints_.cast<T>(), target_chain_dh_offsets);
    Isometry3<T> camera_base_to_target = camera_base_to_target_base * target_chain_fk * target_mount_to_target;

    // Now that we have two transforms in the same frame, get the target point in the camera frame
    Isometry3<T> camera_to_target = camera_base_to_camera.inverse() * camera_base_to_target;
    Vector3<T> target_in_camera = camera_to_target * target_pt_.cast<T>();

    // Project the target into the image plane
    Vector2<T> target_in_image = projectPoint(intr_, target_in_camera);

    // Step 3: Calculate the error
    residual[0] = target_in_image.x() - obs_.x();
    residual[1] = target_in_image.y() - obs_.y();

    return true;
  }

protected:
  const Eigen::Vector2d obs_;
  const Eigen::Vector3d target_pt_;
  const CameraIntrinsics intr_;
};

/**
 * @brief Cost function for performing kinematic calibration between two DH chains using a measurement of the pose
 * between the camera and target
 */
class DualDHChainMeasurementCost : public DualDHChainCost
{
public:
  DualDHChainMeasurementCost(const KinematicMeasurement& measurement, const DHChain& camera_chain,
                             const DHChain& target_chain, const double orientation_weight)
    : DualDHChainCost(camera_chain, target_chain, measurement.camera_chain_joints, measurement.target_chain_joints)
    , camera_to_target_measured_(measurement.camera_to_target)
    , orientation_weight_(orientation_weight)
  {
  }

  template <typename T>
  bool operator()(T const* const* parameters, T* residual) const
  {
    // Step 1: Load the data
    // The first parameter is a pointer to the DH parameter offsets of the camera kinematic chain
    Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 4>> camera_chain_dh_offsets(parameters[0], camera_chain_.dof(),
                                                                                  4);

    // The next parameter is a pointer to the DH parameter offsets of the target kinematic chain
    Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 4>> target_chain_dh_offsets(parameters[1], target_chain_.dof(),
                                                                                  4);

    // The next two parameters are pointers to the position and angle axis of the transform from the camera mount to the
    // camera
    std::size_t cm_to_c_idx = 2;
    const Isometry3<T> camera_mount_to_camera = createTransform(parameters, cm_to_c_idx);

    // The next two parameters are pointers to the position and angle axis of the transform from the target mount to the
    // target
    std::size_t tm_to_t_idx = cm_to_c_idx + 2;
    const Isometry3<T> target_mount_to_target = createTransform(parameters, tm_to_t_idx);

    // The next two parameters are pointers to the position and angle axis of the transform from the camera chain base
    // to the target chain base
    std::size_t cb_to_tb_idx = tm_to_t_idx + 2;
    const Isometry3<T> camera_base_to_target_base = createTransform(parameters, cb_to_tb_idx);

    // Step 2: Transformation math
    // Build the transforms from the camera chain base out to the camera
    Isometry3<T> camera_chain_fk = camera_chain_.getFK<T>(camera_chain_joints_.cast<T>(), camera_chain_dh_offsets);
    Isometry3<T> camera_base_to_camera = camera_chain_fk * camera_mount_to_camera;

    // Build the transforms from the camera chain base out to the target
    Isometry3<T> target_chain_fk = target_chain_.getFK<T>(target_chain_joints_.cast<T>(), target_chain_dh_offsets);
    Isometry3<T> camera_base_to_target = camera_base_to_target_base * target_chain_fk * target_mount_to_target;

    // Now that we have two transforms in the same frame, get the difference between the expected and observed pose of
    // the target
    Isometry3<T> camera_to_target = camera_base_to_camera.inverse() * camera_base_to_target;

    Isometry3<T> tform_error = camera_to_target_measured_.cast<T>() * camera_to_target.inverse();

    residual[0] = tform_error.translation().x();
    residual[1] = tform_error.translation().y();
    residual[2] = tform_error.translation().z();

    T rot_diff = Eigen::Quaternion<T>(camera_to_target_measured_.cast<T>().linear())
                     .angularDistance(Eigen::Quaternion<T>(camera_to_target.linear()));

#if CERES_VERSION_LT_2_1
    residual[3] = ceres::IsNaN(rot_diff) ? T(0.0) : T(orientation_weight_) * rot_diff;
#else
    residual[3] = ceres::isnan(rot_diff) ? T(0.0) : T(orientation_weight_) * rot_diff;
#endif

    return true;
  }

protected:
  const Eigen::Isometry3d camera_to_target_measured_;
  const double orientation_weight_;
};

}  // namespace industrial_calibration
