#include <industrial_calibration/optimizations/analysis/projection.h>

namespace industrial_calibration
{
Eigen::Vector3d computeLinePlaneIntersection(const Eigen::Vector3d& plane_normal, const Eigen::Vector3d& plane_pt,
                                             const Eigen::Vector3d& line, const Eigen::Vector3d& line_pt,
                                             const double epsilon)
{
  double dp = line.dot(plane_normal);
  if (std::abs(dp) < epsilon)
    throw std::runtime_error("No intersection between line and plane because they are parallel");

  // Solve for the distance from line_pt to the plane: d = ((plane_pt - line_pt) * plane_normal) / (line * plane_normal)
  double d = (plane_pt - line_pt).dot(plane_normal) / dp;

  // Compute a new point at distance d along line from line_pt
  return line_pt + line * d;
}

Eigen::MatrixX3d project3D(const Eigen::MatrixX2d& source, const Eigen::Isometry3d& source_to_plane,
                           const Eigen::Matrix3d& k)
{
  const Eigen::Index n = source.rows();

  // Convert 2D source points to 3D homogeneous coordinates
  Eigen::MatrixX3d source_3d(n, 3);
  source_3d.block(0, 0, n, 2) = source;
  source_3d.col(2) = Eigen::VectorXd::Ones(n);

  // Using camera as reference frame
  Eigen::Vector3d plane_normal = source_to_plane.matrix().col(2).head<3>();  // Plane z-axis in source frame
  Eigen::Vector3d plane_pt = source_to_plane.translation();                  // Plane origin in source frame
  Eigen::Vector3d camera_origin = Eigen::Vector3d::Zero();

  // Create 3D unit vector rays for each 3D coorindate in the camera frame
  Eigen::MatrixX3d rays_in_camera = (k.inverse() * source_3d.transpose()).transpose();
  rays_in_camera.rowwise().normalize();

  Eigen::MatrixX3d projected_source_3d(n, 3);
  for (Eigen::Index i = 0; i < n; ++i)
    projected_source_3d.row(i) =
        computeLinePlaneIntersection(plane_normal, plane_pt, rays_in_camera.row(i), camera_origin);

  // Transform the projected source points into the plane frame
  //   First convert 3D projected source points into 4D homogeneous coordinates
  Eigen::MatrixX4d projected_source_4d(n, 4);
  projected_source_4d.block(0, 0, n, 3) = projected_source_3d;
  projected_source_4d.col(3) = Eigen::VectorXd::Ones(n);
  //   Apply the transform
  Eigen::MatrixX4d projected_source_4d_in_plane =
      (source_to_plane.inverse() * projected_source_4d.transpose()).transpose();

  // Return only the 3D points in matrix form
  return projected_source_4d_in_plane.block(0, 0, n, 3);
}

Eigen::ArrayXd compute3DProjectionError(const Observation2D3D& obs, const CameraIntrinsics& intr,
                                        const Eigen::Isometry3d& camera_mount_to_camera,
                                        const Eigen::Isometry3d& target_mount_to_target)
{
  Eigen::Matrix3d camera_matrix;
  camera_matrix << intr.fx(), 0.0, intr.cx(), 0.0, intr.fy(), intr.cy(), 0.0, 0.0, 1.0;

  // Calculate the optimized transform from the camera to the target for the ith observation
  Eigen::Isometry3d camera_to_target =
      camera_mount_to_camera.inverse() * obs.to_camera_mount.inverse() * obs.to_target_mount * target_mount_to_target;

  // Convert the image and target features to matrices
  const Eigen::Index n = static_cast<Eigen::Index>(obs.correspondence_set.size());
  Eigen::MatrixX2d image_features(n, 2);
  Eigen::MatrixX3d target_features(n, 3);
  for (Eigen::Index i = 0; i < n; ++i)
  {
    image_features.row(i) = obs.correspondence_set[i].in_image;
    target_features.row(i) = obs.correspondence_set[i].in_target;
  }

  // Project the image features onto the 3D target plane
  Eigen::MatrixX3d projected_image_features_in_target = project3D(image_features, camera_to_target, camera_matrix);

  // Compute the error
  Eigen::ArrayXd error = (target_features - projected_image_features_in_target).rowwise().norm().array();

  return error;
}

Eigen::ArrayXd compute3DProjectionError(const Observation2D3D::Set& observations, const CameraIntrinsics& intr,
                                        const Eigen::Isometry3d& camera_mount_to_camera,
                                        const Eigen::Isometry3d& target_mount_to_target)
{
  Eigen::ArrayXd error;

  // Iterate over all of the images in which an observation of the target was made
  for (const Observation2D3D& obs : observations)
  {
    Eigen::Index n = static_cast<Eigen::Index>(obs.correspondence_set.size());

    // Append the projection error
    error.conservativeResize(error.rows() + n);
    error.tail(n) = compute3DProjectionError(obs, intr, camera_mount_to_camera, target_mount_to_target);
  }

  return error;
}

}  // namespace industrial_calibration
