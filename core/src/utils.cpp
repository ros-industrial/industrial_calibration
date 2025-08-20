#include <industrial_calibration/core/utils.h>

namespace industrial_calibration
{
Pose6d poseEigenToCal(const Eigen::Isometry3d& pose)
{
  Pose6d p;
  p.x() = pose.translation().x();
  p.y() = pose.translation().y();
  p.z() = pose.translation().z();

  Eigen::AngleAxisd aa(pose.linear());
  Eigen::Vector3d a = aa.axis() * aa.angle();

  p.rx() = a.x();
  p.ry() = a.y();
  p.rz() = a.z();
  return p;
}

Eigen::Isometry3d poseCalToEigen(const Pose6d& pose)
{
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.translation().x() = pose.x();
  p.translation().y() = pose.y();
  p.translation().z() = pose.z();

  const Eigen::Vector3d rr(pose.rx(), pose.ry(), pose.rz());
  const double dot_product = rr.dot(rr);

  // Check for 0-rotation
  if (dot_product > std::numeric_limits<double>::epsilon())
  {
    Eigen::AngleAxisd aa(rr.norm(), rr.normalized());
    p.linear() = aa.toRotationMatrix();
  }
  else
  {
    p.linear()(0, 0) = 1.0;  // Logic taken from Ceres' own aaxis to rot matrix conversion
    p.linear()(1, 0) = rr[2];
    p.linear()(2, 0) = -rr[1];
    p.linear()(0, 1) = -rr[2];
    p.linear()(1, 1) = 1.0;
    p.linear()(2, 1) = rr[0];
    p.linear()(0, 2) = rr[1];
    p.linear()(1, 2) = -rr[0];
    p.linear()(2, 2) = 1.0;
  }

  return p;
}

Eigen::Matrix<double, 3, 3, Eigen::RowMajor> calculateHomography(const Correspondence2D3D::Set correspondences)
{
  Eigen::MatrixXd A(2 * correspondences.size(), 8);
  Eigen::MatrixXd b(2 * correspondences.size(), 1);

  // Fill the A and B matrices with data from the selected correspondences
  for (std::size_t i = 0; i < correspondences.size(); ++i)
  {
    const Correspondence2D3D& corr = correspondences[i];

    // assign A row-th row:
    const double x = corr.in_target.x();
    const double y = corr.in_target.y();
    const double u = corr.in_image.x();
    const double v = corr.in_image.y();
    A.row(2 * i) << -x, -y, -1.0, 0.0, 0.0, 0.0, u * x, u * y;
    A.row(2 * i + 1) << 0.0, 0.0, 0.0, -x, -y, -1.0, v * x, v * y;

    b.block<2, 1>(2 * i, 0) = -1.0 * corr.in_image;
  }

  // Create the homography matrix
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> H = Eigen::Matrix3d::Ones();

  // Map the elements of the H matrix into a column vector and solve for the first 8
  {
    Eigen::Map<Eigen::VectorXd> hv(H.data(), 9);
    hv.head<8>() = A.fullPivLu().solve(b);
  }

  return H;
}

Eigen::Matrix2Xd projectHomography(const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>& H, const Eigen::Matrix2Xd& xy)
{
  Eigen::Matrix3Xd pts = Eigen::Matrix3Xd::Ones(3, xy.cols());
  pts.block(0, 0, 2, xy.cols()) = xy;

  // Perform the projection
  Eigen::Matrix3Xd uv = H * pts;

  // Calculate the per-point scaling factors (shape [n, 1])
  // k = 1.0 / (H[2, 0] * x + H[2, 1] * y + 1)
  Eigen::ArrayXd k = (H.row(2) * pts).array().cwiseInverse();

  // Apply the scale factor through elementwise multiplication
  uv.array().rowwise() *= k.transpose();

  return uv.block(0, 0, 2, uv.cols());
}

}  // namespace industrial_calibration
