/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>
#include <industrial_extrinsic_cal/basic_types.h>

namespace industrial_extrinsic_cal
{
Pose6d::Pose6d(double tx, double ty, double tz, double aax, double aay, double aaz)
{
  x = tx;
  y = ty;
  z = tz;
  ax = aax;
  ay = aay;
  az = aaz;
}
Pose6d::Pose6d()
{
  x = y = z = ax = ay = az = 0.0;
}
void Pose6d::setBasis(tf::Matrix3x3& m)
{
  double R[9];
  R[0] = m[0][0];
  R[1] = m[1][0];
  R[2] = m[2][0];
  R[3] = m[0][1];
  R[4] = m[1][1];
  R[5] = m[2][1];
  R[6] = m[0][2];
  R[7] = m[1][2];
  R[8] = m[2][2];
  double angle_axis[3];
  ceres::RotationMatrixToAngleAxis(R, angle_axis);
  ax = angle_axis[0];
  ay = angle_axis[1];
  az = angle_axis[2];
}

void Pose6d::setOrigin(tf::Vector3& v)
{
  x = v.m_floats[0];
  y = v.m_floats[1];
  z = v.m_floats[2];
}

void Pose6d::setOrigin(double tx, double ty, double tz)
{
  x = tx;
  y = ty;
  z = tz;
}

void Pose6d::setEulerZYX(double ez, double ey, double ex)
{
  double ci = cos(ex);
  double cj = cos(ey);
  double ch = cos(ez);
  double si = sin(ex);
  double sj = sin(ey);
  double sh = sin(ez);
  double cc = ci * ch;
  double cs = ci * sh;
  double sc = si * ch;
  double ss = si * sh;

  tf::Matrix3x3 m;
  m[0][0] = cj * ch;
  m[0][1] = sj * sc - cs;
  m[0][2] = sj * cc + ss;
  m[1][0] = cj * sh;
  m[1][1] = sj * ss + cc;
  m[1][2] = sj * cs - sc;
  m[2][0] = -sj;
  m[2][1] = cj * si;
  m[2][2] = cj * ci;

  setBasis(m);
}

void Pose6d::setQuaternion(double qx, double qy, double qz, double qw)
{
  double angle = 2.0 * acos(qw);
  ax = qx / sqrt(1 - qw * qw) * angle;
  ay = qy / sqrt(1 - qw * qw) * angle;
  az = qz / sqrt(1 - qw * qw) * angle;
}

void Pose6d::setAngleAxis(double aax, double aay, double aaz)
{
  ax = aax;
  ay = aay;
  az = aaz;
}

tf::Matrix3x3 Pose6d::getBasis() const
{
  tf::Matrix3x3 R;
  double angle = sqrt(ax * ax + ay * ay + az * az);
  if (angle < .0001)
  {
    R[0][0] = 1.0;
    R[0][1] = 0.0;
    R[0][2] = 0.0;
    R[1][0] = 0.0;
    R[1][1] = 1.0;
    R[1][2] = 0.0;
    R[2][0] = 0.0;
    R[2][1] = 0.0;
    R[2][2] = 1.0;
    return (R);
  }
  double cos_theta = cos(angle);
  double sin_theta = sin(angle);
  double wx = ax / angle;
  double wy = ay / angle;
  double wz = az / angle;
  double omct = 1.0 - cos_theta;
  R[0][0] = cos_theta + wx * wx * omct;
  R[0][1] = wx * wy * omct - wz * sin_theta;
  R[0][2] = wx * wz * omct + wy * sin_theta;
  R[1][0] = wy * wx * omct + wz * sin_theta;
  R[1][1] = cos_theta + wy * wy * omct;
  R[1][2] = wy * wz * omct - wx * sin_theta;
  R[2][0] = wz * wx * omct - wy * sin_theta;
  R[2][1] = wz * wy * omct + wx * sin_theta;
  R[2][2] = cos_theta + wz * wz * omct;
  return (R);
}

tf::Vector3 Pose6d::getOrigin() const
{
  tf::Vector3 V(x, y, z);
  return (V);
}

void Pose6d::getEulerZYX(double& ez, double& ey, double& ex) const
{
  double PI = 4 * atan(1);
  double theta;
  double psi;
  double phi;
  tf::Matrix3x3 R = this->getBasis();

  if (fabs(R[2][0]) != 1.0)
  {  // cos(theta) = 0.0
    theta = -asin(R[2][0]);
    double cos_theta = cos(theta);
    psi = atan2(R[2][1] / cos_theta, R[2][2] / cos_theta);
    phi = atan2(R[1][0] / cos_theta, R[0][0] / cos_theta);
  }
  else
  {
    phi = 0.0;  // could be anything
    if (R[2][0] == -1.0)
    {
      theta = PI / 2.0;
      psi = phi + atan2(R[0][1], R[0][2]);
    }
    else
    {
      theta = -PI / 2.0;
      psi = -phi + atan2(-R[0][1], -R[0][2]);
    }
  }  // end of cos(theta) = 0

  // Rz(phi) * Ry(theta) * Rx(psi)
  ez = phi;
  ey = theta;
  ex = psi;
}
void Pose6d::getQuaternion(double& qx, double& qy, double& qz, double& qw) const
{
  double angle_axis[3];
  angle_axis[0] = ax;
  angle_axis[1] = ay;
  angle_axis[2] = az;
  double quaternion[4];
  ceres::AngleAxisToQuaternion(angle_axis, quaternion);
  qw = quaternion[0];
  qx = quaternion[1];
  qy = quaternion[2];
  qz = quaternion[3];
}

Pose6d Pose6d::getInverse() const
{
  double newx, newy, newz;
  tf::Matrix3x3 R = getBasis();
  newx = -(R[0][0] * x + R[1][0] * y + R[2][0] * z);
  newy = -(R[0][1] * x + R[1][1] * y + R[2][1] * z);
  newz = -(R[0][2] * x + R[1][2] * y + R[2][2] * z);
  Pose6d new_pose(newx, newy, newz, -ax, -ay, -az);
  return (new_pose);
}

void Pose6d::show(std::string message)
{
  tf::Matrix3x3 basis = this->getBasis();
  double ez_yaw, ey_pitch, ex_roll;
  double qx, qy, qz, qw;
  this->getEulerZYX(ez_yaw, ey_pitch, ex_roll);
  this->getQuaternion(qx, qy, qz, qw);
  printf("%s =[\n %6.4lf  %6.4lf  %6.4lf  %6.4lf\n  %6.4lf  %6.4lf  %6.4lf  %6.4lf\n  %6.4lf  %6.4lf %6.4lf  %6.4lf\n  "
         "%6.4lf  %6.4lf %6.4lf  %6.4lf];\n rpy= %6.4lf %6.4lf %6.4lf\n quat= %6.4lf  %6.4lf  %6.4lf %6.4lf\n ",
         message.c_str(), basis[0][0], basis[0][1], basis[0][2], this->x, basis[1][0], basis[1][1], basis[1][2],
         this->y, basis[2][0], basis[2][1], basis[2][2], this->z, 0.0, 0.0, 0.0, 1.0, ez_yaw, ey_pitch, ex_roll, qx, qy,
         qz, qw);
}

Pose6d Pose6d::operator*(Pose6d pose2) const
{
  tf::Matrix3x3 R1 = getBasis();
  tf::Matrix3x3 R2 = pose2.getBasis();
  tf::Vector3 T1 = getOrigin();
  tf::Vector3 T2 = pose2.getOrigin();

  tf::Matrix3x3 R3;
  R3[0][0] = R1[0][0] * R2[0][0] + R1[0][1] * R2[1][0] + R1[0][2] * R2[2][0];
  R3[1][0] = R1[1][0] * R2[0][0] + R1[1][1] * R2[1][0] + R1[1][2] * R2[2][0];
  R3[2][0] = R1[2][0] * R2[0][0] + R1[2][1] * R2[1][0] + R1[2][2] * R2[2][0];

  R3[0][1] = R1[0][0] * R2[0][1] + R1[0][1] * R2[1][1] + R1[0][2] * R2[2][1];
  R3[1][1] = R1[1][0] * R2[0][1] + R1[1][1] * R2[1][1] + R1[1][2] * R2[2][1];
  R3[2][1] = R1[2][0] * R2[0][1] + R1[2][1] * R2[1][1] + R1[2][2] * R2[2][1];

  R3[0][2] = R1[0][0] * R2[0][2] + R1[0][1] * R2[1][2] + R1[0][2] * R2[2][2];
  R3[1][2] = R1[1][0] * R2[0][2] + R1[1][1] * R2[1][2] + R1[1][2] * R2[2][2];
  R3[2][2] = R1[2][0] * R2[0][2] + R1[2][1] * R2[1][2] + R1[2][2] * R2[2][2];

  double tempx, tempy, tempz;
  tempx = R1[0][0] * T2.x() + R1[0][1] * T2.y() + R1[0][2] * T2.z() + T1.x();
  tempy = R1[1][0] * T2.x() + R1[1][1] * T2.y() + R1[1][2] * T2.z() + T1.y();
  tempz = R1[2][0] * T2.x() + R1[2][1] * T2.y() + R1[2][2] * T2.z() + T1.z();
  tf::Vector3 T3(tempx, tempy, tempz);

  Pose6d pose;
  pose.setBasis(R3);
  pose.setOrigin(T3);

  return (pose);
}

}  // end of namespace
