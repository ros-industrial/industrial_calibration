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
    x=y=z=ax=ay=az=0.0;
  }
  void Pose6d::setBasis( tf::Matrix3x3 & m)
  { // TODO this may have issues see rotation.h from ceres to fix STILL A TODO, st =0 will cause divide by zero
    double trace_R = m[0][0]+m[1][1]+m[2][2];
    double angle = acos((trace_R - 1.0)/2.0);
    double st = sin(angle);
    ax = (m[2][1]-m[1][2])/(2.0*st)*angle;
    ay = (m[0][2]-m[2][0])/(2.0*st)*angle;
    az = (m[1][0]-m[0][1])/(2.0*st)*angle;
  }

  void Pose6d::setOrigin(tf::Vector3 & v)
  {
    x = v[0];
    y = v[1];
    z = v[2];
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
    double cc = ci*ch;
    double cs = ci*sh;
    double sc = si*ch;
    double ss = si*sh;

    tf::Matrix3x3 m;
    m[0][0] = cj*ch;  m[0][1] = sj*sc - cs;     m[0][2] = sj*cc + ss;
    m[1][0] = cj*sh;  m[1][1] = sj*ss + cc;   m[1][2] = sj*cs - sc;
    m[2][0] = -sj;      m[2][1] = cj*si;           m[2][2] =cj*ci ;

    setBasis(m);
  }

  void Pose6d::setQuaternion(double qx, double qy, double qz, double qw)
  {
    double angle = 2.0 * acos(qw);
    ax = qx / sqrt(1-qw*qw)*angle;
    ay = qy / sqrt(1-qw*qw)*angle;
    az = qz / sqrt(1-qw*qw)*angle;
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
    double angle = sqrt(ax*ax + ay*ay + az*az);
    if(angle < .0001){
      R[0][0] = 1.0;  R[0][1] = 0.0;  R[0][2] = 0.0;
      R[1][0] = 0.0;  R[1][1] = 1.0;  R[1][2] = 0.0;
      R[2][0] = 0.0;  R[2][1] = 0.0;  R[2][2] = 1.0;
      return(R);
    }
    double ct = cos(angle);
    double st = sin(angle);
    double ux = ax/angle;
    double uy = ay/angle;
    double uz = az/angle;
    double omct = 1.0 - ct;
    R[0][0] = ct + ux*ux*omct;      R[0][1] = ux*uy*omct - uz*st;   R[0][2] = ux*uz*omct + uy*st; 
    R[1][0] = uy*ux*omct + uz*st; R[1][1] =  ct+uy*uy*omct;        R[1][2] = uy*uz*omct - ux*st;  
    R[2][0] = uz*ux*omct - uy*st;  R[2][1] =  uz*uy*omct + ux*st;  R[2][2] = ct + uz*uz*omct;     
    return(R);
  }

  tf::Vector3 Pose6d::getOrigin() const
  {
    tf::Vector3 V;
    V[0] = x;
    V[1] = y;
    V[2] = z;
    return(V);
  }

  //TODO
  //  void Pose6d::get_eulerZYX(double &ez, double &ey, double & ex)
  //  {
  //    
  //  }
  void Pose6d::getQuaternion(double &qx,  double &qy, double &qz, double &qw)
  {
    // the following was taken from ceres equivalent function
    double theta_squared = ax*ax + ay*ay + az*az;
    if(theta_squared>0.0){
      double theta = sqrt(theta_squared);
      double half_theta = theta*0.5;
      double k = sin(half_theta)/theta;
      qw = cos(half_theta);
      qx = ax*k;
      qy = ay*k;
      qz = az*k;
    }
    else{ // can't do division by zeor
      double k = 0.5;
      qw = 1.0;
      qx = ax*k;
      qy = ay*k;
      qz = az*k;
    }
  }

  Pose6d Pose6d::getInverse() const
  {
    double newx,newy,newz;
    tf::Matrix3x3 R = getBasis();
    newx =-( R[0][0] * x + R[1][0] * y + R[2][0] * z);
    newy = -(R[0][1] * x + R[1][1] * y + R[2][1] * z);
    newz = -(R[0][2] * x + R[1][2] * y + R[2][2] * z);
    Pose6d new_pose(newx, newy, newz, -ax,- ay, -az);
    return(new_pose);
  }

  Pose6d Pose6d::operator * ( Pose6d pose2) const
  {
    tf::Matrix3x3  R1   = getBasis();
    tf::Matrix3x3 R2 = pose2.getBasis();
    tf::Vector3 T1     = getOrigin();
    tf::Vector3 T2     = pose2.getOrigin();
    
    tf::Matrix3x3 R3;
    R3[0][0] = R1[0][0] * R2[0][0] + R1[0][1]*R2[1][0] + R1[0][2]*R2[2][0]; 
    R3[1][0] = R1[1][0] * R2[0][0] + R1[1][1]*R2[1][0] + R1[1][2]*R2[2][0];
    R3[2][0] = R1[2][0] * R2[0][0] + R1[2][1]*R2[1][0] + R1[2][2]*R2[2][0];

    R3[0][1] = R1[0][0] * R2[0][1] + R1[0][1]*R2[1][1] + R1[0][2]*R2[2][1]; 
    R3[2][1] = R1[1][0] * R2[0][1] + R1[1][1]*R2[1][1] + R1[1][2]*R2[2][1];
    R3[2][1] = R1[2][0] * R2[0][1] + R1[2][1]*R2[1][1] + R1[2][2]*R2[2][1];

     R3[0][2] = R1[0][0] * R2[0][2] + R1[0][1]*R2[1][2] + R1[0][2]*R2[2][2]; 
     R3[1][2] = R1[1][0] * R2[0][2] + R1[1][1]*R2[1][2] + R1[1][2]*R2[2][2];
     R3[2][2] = R1[2][0] * R2[0][2] + R1[2][1]*R2[1][2] + R1[2][2]*R2[2][2];

    tf::Vector3 T3;
    T3[0] = R1[0][0] * T2[0] + R1[0][1]*T2[1] + R1[0][2]*T2[2] + T1[0] ;
    T3[1] = R1[1][0] * T2[0] + R1[1][1]*T2[1] + R1[1][2]*T2[2] + T1[1] ;
    T3[2] = R1[1][0] * T2[0] + R1[2][1]*T2[1] + R1[2][2]*T2[2] + T1[2] ;
    
    Pose6d pose;
    pose.setBasis(R3);
    pose.setOrigin(T3);

    return(pose);
  }

}// end of namespace
