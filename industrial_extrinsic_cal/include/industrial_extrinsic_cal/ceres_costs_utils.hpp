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

#ifndef CERES_COSTS_UTILS_HPP_
#define CERES_COSTS_UTILS_HPP_

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <industrial_extrinsic_cal/basic_types.h>

namespace industrial_extrinsic_cal
{

  struct TargetCameraReprjErrorNoDistortionOLD
  {
    TargetCameraReprjErrorNoDistortionOLD(double ob_x, double ob_y,
				       double fx,   double fy,
				       double cx,   double cy,
				       double pnt_x,   double pnt_y, double pnt_z)
      : ox_(ob_x),oy_(ob_y),
	fx_(fx),fy_(fy),
	cx_(cx),cy_(cy),
        pnt_x_(pnt_x), pnt_y_(pnt_y), pnt_z_(pnt_z){}
  
    template <typename T>
    bool operator()(const T* const c_p1,   /** extrinsic parameters */
		    const T* const t_p1,   /** 6Dof transform of target points into world frame */
		    T* resid) const {

      /** extract the variables from parameter blocks  */
      int q=0; /** extract extrinsic block of parameters */
      const T& ax    = c_p1[q++]; /**  angle_axis x for rotation of camera		 */
      const T& ay    = c_p1[q++]; /**  angle_axis y for rotation of camera */
      const T& az    = c_p1[q++]; /**  angle_axis z for rotation of camera */
      const T& tx    = c_p1[q++]; /**  translation of camera x */
      const T& ty    = c_p1[q++]; /**  translation of camera y */
      const T& tz    = c_p1[q++]; /**  translation of camera z */

      q=0; /** extract target pose block of parameters */
      const T& target_ax  = t_p1[q++]; /**  target's ax angle axis value */
      const T& target_ay  = t_p1[q++]; /**  target's ay angle axis value */
      const T& target_az  = t_p1[q++]; /**  target's az angle axis value */
      const T& target_x   = t_p1[q++]; /**  target's x location */
      const T& target_y   = t_p1[q++]; /**  target's y location */
      const T& target_z   = t_p1[q++]; /**  target's z location */
      
      // create a vector from the location of the point in the target's frame
      T point[3];
      point[0] = T(pnt_x_);
      point[1] = T(pnt_y_);
      point[2] = T(pnt_z_);

      /** rotate and translate points into world frame */
      T target_aa[3];/** angle axis  */
      T world_point_loc[3]; /** point rotated */
      target_aa[0] = target_ax; 
      target_aa[1] = target_ay; 
      target_aa[2] = target_az; 
      //takes rotation as a Rodriques’ axis-angle vector, and point, returns world_point_loc as rotated point
      ceres::AngleAxisRotatePoint(target_aa,point,world_point_loc);

      /** apply target translation */
      world_point_loc[0] = world_point_loc[0] + target_x;
      world_point_loc[1] = world_point_loc[1] + target_y;
      world_point_loc[2] = world_point_loc[2] + target_z;

      /** rotate and translate points into camera frame */
      /*  Note that camera transform is from world into camera frame, not vise versa */
      T aa[3];/** angle axis  */
      T camera_point_loc[3]; /** point rotated */
      aa[0] = ax; 
      aa[1] = ay; 
      aa[2] = az; 
      ceres::AngleAxisRotatePoint(aa,point,camera_point_loc);

      /** apply camera translation */
      T xp1 = camera_point_loc[0] + tx; /** point rotated and translated */
      T yp1 = camera_point_loc[1] + ty;
      T zp1 = camera_point_loc[2] + tz;

      /** scale into the image plane by distance away from camera */
      T xp = xp1/zp1;			
      T yp = yp1/zp1;

      /** perform projection using focal length and camera center into image plane */
      resid[0] = T(fx_)*xp + T(cx_) - T(ox_); 
      resid[1] = T(fy_)*yp + T(cy_) - T(oy_);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y,
				       const double fx,  const double fy,
				       const double cx,  const double cy,
				       const double pnt_x, const double pnt_y,
                                       const double pnt_z)
  {
    return (new ceres::AutoDiffCostFunction<TargetCameraReprjErrorNoDistortionOLD, 2, 6, 6>(
        new TargetCameraReprjErrorNoDistortionOLD(o_x, o_y, fx, fy, cx, cy, pnt_x, pnt_y, pnt_z)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double fx_; /*!< known focal length of camera in x */
    double fy_; /*!< known focal length of camera in y */
    double cx_; /*!< known optical center of camera in x */
    double cy_; /*!< known optical center of camera in y */
    double pnt_x_;/*!< known location of point in target's reference frame x */
    double pnt_y_;/*!< known location of point in target's reference frame y */
    double pnt_z_;/*!< known location of point in target's reference frame z */
  };

  struct CameraReprjErrorNoDistortionOLD
  {
    CameraReprjErrorNoDistortionOLD(double ob_x, double ob_y, double fx, double fy, double cx, double cy) :
        ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy)
    {
    }

    template<typename T>
      bool operator()(const T* const c_p1, /** extrinsic parameters */
                      const T* c_p2, /** intrinsic parameters */
                      const T* point, /** point being projected, yes this is has 3 parameters */
                      T* resid) const
      {
        /** extract the variables from the camera parameters */
        int q = 0; /** extrinsic block of parameters */
        const T& ax = c_p1[q++]; /**  angle_axis x for rotation of camera           */
        const T& ay = c_p1[q++]; /**  angle_axis y for rotation of camera */
        const T& az = c_p1[q++]; /**  angle_axis z for rotation of camera */
        const T& tx = c_p1[q++]; /**  translation of camera x */
        const T& ty = c_p1[q++]; /**  translation of camera y */
        const T& tz = c_p1[q++]; /**  translation of camera z */

        q = 0; /** intrinsic block of parameters */
        const T& fx = c_p2[q++]; /**  focal length x */
        const T& fy = c_p2[q++]; /**  focal length x */
        const T& cx = c_p2[q++]; /**  center point x */
        const T& cy = c_p2[q++]; /**  center point y */

        /** rotate and translate points into camera frame */
        T aa[3];/** angle axis  */
        T p[3]; /** point rotated */
        T P[3]; /** point original */
        aa[0] = ax;
        aa[1] = ay;
        aa[2] = az;
        ceres::AngleAxisRotatePoint(aa, point, p);

        /** apply camera translation */
        T xp1 = p[0] + tx; /** point rotated and translated */
        T yp1 = p[1] + ty;
        T zp1 = p[2] + tz;

        /** scale into the image plane by distance away from camera */
        T xp = xp1 / zp1;
        T yp = yp1 / zp1;

        /** perform projection using focal length and camera center into image plane */
        resid[0] = T(fx_) * xp + T(cx_) - T(ox_);
        resid[1] = T(fy_) * yp + T(cy_) - T(oy_);

        return true;
      } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double f_x, const double f_y, const double c_x, const double c_y)
    {
      return (new ceres::AutoDiffCostFunction<CameraReprjErrorNoDistortionOLD, 2, 6, 9, 3>(new CameraReprjErrorNoDistortionOLD(o_x, o_y, f_x, f_y, c_x, c_y)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double fx_; /*!< known focal length of camera in x */
    double fy_; /*!< known focal length of camera in y */
    double cx_; /*!< known optical center of camera in x */
    double cy_; /*!< known optical center of camera in y */
  };

  struct CameraReprjErrorWithDistortionOLD
  {
    CameraReprjErrorWithDistortionOLD(double ob_x, double ob_y) :
        ox_(ob_x), oy_(ob_y)
    {
    }

    template<typename T>
      bool operator()(const T* const c_p1, /** extrinsic parameters */
                      const T* c_p2, /** intrinsic parameters */
                      const T* point, /** point being projected, yes this is has 3 parameters */
                      T* resid) const
      {
        /** extract the variables from the camera parameters */
        int q = 0; /** extrinsic block of parameters */
        const T& ax = c_p1[q++]; /**  angle_axis x for rotation of camera           */
        const T& ay = c_p1[q++]; /**  angle_axis y for rotation of camera */
        const T& az = c_p1[q++]; /**  angle_axis z for rotation of camera */
        const T& tx = c_p1[q++]; /**  translation of camera x */
        const T& ty = c_p1[q++]; /**  translation of camera y */
        const T& tz = c_p1[q++]; /**  translation of camera z */

        q = 0; /** intrinsic block of parameters */
        const T& fx = c_p2[q++]; /**  focal length x */
        const T& fy = c_p2[q++]; /**  focal length x */
        const T& cx = c_p2[q++]; /**  center point x */
        const T& cy = c_p2[q++]; /**  center point y */
        const T& k1 = c_p2[q++]; /**  distortion coefficient on 2nd order terms */
        const T& k2 = c_p2[q++]; /**  distortion coefficient on 4th order terms */
        const T& k3 = c_p2[q++]; /**  distortion coefficient on 6th order terms */
        const T& p1 = c_p2[q++]; /**  tangential distortion coefficient x */
        const T& p2 = c_p2[q++]; /**  tangential distortion coefficient y */

        /** rotate and translate points into camera frame */
        T aa[3];/** angle axis  */
        T p[3]; /** point rotated */
        aa[0] = ax;
        aa[1] = ay;
        aa[2] = az;
        ceres::AngleAxisRotatePoint(aa, point, p);

        /** apply camera translation */
        T xp1 = p[0] + tx; /** point rotated and translated */
        T yp1 = p[1] + ty;
        T zp1 = p[2] + tz;

        /** scale into the image plane by distance away from camera */
        T xp = xp1 / zp1;
        T yp = yp1 / zp1;

        /** calculate terms for polynomial distortion */
        T r2 = xp * xp + yp * yp;
        T r4 = r2 * r2;
        T r6 = r2 * r4;

        T xp2 = xp * xp; /** temporary variables square of others */
        T yp2 = yp * yp;
        /*apply the distortion coefficients to refine pixel location */
        T xpp = xp + k1 * r2 * xp + k2 * r4 * xp + k3 * r6 * xp + p2 * (r2 + T(2.0) * xp2) + T(2.0) * p1 * xp * yp;
        T ypp = yp + k1 * r2 * yp + k2 * r4 * yp + k3 * r6 * yp + p1 * (r2 + T(2.0) * yp2) + T(2.0) * p2 * xp * yp;
        /** perform projection using focal length and camera center into image plane */
        resid[0] = fx * xpp + cx - T(ox_);
        resid[1] = fy * ypp + cy - T(oy_);

        return true;
      } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y)
    {
      return (new ceres::AutoDiffCostFunction<CameraReprjErrorWithDistortionOLD, 2, 6, 9, 3>(new CameraReprjErrorWithDistortionOLD(o_x, o_y)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
  };

  // HELPER TEMPLATES  

  template<typename T>  void rotationProduct(const T R1[9], const T R2[9], T R3[9]);
  template<typename T> inline void rotationProduct(const T R1[9], const T R2[9], T R3[9])
  {
    // We assume that the rotation matrices are in column major order
    // x column
    R3[0] = R1[0]*R2[0] +  R1[3]*R2[1] +  R1[6]*R2[2];
    R3[1] = R1[1]*R2[0] +  R1[4]*R2[1] +  R1[7]*R2[2];
    R3[2] = R1[2]*R2[0] +  R1[5]*R2[1] +  R1[8]*R2[2];
    // y column
    R3[3] = R1[0]*R2[3] +  R1[3]*R2[4] +  R1[6]*R2[5];
    R3[4] = R1[1]*R2[3] +  R1[4]*R2[4] +  R1[7]*R2[5];
    R3[5] = R1[2]*R2[3] +  R1[5]*R2[4] +  R1[8]*R2[5];
    // z column
    R3[6] = R1[0]*R2[6] +  R1[3]*R2[7] +  R1[6]*R2[8];
    R3[7] = R1[1]*R2[6] +  R1[4]*R2[7] +  R1[7]*R2[8];
    R3[8] = R1[2]*R2[6] +  R1[5]*R2[7] +  R1[8]*R2[8];
  }

  template<typename T>  void extractCameraIntrinsics(const T intrinsics[9], T &fx, T &fy, T &cx, T &cy, T &k1, T &k2, T &k3, T &p1, T &p2);
  template<typename T> inline void extractCameraIntrinsics(const T intrinsics[9], T &fx, T &fy, T &cx, T &cy, T &k1, T &k2, T &k3, T &p1, T &p2)
  {
    fx  = intrinsics[0]; /** focal length x */
    fy  = intrinsics[1]; /** focal length y */
    cx  = intrinsics[2]; /** central point x */
    cy  = intrinsics[3]; /** central point y */
    k1  = intrinsics[4]; /** distortion k1  */
    k2  = intrinsics[5]; /** distortion k2  */
    k3  = intrinsics[6]; /** distortion k3  */
    p1  = intrinsics[7]; /** distortion p1  */
    p2  = intrinsics[8]; /** distortion p2  */
  }

  template<typename T>  void rotationInverse(const T R[9], const T RI[9]);
  template<typename T> inline void rotationInverse(const T R[9], T RI[9])
  {
    RI[0] = R[0]; RI[3] = R[1];  RI[6] = R[2];
    RI[1] = R[3]; RI[4] = R[4];  RI[7] = R[5];
    RI[2] = R[6]; RI[5] = R[7];  RI[8] = R[8];
  }
  template<typename T> inline void transformPoint(const T angle_axis[3], const T tx[3], const T point[3], T t_point[3]);
  template<typename T> inline void transformPoint(const T angle_axis[3], const T tx[3], const T point[3], T t_point[3])
  {
    ceres::AngleAxisRotatePoint(angle_axis, point, t_point);
    t_point[0] = t_point[0] + tx[0];
    t_point[1] = t_point[1] + tx[1];
    t_point[2] = t_point[2] + tx[2];
  }

  template<typename T> inline void poseTransformPoint(const Pose6d &pose, const T point[3], T t_point[3]);
  template<typename T> inline void poseTransformPoint(const Pose6d &pose, const T point[3], T t_point[3])
  {
    T angle_axis[3];
    angle_axis[0]  = T(pose.ax);
    angle_axis[1]  = T(pose.ay);
    angle_axis[2]  = T(pose.az);
    ceres::AngleAxisRotatePoint(angle_axis, point, t_point);
    t_point[0] = t_point[0] + T(pose.x);
    t_point[1] = t_point[1] + T(pose.y);
    t_point[2] = t_point[2] + T(pose.z);
  }

  template<typename T>  void transformPoint3d(const T angle_axis[3], const T tx[3], const Point3d &point, T t_point[3]);
  template<typename T> inline void transformPoint3d(const T angle_axis[3], const T tx[3], const Point3d &point, T t_point[3])
  {
    T point_[3];
    point_[0] = T(point.x);
    point_[1] = T(point.y);
    point_[2] = T(point.z);
    ceres::AngleAxisRotatePoint(angle_axis, point_, t_point);
    t_point[0] = t_point[0] + tx[0];
    t_point[1] = t_point[1] + tx[1];
    t_point[2] = t_point[2] + tx[2];
  }

  template<typename T>  void poseRotationMatrix(const Pose6d &pose, T R[9]);
  template<typename T> inline void poseRotationMatrix(const Pose6d &pose, T R[9])
  {
    T angle_axis[3];
    angle_axis[0] = T(pose.ax);
    angle_axis[1] = T(pose.ay);
    angle_axis[2] = T(pose.az);
    ceres::AngleAxisToRotationMatrix(angle_axis, R);
  }
  
  template<typename T>  void cameraPntResidualDist(T point[3], T &k1, T &k2, T &k3, T &p1, T &p2, T &fx, T &fy, T &cx, T &cy, T &ox, T &oy, T resid[2]);
  template<typename T> inline void cameraPntResidualDist(T point[3], T &k1, T &k2, T &k3, T &p1, T &p2, T &fx, T &fy, T &cx, T &cy, T &ox, T &oy, T resid[2])
  {
    T xp1 = point[0];
    T yp1 = point[1];
    T zp1 = point[2];

    /** scale into the image plane by distance away from camera */
    T xp = xp1 / zp1;
    T yp = yp1 / zp1;

    /** calculate terms for polynomial distortion */
    T r2 = xp * xp + yp * yp;
    T r4 = r2 * r2;
    T r6 = r2 * r4;

    T xp2 = xp * xp; /** temporary variables square of others */
    T yp2 = yp * yp;

    /*apply the distortion coefficients to refine pixel location */
    T xpp = xp + k1 * r2 * xp + k2 * r4 * xp + k3 * r6 * xp + p2 * (r2 + T(2.0) * xp2) + T(2.0) * p1 * xp * yp;
    T ypp = yp + k1 * r2 * yp + k2 * r4 * yp + k3 * r6 * yp + p1 * (r2 + T(2.0) * yp2) + T(2.0) * p2 * xp * yp;

    /** perform projection using focal length and camera center into image plane */
    resid[0] = fx * xpp + cx - ox;
    resid[1] = fy * ypp + cy - oy;

  }
  template<typename T>  void cameraCircResidualDist(T point[3], T &circle_diameter, T R_TtoC[9], 
							  T &k1, T &k2, T &k3, T &p1, T &p2, 
							  T &fx, T &fy, T &cx, T &cy, T &ox, T &oy, T resid[2]);
  template<typename T> inline void cameraCircResidualDist(T point[3], T &circle_diameter, T R_TtoC[9],
							  T &k1, T &k2, T &k3, T &p1, T &p2, 
							  T &fx, T &fy, T &cx, T &cy, T &ox, T &oy, T resid[2])
  {
    T xp1 = point[0];
    T yp1 = point[1];
    T zp1 = point[2];
    
    // Circle Delta is the difference between the projection of the center of the circle
    // and the center of the projected ellipse 

    // The 3 columns of R_TtoC represent:
    // 1. the x-axis of the target in camera coordinates
    // 2. the y-axis of the target in camera coordinates
    // 3. the z-axis (normal of the target plane) in camera coordinates
    // NOTE: we assume target is an XY planar target (all points on target have nominal z=0)
    
    // Find projection of distance vector D = [xp1 yp1 zp1] on to plane of target
    T D_targetx = xp1*R_TtoC[0] + yp1*R_TtoC[1] + zp1*R_TtoC[2];
    T D_targety = xp1*R_TtoC[3] + yp1*R_TtoC[4] + zp1*R_TtoC[5];
    
    // projection of D onto target xy plane expressed in camera frame is given by 
    // D_targetx * R_TtoC(1stcol) + D_targety * R_TtoC(2ndcol)
    
    // The vector of interest "Vperp" is orthogonal to this in the target xy plane
    // Vperp = -D_targety * R_TtoC(1stcol) + D_targetx * R_TtoC(2ndcol)
    // However we want Vperp to be in the direction with a negative z component
    
    T Vperp[3]; 
    Vperp[0] = -D_targety*R_TtoC[0] + D_targetx*R_TtoC[3] ;
    Vperp[1] = -D_targety*R_TtoC[1] + D_targetx*R_TtoC[4] ;
    Vperp[2] = -D_targety*R_TtoC[2] + D_targetx*R_TtoC[5] ;
    
    // Vector direction of Vperp is arbitrary, but need to specify direction closer to camera
    T mysign = -abs(Vperp[2])/Vperp[2]; // Warning, division by zero could happen
    Vperp[0] = mysign*Vperp[0];
    Vperp[1] = mysign*Vperp[1];
    Vperp[2] = mysign*Vperp[2];
        
    /** scale into the image plane by distance away from camera */
    T xp = xp1 / zp1;
    T yp = yp1 / zp1;
    
    if(zp1+Vperp[2] !=0.0){	// adjust only focal plan not parallel to target's xy plane
      T Vpx = (xp1+Vperp[0])/(zp1+Vperp[2]);
      T Vpy = (yp1+Vperp[1])/(zp1+Vperp[2]);
      T Vnorm = sqrt(Vpx*Vpx+Vpy*Vpy);
      if(Vnorm!=0.0){
	// find scale of motion
	// Delta = (r*sin(theta)/(D-rcos(theta)) - r*sin(theta)/(D+rcos(theta)))/2
	// where r is the radius of the circle being projected
	//       D is the distance between camera and circle center
	//       theta is the angle between D vector an target xy plane
	Vpx = Vpx/Vnorm;
	Vpy = Vpy/Vnorm;
	T D = sqrt(xp1*xp1 + yp1*yp1 + zp1*zp1);
	T s_theta = (R_TtoC[6]*xp1 + R_TtoC[7]*yp1 + R_TtoC[8]*zp1)/D;
	T c_theta = sqrt( T(1.0) - s_theta*s_theta);
	T r = T(circle_diameter/2.0);
	T Delta = r*s_theta*(T(1.0)/(D-r*c_theta) - T(1.0)/(D+r*c_theta))/T(2.0);
	xp = xp + Delta*Vpx;
	yp = yp + Delta*Vpy;
      }
    }
    
    /* temporary variables for distortion model */
    T xp2 = xp * xp;		/* x^2 */
    T yp2 = yp * yp;		/* y^2 */
    T r2  = xp2 + yp2;	/* r^2 radius squared */
    T r4  = r2 * r2;		/* r^4 */
    T r6  = r2 * r4;		/* r^6 */
    
    /* apply the distortion coefficients to refine pixel location */
    T xpp = xp 
      + k1 * r2 * xp		// 2nd order term
      + k2 * r4 * xp		// 4th order term
      + k3 * r6 * xp		// 6th order term
      + p2 * (r2 + T(2.0) * xp2) // tangential
      + p1 * xp * yp * T(2.0); // other tangential term
    T ypp = yp 
      + k1 * r2 * yp		// 2nd order term
      + k2 * r4 * yp		// 4th order term
      + k3 * r6 * yp		// 6th order term
      + p1 * (r2 + T(2.0) * yp2) // tangential term
      + p2 * xp * yp * T(2.0); // other tangential term
    
    /** perform projection using focal length and camera center into image plane */
    resid[0] = fx * xpp + cx - ox;
    resid[1] = fy * ypp + cy - oy;
  }

  template<typename T>  void cameraCircResidual(T point[3], T &circle_diameter, T R_TtoC[9],
						      T &fx, T &fy, T &cx, T &cy, T &ox, T &oy, T resid[2]);
  template<typename T> inline void cameraCircResidual(T point[3], T &circle_diameter, T R_TtoC[9],
						      T &fx, T &fy, T &cx, T &cy, T &ox, T &oy, T resid[2])
  {
    T xp1 = point[0];
    T yp1 = point[1];
    T zp1 = point[2];
    
    // Circle Delta is the difference between the projection of the center of the circle
    // and the center of the projected ellipse 
    
    // find rotation from target coordinates into camera coordinates
    // The 3 columns represent:
    // 1. the x-axis of the target in camera coordinates
    // 2. the y-axis of the target in camera coordinates
    // 3. the z-axis (normal of the target plane) in camera coordinates
    // NOTE: we assume target is an XY planar target (all points on target have nominal z=0)
    
    // Find projection of distance vector D = [xp1 yp1 zp1] on to plane of target
    T D_targetx = xp1*R_TtoC[0] + yp1*R_TtoC[1] + zp1*R_TtoC[2];
    T D_targety = xp1*R_TtoC[3] + yp1*R_TtoC[4] + zp1*R_TtoC[5];
    
    // projection of D onto target xy plane expressed in camera frame is given by 
    // D_targetx * R_TtoC(1stcol) + D_targety * R_TtoC(2ndcol)
    
    // The vector of interest "Vperp" is orthogonal to this in the target xy plane
    // Vperp = -D_targety * R_TtoC(1stcol) + D_targetx * R_TtoC(2ndcol)
    // However we want Vperp to be in the direction with a negative z component
    
    T Vperp[3]; 
    Vperp[0] = -D_targety*R_TtoC[0] + D_targetx*R_TtoC[3] ;
    Vperp[1] = -D_targety*R_TtoC[1] + D_targetx*R_TtoC[4] ;
    Vperp[2] = -D_targety*R_TtoC[2] + D_targetx*R_TtoC[5] ;
    
    // Vector direction of Vperp is arbitrary, but need to specify direction closer to camera
    T mysign = -abs(Vperp[2])/Vperp[2]; // Warning, division by zero could happen
    Vperp[0] = mysign*Vperp[0];
    Vperp[1] = mysign*Vperp[1];
    Vperp[2] = mysign*Vperp[2];
    
    /** scale into the image plane by distance away from camera */
    T xp = xp1 / zp1;
    T yp = yp1 / zp1;
    
    if(zp1+Vperp[2] !=0.0){	// adjust only focal plan not parallel to target's xy plane
      T Vpx = (xp1+Vperp[0])/(zp1+Vperp[2]);
      T Vpy = (yp1+Vperp[1])/(zp1+Vperp[2]);
      T Vnorm = sqrt(Vpx*Vpx+Vpy*Vpy);
      if(Vnorm!=0.0){
	// find scale of motion
	// Delta = (r*sin(theta)/(D-rcos(theta)) - r*sin(theta)/(D+rcos(theta)))/2
	// where r is the radius of the circle being projected
	//       D is the distance between camera and circle center
	//       theta is the angle between D vector an target xy plane
	Vpx = Vpx/Vnorm;
	Vpy = Vpy/Vnorm;
	T D = sqrt(xp1*xp1 + yp1*yp1 + zp1*zp1);
	T s_theta = (R_TtoC[6]*xp1 + R_TtoC[7]*yp1 + R_TtoC[8]*zp1)/D;
	T c_theta = sqrt( T(1.0) - s_theta*s_theta);
	T r = T(circle_diameter/2.0);
	T Delta = r*s_theta*(T(1.0)/(D-r*c_theta) - T(1.0)/(D+r*c_theta))/T(2.0);
	xp = xp + Delta*Vpx;
	yp = yp + Delta*Vpy;
      }
    }
    
    /** perform projection using focal length and camera center into image plane */
    resid[0] = fx * xp + cx - ox;
    resid[1] = fy * yp + cy - oy;
  }

  template<typename T>  void cameraPntResidual(T point[3], T &fx, T &fy, T &cx, T &cy, T &ox, T &oy, T resid[2]);
  template<typename T> inline void cameraPntResidual(T point[3], T &fx, T &fy, T &cx, T &cy, T &ox, T &oy, T resid[2])
  {
    T xp1 = point[0];
    T yp1 = point[1];
    T zp1 = point[2];

    /** scale into the image plane by distance away from camera */
    T xp = xp1 / zp1;
    T yp = yp1 / zp1;

    /** perform projection using focal length and camera center into image plane */
    resid[0] = fx * xp + cx - ox;
    resid[1] = fy * yp + cy - oy;

  }

  class CameraReprjErrorWithDistortion
  {
  public:
    CameraReprjErrorWithDistortion(double ob_x, double ob_y) :
      ox_(ob_x), oy_(ob_y)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    const T* c_p2, /** intrinsic parameters */
                    const T* point, /** point being projected, has 3 parameters */
                    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      T fx, fy, cx, cy, k1, k2, k3, p1, p2;
      extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
      T camera_point[3]; /** point in camera coordinates*/

      /** transform point into camera coordinates */
      transformPoint(camera_aa, camera_tx, point, camera_point);

      /** compute project point into image plane and compute residual */
      T ox = T(ox_);
      T oy = T(oy_);
      cameraPntResidualDist(camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy,  resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y)
    {
      return (new ceres::AutoDiffCostFunction<CameraReprjErrorWithDistortion, 2, 6, 9, 3>(new CameraReprjErrorWithDistortion(o_x, o_y)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
  };

  // reprojection error of a single simple point observed by a camera with lens distortion
  // typically used for intrinsic calibration
  // location of target is known, and fixed, point's location within target is also known
  // both extrinsic and intrinsic parameters of camera are being computed
  class CameraReprjErrorWithDistortionPK
  {
  public:
    CameraReprjErrorWithDistortionPK(double ob_x, double ob_y, Point3d point) :
      ox_(ob_x), oy_(ob_y), point_(point)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    const T* c_p2, /** intrinsic parameters */
                    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      T fx, fy, cx, cy, k1, k2, k3, p1, p2;
      extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
      T camera_point[3];

      /** transform point into camera coordinates */
      transformPoint3d(camera_aa, camera_tx, point_, camera_point);

      /** compute project point into image plane and compute residual */
      T ox = T(ox_);
      T oy = T(oy_);
      cameraPntResidualDist(camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy,  resid);
      
      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, Point3d point)
    {
      return (new ceres::AutoDiffCostFunction<CameraReprjErrorWithDistortionPK, 2, 6, 9>(new CameraReprjErrorWithDistortionPK(o_x, o_y, point)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    Point3d point_; /*! location of point in target coordinates */
  };

  // reprojection error of a single simple point observed by a camera with NO lens distortion
  // should subscribe to a rectified image when using the error function
  class CameraReprjError
  {
  public:
    CameraReprjError(double ob_x, double ob_y, double fx, double fy, double cx, double cy) :
      ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    const T* point, /** point being projected, yes this is has 3 parameters */
                    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      T camera_point[3]; /** point in camera coordinates */

      /** transform point into camera coordinates */
      transformPoint(camera_aa, camera_tx, point, camera_point);

      /** compute project point into image plane and compute residual */
      T fx = T(fx_);
      T fy = T(fy_);
      T cx = T(cx_);
      T cy = T(cy_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy,  resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, 
				       const double fx, const double fy, 
				       const double cx, const double cy)
    {
      return (new ceres::AutoDiffCostFunction<CameraReprjError, 2, 6, 3>(new CameraReprjError(o_x, o_y, fx, fy, cx, cy)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double fx_; /*!< known focal length of camera in x */
    double fy_; /*!< known focal length of camera in y */
    double cx_; /*!< known optical center of camera in x */
    double cy_; /*!< known optical center of camera in y */
  };

  // reprojection error of a single simple point observed by a camera with NO lens distortion
  // should subscribe to a rectified image when using the error function
  class CameraReprjErrorPK
  {
  public:
    CameraReprjErrorPK(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Point3d point) :
      ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), point_(point)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      T camera_point[3]; /** point in camera coordinates */

      /** transform point into camera coordinates */
      transformPoint3d(camera_aa, camera_tx, point_, camera_point);

      /** compute project point into image plane and compute residual */
      T fx = T(fx_);
      T fy = T(fy_);
      T cx = T(cx_);
      T cy = T(cy_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy,  resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, 
				       const double fx, const double fy, 
				       const double cx, const double cy,
				       const Point3d point)
    {
      return (new ceres::AutoDiffCostFunction<CameraReprjErrorPK, 2, 6>(new CameraReprjErrorPK(o_x, o_y, fx, fy, cx, cy, point)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double fx_; /*!< known focal length of camera in x */
    double fy_; /*!< known focal length of camera in y */
    double cx_; /*!< known optical center of camera in x */
    double cy_; /*!< known optical center of camera in y */
    Point3d point_; /*! location of point in target coordinates */

  };

  // reprojection error of a single point attatched to a target observed by a camera with NO lens distortion
  // should subscribe to a rectified image when using the error function
  //
  class TargetCameraReprjError
  {
  public:
    TargetCameraReprjError(double ob_x, double ob_y, double fx, double fy, double cx, double cy) :
      ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    const T* const c_p2, /** 6Dof transform of target points into world frame */
                    const T* const point, /** point described in target frame */
                    T* resid) const
    {

      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);

      T world_point[3]; /** point in world coordinates */
      T camera_point[3]; /** point in camera coordinates */

      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);

      /** compute project point into image plane and compute residual */
      T fx = T(fx_);
      T fy = T(fy_);
      T cx = T(cx_);
      T cy = T(cy_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy,  resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y,
				       const double fx, const double fy, 
				       const double cx, const double cy)

    {
      return (new ceres::AutoDiffCostFunction<TargetCameraReprjError, 2, 6, 6, 3>(new TargetCameraReprjError(o_x, o_y, fx, fy, cx, cy)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double fx_; /*!< known focal length of camera in x */
    double fy_; /*!< known focal length of camera in y */
    double cx_; /*!< known optical center of camera in x */
    double cy_; /*!< known optical center of camera in y */
  };

  // reprojection error of a single point attatched to a target observed by a camera with NO lens distortion
  // should subscribe to a rectified image when using the error function
  //
  class TargetCameraReprjErrorPK
  {
  public:
    TargetCameraReprjErrorPK(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Point3d point) :
      ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), point_(point)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    const T* const c_p2, /** 6Dof transform of target points into world frame */
                    T* resid) const
    {

      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T world_point[3]; /** point in world coordinates */
      T camera_point[3]; /** point in camera coordinates */

      /** transform point into camera coordinates */
      transformPoint3d(target_aa, target_tx, point_, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);

      /** compute project point into image plane and compute residual */
      T fx = T(fx_);
      T fy = T(fy_);
      T cx = T(cx_);
      T cy = T(cy_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy,  resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y,
				       const double fx, const double fy, 
				       const double cx, const double cy,
				       const Point3d point)

    {
      return (new ceres::AutoDiffCostFunction<TargetCameraReprjErrorPK, 2, 6, 6>(new TargetCameraReprjErrorPK(o_x, o_y, fx, fy, cx, cy, point)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double fx_; /*!< known focal length of camera in x */
    double fy_; /*!< known focal length of camera in y */
    double cx_; /*!< known optical center of camera in x */
    double cy_; /*!< known optical center of camera in y */
    Point3d point_; /*! location of point in target coordinates */
  };

  // reprojection error of a single point attatched to a target observed by a camera with NO lens distortion
  // should subscribe to a rectified image when using the error function
  //
  class LinkTargetCameraReprjError
  {
  public:
    LinkTargetCameraReprjError(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Pose6d link_pose) :
      ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), link_pose_(link_pose)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    const T* const c_p2, /** 6Dof transform of target points into world frame */
                    const T* const point, /** point described in target frame that is being seen */
                    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T link_point[3]; /** point in link coordinates */
      T world_point[3]; /** point in world coordinates */
      T camera_point[3]; /** point in camera coordinates */

      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point, link_point);
      poseTransformPoint(link_pose_, link_point, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);

      /** compute project point into image plane and compute residual */
      T fx = T(fx_);
      T fy = T(fy_);
      T cx = T(cx_);
      T cy = T(cy_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy,  resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double ox, const double oy,
				       const double fx, const double fy, 
				       const double cx, const double cy,
				       Pose6d pose)

    {
      return (new ceres::AutoDiffCostFunction<LinkTargetCameraReprjError, 2, 6, 6, 3>(new LinkTargetCameraReprjError(ox, oy, fx, fy, cx, cy, pose)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double fx_; /*!< known focal length of camera in x */
    double fy_; /*!< known focal length of camera in y */
    double cx_; /*!< known optical center of camera in x */
    double cy_; /*!< known optical center of camera in y */
    Pose6d link_pose_; /*!< transform from world to link coordinates */ 
  };

  // reprojection error of a single point attatched to a target observed by a camera with NO lens distortion
  // should subscribe to a rectified image when using the error function
  //
  class LinkTargetCameraReprjErrorPK
  {
  public:
    LinkTargetCameraReprjErrorPK(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Pose6d link_pose, Point3d point) :
      ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), link_pose_(link_pose), point_(point)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    const T* const c_p2, /** 6Dof transform of target points into world frame */
                    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T link_point[3]; /** point in link coordinates */
      T world_point[3];/** point in worls coordinates */			   
      T camera_point[3]; /** point in camera coordinates */

      /** transform point into camera coordinates */
      transformPoint3d(target_aa, target_tx, point_, link_point);
      poseTransformPoint(link_pose_, link_point, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);

      /** compute project point into image plane and compute residual */
      T fx = T(fx_);
      T fy = T(fy_);
      T cx = T(cx_);
      T cy = T(cy_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy,  resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y,
				       const double fx, const double fy, 
				       const double cx, const double cy,
				       Pose6d pose, Point3d point)

    {
      return (new ceres::AutoDiffCostFunction<LinkTargetCameraReprjErrorPK, 2, 6, 6>( new LinkTargetCameraReprjErrorPK(o_x, o_y, fx, fy, cx, cy, pose, point)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double fx_; /*!< known focal length of camera in x */
    double fy_; /*!< known focal length of camera in y */
    double cx_; /*!< known optical center of camera in x */
    double cy_; /*!< known optical center of camera in y */
    Pose6d link_pose_; /*!< transform from world to link coordinates */ 
    Point3d point_; /*! location of point in target coordinates */
  };

  // reprojection error of a single point attatched to a target observed by a camera with NO lens distortion
  // should subscribe to a rectified image when using the error function
  //
  class LinkCameraTargetReprjError
  {
  public:
    LinkCameraTargetReprjError(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Pose6d link_pose) :
      ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), link_pose_(link_pose)
    {
      link_posei_ = link_pose_.getInverse();
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    const T* const c_p2, /** 6Dof transform of target points into world frame */
                    const T* const point, /** point described in target frame that is being seen */
                    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T world_point[3]; /** point in world coordinates */
      T link_point[3]; /** point in link coordinates */
      T camera_point[3];/** point in camera coordinates */

      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point, world_point);
      poseTransformPoint(link_posei_, world_point, link_point);
      transformPoint(camera_aa, camera_tx, link_point, camera_point);

      /** compute project point into image plane and compute residual */
      T fx = T(fx_);
      T fy = T(fy_);
      T cx = T(cx_);
      T cy = T(cy_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy,  resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double ox, const double oy,
				       const double fx, const double fy, 
				       const double cx, const double cy,
				       Pose6d pose)

    {
      return (new ceres::AutoDiffCostFunction<LinkCameraTargetReprjError, 2, 6, 6, 3>(new LinkCameraTargetReprjError(ox, oy, fx, fy, cx, cy, pose)));
    }
    double ox_; /*!< observed x location of object in image */
    double oy_; /*!< observed y location of object in image */
    double fx_; /*!< known focal length of camera in x */
    double fy_; /*!< known focal length of camera in y */
    double cx_; /*!< known optical center of camera in x */
    double cy_; /*!< known optical center of camera in y */
    Pose6d link_pose_; /*!< transform from world to link coordinates */ 
    Pose6d link_posei_; /*!< transform from link to world coordinates */ 
  };

  // reprojection error of a single point attatched to a target observed by a camera with NO lens distortion
  // should subscribe to a rectified image when using the error function
  //
  class LinkCameraTargetReprjErrorPK
  {
  public:
    LinkCameraTargetReprjErrorPK(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Pose6d link_pose, Point3d point) :
      ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), link_pose_(link_pose), point_(point)
    {
      link_posei_ = link_pose_.getInverse();
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    const T* const c_p2, /** 6Dof transform of target points into world frame */
                    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T world_point[3]; /** point in world coordinates */
      T link_point[3]; /** point in link coordinates */
      T camera_point[3]; /** point in camera coordinates */

      /** transform point into camera coordinates */
      transformPoint3d(target_aa, target_tx, point_, world_point);
      poseTransformPoint(link_posei_, world_point, link_point);
      transformPoint(camera_aa, camera_tx, link_point, camera_point);

      /** compute project point into image plane and compute residual */
      T fx = T(fx_);
      T fy = T(fy_);
      T cx = T(cx_);
      T cy = T(cy_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraPntResidual(camera_point, fx, fy, cx, cy, ox, oy,  resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y,
				       const double fx, const double fy, 
				       const double cx, const double cy,
				       Pose6d pose, Point3d pnt)

    {
      return (new ceres::AutoDiffCostFunction<LinkCameraTargetReprjErrorPK, 2, 6, 6>( new LinkCameraTargetReprjErrorPK(o_x, o_y, fx, fy, cx, cy, pose, pnt)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double fx_; /*!< known focal length of camera in x */
    double fy_; /*!< known focal length of camera in y */
    double cx_; /*!< known optical center of camera in x */
    double cy_; /*!< known optical center of camera in y */
    Pose6d link_pose_; /*!< transform from world to link coordinates */ 
    Pose6d link_posei_; /*!< transform from link to world coordinates */ 
    Point3d point_; /*! location of point in target coordinates */
  };

  // WARNING, ASSUMES CIRCLE LIES IN XY PLANE OF WORLD
  class  CircleCameraReprjErrorWithDistortion
  {
  public:
    CircleCameraReprjErrorWithDistortion(double ob_x, double ob_y, double c_dia) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6]*/
		    const T* const c_p2, /** intrinsic parameters of camera fx,fy,cx,cy,k1,k2,k2,p1,p2 [9]*/
		    const T* const point, /** point described in target frame that is being seen [3]*/
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      T fx, fy, cx, cy, k1, k2, k3, p1, p2;      
      extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
      T camera_point[3];  /** point in camera coordinates*/       
      T R_TtoC[9];

      /** transform point into camera coordinates */
      transformPoint(camera_aa, camera_tx, point, camera_point);
      
      // find rotation from target to camera frame
      ceres::AngleAxisToRotationMatrix(camera_aa,R_TtoC);

      /** compute project point into image plane and compute residual */
      T circle_diameter = T(circle_diameter_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraCircResidualDist(camera_point, circle_diameter, R_TtoC, k1, k2, k3, p1,p2, fx, fy,cx,cy, ox, oy, resid);

      return true;
    } /** end of operator() */
    
    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia)
    {
      return (new ceres::AutoDiffCostFunction<CircleCameraReprjErrorWithDistortion, 2, 6, 9, 3>(new CircleCameraReprjErrorWithDistortion(o_x, o_y, c_dia)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; //** diameter of circle being observed */
  };

  // WARNING, ASSUMES CIRCLE LIES IN XY PLANE OF WORLD
  class  CircleCameraReprjErrorWithDistortionPK
  {
  public:
    CircleCameraReprjErrorWithDistortionPK(double ob_x, double ob_y, double c_dia, Point3d point) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), point_(point)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
		    const T* const c_p2, /** intrinsic parameters of camera fx,fy,cx,cy,k1,k2,k2,p1,p2 [9] */
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      T fx, fy, cx, cy, k1, k2, k3, p1, p2;      
      extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
      T camera_point[3]; /** point in camera coordinates */
      T R_TtoC[9]; /** rotation from target to camera coordinates 

      /** find point in camera coordinates */
      transformPoint3d(camera_aa, camera_tx, point_, camera_point);

      // find rotation from target to camera coordinates
      ceres::AngleAxisToRotationMatrix(camera_aa, R_TtoC);

      /** compute project point into image plane and compute residual */
      T circle_diameter = T(circle_diameter_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraCircResidualDist(camera_point, circle_diameter, R_TtoC, k1, k2, k3, p1, p2, fx, fy,cx,cy, ox, oy, resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, Point3d point)
    {
      return (new ceres::AutoDiffCostFunction<CircleCameraReprjErrorWithDistortionPK, 2, 6, 9>(											       new CircleCameraReprjErrorWithDistortionPK(o_x, o_y, c_dia, point)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; //** diameter of circle being observed */
    Point3d point_;
  };

  class  CircleCameraReprjError
  {
  public:
    CircleCameraReprjError(double ob_x, double ob_y, double c_dia, double fx, double fy, double cx, double cy) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), fx_(fx), fy_(fy), cx_(cx), cy_(cy)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6]*/
		    const T* const point, /** point described in target frame that is being seen [3]*/
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      T camera_point[3];  /** point in camera coordinates */ 
      T R_TtoC[9]; /** rotation from target to camera coordinates */

      /** transform point into camera coordinates */
      transformPoint(camera_aa, camera_tx, point, camera_point);
      
      // find rotation from target to camera coordinates
      ceres::AngleAxisToRotationMatrix(camera_aa,R_TtoC);

      /** compute project point into image plane and compute residual */
      T circle_diameter = T(circle_diameter_);
      T fx = T(fx_);
      T fy = T(fy_);
      T cx = T(cx_);
      T cy = T(cy_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraCircResidual(camera_point, circle_diameter, R_TtoC, fx, fy,cx,cy, ox, oy, resid);

      return true;
    } /** end of operator() */
    
    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, 
				       const double fx,  const double fy, const double cx, const double cy)
    {
      return (new ceres::AutoDiffCostFunction<CircleCameraReprjError, 2, 6, 3>(new CircleCameraReprjError(o_x, o_y, c_dia, fx, fy, cx, cy)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; /** diameter of circle being observed */
    double fx_; /** focal length of camera in x (pixels) */
    double fy_; /** focal length of camera in y (pixels) */
    double cx_; /** focal center of camera in x (pixels) */
    double cy_; /** focal center of camera in y (pixels) */
  };

  class  CircleCameraReprjErrorPK
  {
  public:
    CircleCameraReprjErrorPK(double ob_x, double ob_y, double c_dia, double fx, double fy, double cx, double cy,  Point3d point) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), fx_(fx), fy_(fy), cx_(cx), cy_(cy), point_(point)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      T camera_point[3]; /* point in camera coordinates */
      T R_TtoC[9]; /** rotation from target to camera coordinates */

      /** rotate and translate point into camera coordinates*/
      transformPoint3d(camera_aa, camera_tx, point_, camera_point);

      // find rotation from target to camera coordinates
      ceres::AngleAxisToRotationMatrix(camera_aa, R_TtoC);

      /** compute project point into image plane and compute residual */
      T circle_diameter = T(circle_diameter_);
      T fx = T(fx_);
      T fy = T(fy_);
      T cx = T(cx_);
      T cy = T(cy_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraCircResidual(camera_point, circle_diameter, R_TtoC, fx, fy,cx,cy, ox, oy, resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, 
				     const double fx, const double fy, const double cx, const double cy,
				     Point3d point)
    {
      return (new ceres::AutoDiffCostFunction<CircleCameraReprjErrorPK, 2, 6>(new CircleCameraReprjErrorPK(o_x, o_y, c_dia, fx, fy, cx, cy, point)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; //** diameter of circle being observed */
    double fx_; /** focal length of camera in x (pixels) */
    double fy_; /** focal length of camera in y (pixels) */
    double cx_; /** focal center of camera in x (pixels) */
    double cy_; /** focal center of camera in y (pixels) */
    Point3d point_; /** location of point in target coordinates */
  };

  class  CircleTargetCameraReprjErrorWithDistortion
  {
  public:
    CircleTargetCameraReprjErrorWithDistortion(double ob_x, double ob_y, double c_dia) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6]*/
		    const T* const c_p2, /** intrinsic parameters of camera fx,fy,cx,cy,k1,k2,k2,p1,p2 [9]*/
		    const T* const c_p3, /** 6Dof transform of target into world frame [6]*/
		    const T* const point, /** point described in target frame that is being seen [3]*/
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p3[0]);
      const T *target_tx(& c_p3[3]);
      T fx, fy, cx, cy, k1, k2, k3, p1, p2;      
      extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
      T world_point[3]; /** point in world coordinates */
      T camera_point[3];  /** point in camera coordinates*/ 
      T R_WtoC[9]; // rotation from world to camera coordinates
      T R_TtoW[9]; // rotation from target to world coordinates
      T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

      /** compute necessary rotation matrices */
      ceres::AngleAxisToRotationMatrix(camera_aa, R_WtoC);  
      ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);

      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);

      // find rotation from target to camera coordinates
      rotationProduct(R_WtoC, R_TtoW, R_TtoC); // R_WtoC*R_TtoW = R_TtoC

      /** compute project point into image plane and compute residual */
      T circle_diameter = T(circle_diameter_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraCircResidualDist(camera_point, circle_diameter, R_TtoC, k1, k2, k3, p1, p2, fx, fy,cx,cy, ox, oy, resid);

      return true;
    } /** end of operator() */
    
    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia)
    {
      return (new ceres::AutoDiffCostFunction<CircleTargetCameraReprjErrorWithDistortion, 2, 6, 6, 9, 3>(new CircleTargetCameraReprjErrorWithDistortion(o_x, o_y, c_dia)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; //** diameter of circle being observed */
  };

  class  CircleTargetCameraReprjErrorWithDistortionPK
  {
  public:
    CircleTargetCameraReprjErrorWithDistortionPK(double ob_x, double ob_y, double c_dia, Point3d point) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), point_(point)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
		    const T* const c_p2,  /** 6Dof transform of target into world frame [6] */
		    const T* const c_p3, /** intrinsic parameters of camera fx,fy,cx,cy,k1,k2,k2,p1,p2 [9] */
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p3[0]);
      const T *target_tx(& c_p3[3]);
      T fx, fy, cx, cy, k1, k2, k3, p1, p2;      
      extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
      T world_point[3]; /** point in world coordinates */
      T camera_point[3]; /** point in world coordinates */
      T R_WtoC[9]; // rotation from world to camera coordinates
      T R_TtoW[9]; // rotation from target to world coordinates
      T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

      /** compute necessary rotation matrices */
      ceres::AngleAxisToRotationMatrix(camera_aa, R_WtoC);  
      ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);

      /** transform point into camera coordinates */
      transformPoint3d(target_aa, target_tx, point_, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);

      // find rotation from target to camera coordinates
      rotationProduct(R_WtoC, R_TtoW, R_TtoC); // R_WtoC*R_TtoW = R_TtoC

      /** compute project point into image plane and compute residual */
      T circle_diameter = T(circle_diameter_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraCircResidualDist(camera_point, circle_diameter, R_TtoC, k1, k2, k3, p1, p2, fx, fy,cx,cy, ox, oy, resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, Point3d point)
    {
      return (new ceres::AutoDiffCostFunction<CircleTargetCameraReprjErrorWithDistortionPK, 2, 6, 6, 9>(											       new CircleTargetCameraReprjErrorWithDistortionPK(o_x, o_y, c_dia, point)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; //** diameter of circle being observed */
    Point3d point_;
  };

  class  CircleTargetCameraReprjError
  {
  public:
    CircleTargetCameraReprjError(double ob_x, double ob_y, double c_dia, double fx, double fy, double cx, double cy) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), fx_(fx), fy_(fy)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6]*/
		    const T* const c_p2, /** 6Dof transform of target into world frame [6]*/
		    const T* const point, /** point described in target frame that is being seen [3]*/
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T world_point[3]; /** point in world coordinates */
      T camera_point[3];  /** point in camera coordinates*/ 
      T R_WtoC[9]; // rotation from world to camera coordinates
      T R_TtoW[9]; // rotation from target to world coordinates
      T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

      /** compute necessary rotation matrices */
      ceres::AngleAxisToRotationMatrix(camera_aa, R_WtoC);  
      ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);
      
      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);
      
      /** find rotation from target to camera coordinates */
      rotationProduct(R_WtoC, R_TtoW, R_TtoC); // R_WtoC*R_TtoW = R_TtoC
      
      /** compute project point into image plane and compute residual */
      T circle_diameter = T(circle_diameter_);
      T fx = T(fx_);
      T fy = T(fy_);
      T cx = T(cx_);
      T cy = T(cy_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraCircResidual(camera_point, circle_diameter, R_TtoC, fx, fy,cx,cy, ox, oy, resid);

      return true;
    } /** end of operator() */
    
    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, 
				       const double fx, const double fy, const double cx, const double cy)
    {
      return (new ceres::AutoDiffCostFunction<CircleTargetCameraReprjError, 2, 6, 6, 3>(new CircleTargetCameraReprjError(o_x, o_y, c_dia, fx, fy, cx, cy)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; //** diameter of circle being observed */
    double fx_; /** focal length of camera in x (pixels) */
    double fy_; /** focal length of camera in y (pixels) */
    double cx_; /** focal center of camera in x (pixels) */
    double cy_; /** focal center of camera in y (pixels) */
  };

  class  CircleTargetCameraReprjErrorPK
  {
  public:
    CircleTargetCameraReprjErrorPK(double ob_x, double ob_y, double c_dia, double fx, double fy, double cx, double cy, Point3d point) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), fx_(fx), fy_(fy), cx_(cx), cy_(cy), point_(point)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
		    const T* const c_p2,  /** 6Dof transform of target into world frame [6] */
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T world_point[3]; /** point in world coordinates */
      T camera_point[3]; /** point in camera coordinates */
      T R_WtoC[9]; // rotation from world to camera coordinates
      T R_TtoW[9]; // rotation from target to world coordinates
      T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

      /** compute necessary rotation matrices */
      ceres::AngleAxisToRotationMatrix(camera_aa, R_WtoC);  
      ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);

      /** transform point into camera frame */
      transformPoint3d(target_aa, target_tx, point_, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);

      /** find rotation from target to camera coordinates */
      rotationProduct(R_WtoC, R_TtoW, R_TtoC); // R_WtoC*R_TtoW = R_TtoC
      
      /** compute project point into image plane and compute residual */
      T circle_diameter = T(circle_diameter_);
      T fx = T(fx_);
      T fy = T(fy_);
      T cx = T(cx_);
      T cy = T(cy_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraCircResidual(camera_point, circle_diameter, R_TtoC, fx, fy,cx,cy, ox, oy, resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia,
				       const double fx, const double fy, const double cx, const double cy,
				       Point3d point)
    {
      return (new ceres::AutoDiffCostFunction<CircleTargetCameraReprjErrorPK, 2, 6, 6>(											       new CircleTargetCameraReprjErrorPK(o_x, o_y, c_dia, fx, fy, cx, cy, point)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; //** diameter of circle being observed */
    double fx_; /** focal length of camera in x (pixels) */
    double fy_; /** focal length of camera in y (pixels) */
    double cx_; /** focal center of camera in x (pixels) */
    double cy_; /** focal center of camera in y (pixels) */
    Point3d point_;
  };

  class  LinkCircleTargetCameraReprjError
  {
  public:
    LinkCircleTargetCameraReprjError(double ob_x, double ob_y, double c_dia, double fx, double fy, double cx, double cy, Pose6d link_pose) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), fx_(fx), fy_(fy), cx_(cx), cy_(cy), link_pose_(link_pose)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6]*/
		    const T* const c_p2, /** 6Dof transform of target into world frame [6]*/
		    const T* const point, /** point described in target frame that is being seen [3]*/
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T link_point[3]; /** point in link coordinates */
      T world_point[3]; /** point in world coordinates */
      T camera_point[3];  /** point in camera coordinates*/ 
      T R_WtoC[9]; // rotation from world to camera coordinates
      T R_TtoL[9]; // rotation from target to linkcoordinates
      T R_LtoW[9]; // rotation from link to world coordinates
      T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)
      T R_LtoC[9]; // rotation from link to camera coordinates

      /** get necessary rotation matrices */
      ceres::AngleAxisToRotationMatrix(camera_aa, R_WtoC);  
      ceres::AngleAxisToRotationMatrix(target_aa, R_TtoL);
      poseRotationMatrix(link_pose_, R_LtoW);

      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point, link_point);
      poseTransformPoint(link_pose_, link_point, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);
      
      /** find rotation from target to camera coordinates */
      rotationProduct(R_WtoC, R_LtoW, R_LtoC);
      rotationProduct(R_LtoC, R_TtoL, R_TtoC); // R_WtoC*R_LtoW*R_TtoL = R_TtoC
      
      /** compute project point into image plane and compute residual */
      T circle_diameter = T(circle_diameter_);
      T fx = T(fx_);
      T fy = T(fy_);
      T cx = T(cx_);
      T cy = T(cy_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraCircResidual(camera_point, circle_diameter, R_TtoC, fx, fy,cx,cy, ox, oy, resid);

      return true;
    } /** end of operator() */
    
    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia,
				       const double fx,  const double fy,
				       const double cx,  const double cy,
				       const Pose6d pose)
    {
      return (new ceres::AutoDiffCostFunction<LinkCircleTargetCameraReprjError, 2, 6, 6, 3>(new LinkCircleTargetCameraReprjError(o_x, o_y, c_dia, fx, fy, cx, cy, pose)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; /** diameter of circle being observed */
    Pose6d link_pose_; /** transform from link to world coordinates*/
    double fx_; /** focal length of camera in x (pixels) */
    double fy_; /** focal length of camera in y (pixels) */
    double cx_; /** focal center of camera in x (pixels) */
    double cy_; /** focal center of camera in y (pixels) */ 
 };

  class  LinkCircleTargetCameraReprjErrorPK
  {
  public:
  LinkCircleTargetCameraReprjErrorPK(double ob_x, double ob_y, double c_dia, double fx, double fy, double cx, double cy, 
				     Pose6d link_pose, Point3d point) :
    ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), fx_(fx), fy_(fy), cx_(cx), cy_(cy), link_pose_(link_pose), point_(point)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
		    const T* const c_p2,  /** 6Dof transform of target into world frame [6] */
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T link_point[3]; /** point in link coordinates */
      T world_point[3];/** point in world coordinates */
      T camera_point[3]; /** point in camera coordinates */
      T R_WtoC[9]; // rotation from world to camera coordinates
      T R_TtoL[9]; // rotation from target to linkcoordinates
      T R_LtoW[9]; // rotation from link to world coordinates
      T R_LtoC[9]; // rotation from link to camera coordinataes
      T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

      /** compute necessary rotation matrices */
      ceres::AngleAxisToRotationMatrix(camera_aa, R_WtoC);  
      ceres::AngleAxisToRotationMatrix(target_aa, R_TtoL);
      poseRotationMatrix(link_pose_,R_LtoW);

      /** transform point into camera frame */
      transformPoint3d(target_aa, target_tx, point_, link_point);
      poseTransformPoint(link_pose_, link_point, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);

      /** find rotation from target to camera coordinates */
      rotationProduct(R_WtoC, R_LtoW, R_LtoC);
      rotationProduct(R_LtoC, R_TtoL, R_TtoC); // R_WtoC*R_LtoW*R_TtoL = R_TtoC
      
      /** compute project point into image plane and compute residual */
      T circle_diameter = T(circle_diameter_);
      T fx = T(fx_);
      T fy = T(fy_);
      T cx = T(cx_);
      T cy = T(cy_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraCircResidual(camera_point, circle_diameter, R_TtoC, fx, fy,cx,cy, ox, oy, resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia,
				       const double fx,  const double fy,
				       const double cx,  const double cy,
				       const Pose6d pose, Point3d point)
    {
      return (new ceres::AutoDiffCostFunction<LinkCircleTargetCameraReprjErrorPK, 2, 6, 6>
	      (
	       new LinkCircleTargetCameraReprjErrorPK(o_x, o_y, c_dia, fx, fy, cx, cy, pose, point)
	       )
	      );
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; //** diameter of circle being observed */
    Pose6d link_pose_; /** transform from link to world coordinates*/
    double fx_; /** focal length of camera in x (pixels) */
    double fy_; /** focal length of camera in y (pixels) */
    double cx_; /** focal center of camera in x (pixels) */
    double cy_; /** focal center of camera in y (pixels) */
    Point3d point_; /** point expressed in target coordinates */
  };


  class  LinkCameraCircleTargetReprjError
  {
  public:
    LinkCameraCircleTargetReprjError(double ob_x, double ob_y, double c_dia,
				     double fx, double fy, double cx, double cy, Pose6d link_pose) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), fx_(fx), fy_(fy), cx_(cx), cy_(cy), link_pose_(link_pose)
    {
      link_posei_ = link_pose_.getInverse();
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6]*/
		    const T* const c_p2, /** 6Dof transform of target into world frame [6]*/
		    const T* const point, /** point described in target frame that is being seen [3]*/
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T world_point[3]; /** point in world coordinates */
      T link_point[3]; /** point in link coordinates */
      T camera_point[3];  /** point in camera coordinates*/ 
      T R_LtoC[9]; // rotation from link to camera coordinates
      T R_WtoL[9]; // rotation from world to link coordinates
      T R_WtoC[9]; // rotation from world to camera coordinates, and intermediate transform
      T R_TtoW[9]; // rotation from target to world coordinates
      T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

      /** compute necessary rotation matrices */
      ceres::AngleAxisToRotationMatrix(camera_aa, R_LtoC);  
      poseRotationMatrix(link_pose_, R_WtoL);
      ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);
      
      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point, world_point);
      poseTransformPoint(link_posei_, world_point, link_point);
      transformPoint(camera_aa, camera_tx, link_point, camera_point);
      
      /** find rotation from target to camera coordinates */
      rotationProduct(R_LtoC,R_WtoL, R_WtoC);
      rotationProduct(R_WtoC, R_TtoW, R_TtoC); 
      
      /** compute project point into image plane and compute residual */
      T circle_diameter = T(circle_diameter_);
      T fx = T(fx_);
      T fy = T(fy_);
      T cx = T(cx_);
      T cy = T(cy_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraCircResidual(camera_point, circle_diameter, R_TtoC, fx, fy,cx,cy, ox, oy, resid);

      return true;
    } /** end of operator() */
    
    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia,
				       double fx, double fy, double cx, double cy, const Pose6d pose)
    {
      return (new ceres::AutoDiffCostFunction<LinkCameraCircleTargetReprjError, 2, 6, 6, 3>
	      (new LinkCameraCircleTargetReprjError(o_x, o_y, c_dia, fx, fy, cx, cy, pose)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; /** diameter of circle being observed */
    Pose6d link_pose_; /** transform from link to world coordinates*/
    Pose6d link_posei_; /** transform from world to link coordinates*/
    double fx_; /** focal length of camera in x (pixels) */
    double fy_; /** focal length of camera in y (pixels) */
    double cx_; /** focal center of camera in x (pixels) */
    double cy_; /** focal center of camera in y (pixels) */

  };

  class  LinkCameraCircleTargetReprjErrorPK
  {
  public:
    LinkCameraCircleTargetReprjErrorPK(const double &ob_x, const double &ob_y, const double &c_dia,
				       const double &fx, const double &fy, const double &cx, const double &cy,
				       const Pose6d &link_pose, const Point3d &point) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), fx_(fx), fy_(fy), cx_(cx), cy_(cy), link_pose_(link_pose), point_(point)
    {
      link_posei_ = link_pose_.getInverse();
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
		    const T* const c_p2,  /** 6Dof transform of target into world frame [6] */
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(&c_p2[0]);
      const T *target_tx(&c_p2[3]);
      T point[3]; /** point in target coordinates */
      T world_point[3]; /** point in world coordinates */
      T link_point[3]; /** point in link coordinates */
      T camera_point[3];  /** point in camera coordinates*/ 
      T R_LtoC[9]; // rotation from link to camera coordinates
      T R_WtoL[9]; // rotation from world to link coordinates
      T R_WtoC[9]; // rotation from world to camera coordinates, and intermediate transform
      T R_TtoW[9]; // rotation from target to world coordinates
      T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

      /** get necessary rotation matrices */
      ceres::AngleAxisToRotationMatrix(camera_aa, R_LtoC);  
      poseRotationMatrix(link_posei_,R_WtoL);
      ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);

      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point, world_point);
      poseTransformPoint(link_posei_, world_point, link_point);
      transformPoint(camera_aa, camera_tx, link_point, camera_point);

      /** find rotation from target to camera coordinates */
      rotationProduct(R_LtoC, R_WtoL, R_WtoC);
      rotationProduct(R_WtoC, R_TtoW, R_TtoC); 

      /** compute project point into image plane and compute residual */
      T circle_diameter = T(circle_diameter_);
      T fx = T(fx_);
      T fy = T(fy_);
      T cx = T(cx_);
      T cy = T(cy_);
      T ox = T(ox_);
      T oy = T(oy_);
      cameraCircResidual(camera_point, circle_diameter, R_TtoC, fx, fy,cx,cy, ox, oy, resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double &o_x, const double &o_y, const double &c_dia,
				       const double &fx,  const double &fy,
				       const double &cx, const double &cy,
				       const Pose6d &pose, Point3d &point)
    {
      return (new ceres::AutoDiffCostFunction<LinkCameraCircleTargetReprjErrorPK, 2, 6, 6>
	      (new LinkCameraCircleTargetReprjErrorPK(o_x, o_y, c_dia, fx, fy, cx, cy, pose, point)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; //** diameter of circle being observed */
    Pose6d link_pose_; /** transform from link to world coordinates*/
    Pose6d link_posei_; /** transform from world to link coordinates*/
    double fx_; /** focal length of camera in x (pixels) */
    double fy_; /** focal length of camera in y (pixels) */
    double cx_; /** focal center of camera in x (pixels) */
    double cy_; /** focal center of camera in y (pixels) */
    Point3d point_; /** point expressed in target coordinates */

  };


} // end of namespace
#endif
