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
  struct TargetCameraReprjErrorNoDistortion
  {
    TargetCameraReprjErrorNoDistortion(double ob_x, double ob_y,
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
      const T& target_x   = t_p1[q++]; /**  target's x location */
      const T& target_y   = t_p1[q++]; /**  target's y location */
      const T& target_z   = t_p1[q++]; /**  target's z location */
      const T& target_ax  = t_p1[q++]; /**  target's ax angle axis value */
      const T& target_ay  = t_p1[q++]; /**  target's ay angle axis value */
      const T& target_az  = t_p1[q++]; /**  target's az angle axis value */

      
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
    return (new ceres::AutoDiffCostFunction<TargetCameraReprjErrorNoDistortion, 2, 6, 6>(
        new TargetCameraReprjErrorNoDistortion(o_x, o_y, fx, fy, cx, cy, pnt_x, pnt_y, pnt_z)));
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

  struct CameraReprjErrorNoDistortion
  {
    CameraReprjErrorNoDistortion(double ob_x, double ob_y, double fx, double fy, double cx, double cy) :
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
      return (new ceres::AutoDiffCostFunction<CameraReprjErrorNoDistortion, 2, 6, 9, 3>(new CameraReprjErrorNoDistortion(o_x, o_y, f_x, f_y, c_x, c_y)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double fx_; /*!< known focal length of camera in x */
    double fy_; /*!< known focal length of camera in y */
    double cx_; /*!< known optical center of camera in x */
    double cy_; /*!< known optical center of camera in y */
  };

  struct CameraReprjErrorWithDistortion
  {
    CameraReprjErrorWithDistortion(double ob_x, double ob_y) :
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
      return (new ceres::AutoDiffCostFunction<CameraReprjErrorWithDistortion, 2, 6, 9, 3>(new CameraReprjErrorWithDistortion(o_x, o_y)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
  };


} // end of namespace
#endif
