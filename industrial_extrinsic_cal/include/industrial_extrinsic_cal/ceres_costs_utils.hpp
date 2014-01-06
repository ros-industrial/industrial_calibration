/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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

namespace industrial_extrinsic_cal {

  /* local prototypes of helper functions */

/*! \brief print a quaternion plus position as a homogeneous transform 
 *  \param qx quaternion x value
 *  \param qy quaternion y value
 *  \param qz quaternion z value
 *  \param qw quaternion w value
 *  \param tx position x value
 *  \param ty position y value
 *  \param tz position z value
 */
void printQTasH(double qx, double qy, double qz, double qw, double tx, double ty, double tz);

/*! \brief print an angle axis transform as a homogeneous transform
 *  \param x angle axis x value
 *  \param y angle axis y value
 *  \param z angle axis z value
 *  \param tx position x value
 *  \param ty position y value
 *  \param tz position z value
 */
void printAATasH(double ax, double ay, double az, double tx, double ty, double tz);

/*! \brief print angle axis to homogeneous transform inverse
 *  \param ax angle axis x value
 *  \param ay angle axis y value
 *  \param az angle axis z value
 *  \param tx position x value
 *  \param ty position y value
 *  \param tz position z value
 */
void printAATasHI(double ax, double ay, double az, double tx, double ty, double tz);

/*! \brief print angle axis as euler angles
 *  \param ax angle axis x value
 *  \param ay angle axis y value
 *  \param az angle axis z value
 */
void printAAasEuler(double ax, double ay, double az);

/*! \brief print Camera Parameters
 *  \param CameraParameters include intrinsic and extrinsic
 *  \param words to provide as a header
 */
void printCameraParameters(CameraParameters C, std::string words);

/*! \brief  computes image of point in cameras image plane 
 *  \param  C both intrinsic and extrinsic camera parameters
 *  \param  P the point to be projected into image
*/
Observation projectPoint(CameraParameters camera_parameters, Point3d point);

  struct TargetCameraReprjErrorNoDistortion{
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
		    const T* const c_p2,   /** 6Dof transform of target points into world frame */
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
      const T& target_x   = c_p2[q++]; /**  target's x location */
      const T& target_y   = c_p2[q++]; /**  target's y location */
      const T& target_z   = c_p2[q++]; /**  target's z location */
      const T& target_ax  = c_p2[q++]; /**  target's ax angle axis value */
      const T& target_ay  = c_p2[q++]; /**  target's ay angle axis value */
      const T& target_az  = c_p2[q++]; /**  target's az angle axis value */

      
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
				       const double pnt_x, const double pnt_y, const double pnt_z
) {
      return (
	      new ceres::AutoDiffCostFunction<TargetCameraReprjErrorNoDistortion, 2,  6, 6>
	      (
	       new TargetCameraReprjErrorNoDistortion(o_x, o_y, fx, fy, cx, cy, pnt_x, pnt_y, pnt_z)
	       )
	      );
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

} // end of namespace
#endif
