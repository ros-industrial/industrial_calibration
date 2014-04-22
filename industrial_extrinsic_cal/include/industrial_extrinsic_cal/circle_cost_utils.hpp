#ifndef CERES_COST_UTILS_HPP
#define CERES_COST_UTILS_HPP

#include "ceres/ceres.h"
#include "ceres/rotation.h"

namespace industrial_extrinsic_cal
{

  class  CircleTargetCameraReprjError
  {
  public:

    CircleTargetCameraReprjError(double ob_x, double ob_y, double c_dia)
      : ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia){};

    template<typename T>
    bool operator()(const T* const c_p1, /**< extrinsic parameters [6]*/
		    const T* const c_p2, /**< 6Dof transform of target into world frame [6]*/
		    const T* const c_p3, /**< intrinsic parameters fx,fy,cx,cy,k1,k2,k2,p1,p2 [9]*/
		    const T* const point, /**< point described in target frame that is being seen [3]*/
		    T* resid) const;

    /** Factory to hide the construction of the CostFunction object from the client code*/
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia)
    {
      return (new ceres::AutoDiffCostFunction<CircleTargetCameraReprjError, 2, 6, 6, 9, 3>
	      (
	       new CircleTargetCameraReprjError(o_x, o_y, c_dia))
	      );
    }

  private:
    double ox_; /**< observed x location of object in image */
    double oy_; /**< observed y location of object in image */
    double circle_diameter_; //**< diameter of circle being observed */
  };

  // member function definitions
  template<typename T>
  bool industrial_extrinsic_cal::CircleTargetCameraReprjError::operator()(const T* const c_p1, /**< extrinsic parameters [6]*/
							const T* const c_p2, /**< 6Dof transform of target into world frame [6]*/
							const T* const c_p3, /**< intrinsic parameters of camera fx,fy,cx,cy,k1,k2,k2,p1,p2 [9]*/
							const T* const point, /**< point described in target frame that is being seen [3]*/
							T* resid) const
  {
    T camera_aa[3];		/**< angle axis  */
    T camera_loc[3];	/**< point rotated */
    T target_aa[3];		/**< angle axis for target */
    T target_loc[3];		/**< location of target */

    /** extract the variables from parameter blocks  */
    int q = 0; /**< a temporary variable */
    camera_aa[0]  = c_p1[q++]; /**<  angle_axis x for rotation of camera		 */
    camera_aa[1]  = c_p1[q++]; /**<  angle_axis y for rotation of camera */
    camera_aa[2]  = c_p1[q++]; /**<  angle_axis z for rotation of camera */
    camera_loc[0] = c_p1[q++]; /**<  translation of camera x (actualy of world in camera frame)*/
    camera_loc[1] = c_p1[q++]; /**<  translation of camera y (actualy of world in camera frame)*/
    camera_loc[2] = c_p1[q++]; /**<  translation of camera z (actualy of world in camera frame)*/

    /** extract target pose block of parameters */
    q = 0;
    target_aa[0]  = c_p2[q++]; /**<  target's ax angle axis value */
    target_aa[1]  = c_p2[q++]; /**<  target's ay angle axis value */
    target_aa[2]  = c_p2[q++]; /**<  target's az angle axis value */
    target_loc[0] = c_p2[q++]; /**<  target's x location */
    target_loc[1] = c_p2[q++]; /**<  target's y location */
    target_loc[2] = c_p2[q++]; /**<  target's z location */

    /** extract intrinsic block of parameters */
    q = 0;
    const T& fx  = c_p3[q++]; /**< focal length x */
    const T& fy  = c_p3[q++]; /**< focal length y */
    const T& cx  = c_p3[q++]; /**< central point x */
    const T& cy  = c_p3[q++]; /**< central point y */
    const T& k1  = c_p3[q++]; /**< distortion k1  */
    const T& k2  = c_p3[q++]; /**< distortion k2  */
    const T& k3  = c_p3[q++]; /**< distortion k3  */
    const T& p1  = c_p3[q++]; /**< distortion p1  */
    const T& p2  = c_p3[q++]; /**< distortion p2  */

    /** rotate and translate point into world frame */
    T world_point_loc[3];
    ceres::AngleAxisRotatePoint(target_aa, point, world_point_loc);

    /** apply target translation */
    world_point_loc[0] = world_point_loc[0] + target_loc[0];
    world_point_loc[1] = world_point_loc[1] + target_loc[1];
    world_point_loc[2] = world_point_loc[2] + target_loc[2];

    /** rotate and translate points into camera frame */
    /*  Note that camera transform is from world into camera frame, not vise versa */
    T camera_point_loc[3];
    ceres::AngleAxisRotatePoint(camera_aa, world_point_loc, camera_point_loc);

    /** apply camera translation */
    camera_point_loc[0] = camera_point_loc[0] + camera_loc[0]; /**< point rotated and translated */
    camera_point_loc[1] = camera_point_loc[1] + camera_loc[1];
    camera_point_loc[2] = camera_point_loc[2] + camera_loc[2];

    T xp1 = camera_point_loc[0]; // shorten variable names to make equations easier to read
    T yp1 = camera_point_loc[1];
    T zp1 = camera_point_loc[2];

    // Circle Delta is the difference between the projection of the center of the circle
    // and the center of the projected ellipse 

    // find rotation from target coordinates into camera coordinates
    // The 3 columns represent:
    // 1. the x-axis of the target in camera coordinates
    // 2. the y-axis of the target in camera coordinates
    // 3. the z-axis (normal of the target plane) in camera coordinates

    T R_TtoW[9];// rotation from target to world
    T R_WtoC[9];// rotation from world to camera
    T R_TtoC[9];// rotation from target to camera (computed as R_TtoC = R_WtoC*R_TtoW)
    ceres::AngleAxisToRotationMatrix(target_aa,R_TtoW);// output in column major order
    ceres::AngleAxisToRotationMatrix(camera_aa,R_WtoC);// output in column major order

    // x-axis of target in camera coordinates
    R_TtoC[0] = R_WtoC[0]*R_TtoW[0] +  R_WtoC[3]*R_TtoW[1] +  R_WtoC[6]*R_TtoW[2];
    R_TtoC[1] = R_WtoC[1]*R_TtoW[0] +  R_WtoC[4]*R_TtoW[1] +  R_WtoC[7]*R_TtoW[2];
    R_TtoC[2] = R_WtoC[2]*R_TtoW[0] +  R_WtoC[5]*R_TtoW[1] +  R_WtoC[8]*R_TtoW[2];
    // y axis of target in camera coordinates
    R_TtoC[3] = R_WtoC[0]*R_TtoW[3] +  R_WtoC[3]*R_TtoW[4] +  R_WtoC[6]*R_TtoW[5];
    R_TtoC[4] = R_WtoC[1]*R_TtoW[3] +  R_WtoC[4]*R_TtoW[4] +  R_WtoC[7]*R_TtoW[5];
    R_TtoC[5] = R_WtoC[2]*R_TtoW[3] +  R_WtoC[5]*R_TtoW[4] +  R_WtoC[8]*R_TtoW[5];
    // z axis of target(normal vector) of target in camera coordinates
    R_TtoC[6] = R_WtoC[0]*R_TtoW[6] +  R_WtoC[3]*R_TtoW[7] +  R_WtoC[6]*R_TtoW[8];
    R_TtoC[7] = R_WtoC[1]*R_TtoW[6] +  R_WtoC[4]*R_TtoW[7] +  R_WtoC[7]*R_TtoW[8];
    R_TtoC[8] = R_WtoC[2]*R_TtoW[6] +  R_WtoC[5]*R_TtoW[7] +  R_WtoC[8]*R_TtoW[8];
      
    // NOTE: we assume target is an XY planar target (all points on target have nominal z=0)
    //       in this case, the target plane is defined by the first two columns of R_TtoC
    //       these are the x and y vectors of the target frame expressed in the camera coordinates

    // Projection of distance vector D = [-xp1 -yp1 -zp1]/sqrt(xp1^2+yp1^2+zp1^2) on target plane
    T C2T_dist = sqrt(xp1*xp1 + yp1*yp1 + zp1*zp1); // distance from camera to point on target
    T D_targetx = -(xp1*R_TtoC[0] + yp1*R_TtoC[1] + zp1*R_TtoC[2])/C2T_dist; // projection on target_x
    T D_targety = -(xp1*R_TtoC[3] + yp1*R_TtoC[4] + zp1*R_TtoC[5])/C2T_dist; // projection on target_y
    T D_targetz = -(xp1*R_TtoC[6] + yp1*R_TtoC[7] + zp1*R_TtoC[8])/C2T_dist; // projection on target_z
    T s_theta   = D_targetz; // Note theta's the complemetary angle used to compute delta
    T c_theta   = sqrt( T(1.0) - s_theta*s_theta);

    // projection of D onto target xy plane expressed in camera frame is given by 
    // D_targetx * R_TtoC(1stcol) + D_targety * R_TtoC(2ndcol)
    T r = T(circle_diameter_/2.0);
    T Delta = r*s_theta*(T(1.0)/(C2T_dist-r*c_theta) - T(1.0)/(C2T_dist+r*c_theta))/T(2.0);
    
    // Vx, Vy vector in target plane
    T Vx = D_targetx*R_TtoC[0] + D_targety*R_TtoC[3];
    T Vy = D_targetx*R_TtoC[1] + D_targety*R_TtoC[4];
    T norm_vx = sqrt(Vx*Vx + Vy*Vy);
    Vx = Vx/norm_vx;
    Vy = Vy/norm_vx;
      
    /** scale into the image plane by distance away from camera */
    T xp = xp1 / zp1 + Delta*Vx;
    T yp = yp1 / zp1 + Delta*Vy;
    
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
    resid[0] = fx * xpp + cx - T(ox_);
    resid[1] = fy * ypp + cy - T(oy_);

    return true;
  } // end of operator() 

  class  CircleTargetCameraReprjErrorNoDistortion
  {
  public:
    
    CircleTargetCameraReprjErrorNoDistortion(double ob_x, double ob_y, double c_dia,
					     double fx, double fy, double cx, double cy)
      : ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), fx_(fx), fy_(fy), cx_(cx), cy_(cy){};

    template<typename T>
    bool operator()(const T* const c_p1, /**< extrinsic parameters [6]*/
		    const T* const c_p2, /**< 6Dof transform of target into world frame [6]*/
		    const T* const point, /**< point described in target frame that is being seen [3]*/
		    T* resid) const;
    
    /** Factory to hide the construction of the CostFunction object from the client code*/
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia,
				       const double fx, const double fy,
				       const double cx, const double cy)
    {
      return (new ceres::AutoDiffCostFunction<CircleTargetCameraReprjErrorNoDistortion, 2, 6, 6, 3>
	      (
	       new CircleTargetCameraReprjErrorNoDistortion(o_x, o_y, c_dia, fx, fy, cx, cy))
	      );
    }
    
  private:
    double ox_; /**< observed x location of object in image */
    double oy_; /**< observed y location of object in image */
    double circle_diameter_; //**< diameter of circle being observed */
    double fx_;			// focal length x
    double fy_;			// focal length y
    double cx_;			// optical center pixel x
    double cy_;			// optical center pixel y
  };
  
  // member function definitions
  template<typename T>
  bool industrial_extrinsic_cal::CircleTargetCameraReprjErrorNoDistortion::operator()
    (
     const T* const c_p1,	/**< extrinsic parameters [6]*/
     const T* const c_p2,	/**< 6Dof transform of target into world frame [6]*/
     const T* const point,	/**< point described in target frame that is being seen [3]*/
     T* resid
     ) const
  {
    T camera_aa[3];		/**< angle axis  */
    T camera_loc[3];		/**< point rotated */
    T target_aa[3];		/**< angle axis for target */
    T target_loc[3];		/**< location of target */
    
    /** extract the variables from parameter blocks  */
    int q = 0; /**< a temporary variable */
    camera_aa[0]  = c_p1[q++]; /**<  angle_axis x for rotation of camera		 */
    camera_aa[1]  = c_p1[q++]; /**<  angle_axis y for rotation of camera */
    camera_aa[2]  = c_p1[q++]; /**<  angle_axis z for rotation of camera */
    camera_loc[0] = c_p1[q++]; /**<  translation of camera x (actualy of world in camera frame)*/
    camera_loc[1] = c_p1[q++]; /**<  translation of camera y (actualy of world in camera frame)*/
    camera_loc[2] = c_p1[q++]; /**<  translation of camera z (actualy of world in camera frame)*/

    /** extract target pose block of parameters */
    q = 0;
    target_aa[0]  = c_p2[q++]; /**<  target's ax angle axis value */
    target_aa[1]  = c_p2[q++]; /**<  target's ay angle axis value */
    target_aa[2]  = c_p2[q++]; /**<  target's az angle axis value */
    target_loc[0] = c_p2[q++]; /**<  target's x location */
    target_loc[1] = c_p2[q++]; /**<  target's y location */
    target_loc[2] = c_p2[q++]; /**<  target's z location */

    /** rotate and translate point into world frame */
    T world_point_loc[3];
    ceres::AngleAxisRotatePoint(target_aa, point, world_point_loc);

    /** apply target translation */
    world_point_loc[0] = world_point_loc[0] + target_loc[0];
    world_point_loc[1] = world_point_loc[1] + target_loc[1];
    world_point_loc[2] = world_point_loc[2] + target_loc[2];

    /** rotate and translate points into camera frame */
    /*  Note that camera transform is from world into camera frame, not vise versa */
    T camera_point_loc[3];
    ceres::AngleAxisRotatePoint(camera_aa, world_point_loc, camera_point_loc);

    /** apply camera translation */
    camera_point_loc[0] = camera_point_loc[0] + camera_loc[0]; /**< point rotated and translated */
    camera_point_loc[1] = camera_point_loc[1] + camera_loc[1];
    camera_point_loc[2] = camera_point_loc[2] + camera_loc[2];

    T xp1 = camera_point_loc[0]; // shorten variable names to make equations easier to read
    T yp1 = camera_point_loc[1];
    T zp1 = camera_point_loc[2];

    // Circle Delta is the difference between the projection of the center of the circle
    // and the center of the projected ellipse 

    // find rotation from target coordinates into camera coordinates
    // The 3 columns represent:
    // 1. the x-axis of the target in camera coordinates
    // 2. the y-axis of the target in camera coordinates
    // 3. the z-axis (normal of the target plane) in camera coordinates

    T R_TtoW[9];// rotation from target to world
    T R_WtoC[9];// rotation from world to camera
    T R_TtoC[9];// rotation from target to camera (computed as R_TtoC = R_WtoC*R_TtoW)
    ceres::AngleAxisToRotationMatrix(target_aa,R_TtoW);// output in column major order
    ceres::AngleAxisToRotationMatrix(camera_aa,R_WtoC);// output in column major order

    // x-axis of target in camera coordinates
    R_TtoC[0] = R_WtoC[0]*R_TtoW[0] +  R_WtoC[3]*R_TtoW[1] +  R_WtoC[6]*R_TtoW[2];
    R_TtoC[1] = R_WtoC[1]*R_TtoW[0] +  R_WtoC[4]*R_TtoW[1] +  R_WtoC[7]*R_TtoW[2];
    R_TtoC[2] = R_WtoC[2]*R_TtoW[0] +  R_WtoC[5]*R_TtoW[1] +  R_WtoC[8]*R_TtoW[2];
    // y axis of target in camera coordinates
    R_TtoC[3] = R_WtoC[0]*R_TtoW[3] +  R_WtoC[3]*R_TtoW[4] +  R_WtoC[6]*R_TtoW[5];
    R_TtoC[4] = R_WtoC[1]*R_TtoW[3] +  R_WtoC[4]*R_TtoW[4] +  R_WtoC[7]*R_TtoW[5];
    R_TtoC[5] = R_WtoC[2]*R_TtoW[3] +  R_WtoC[5]*R_TtoW[4] +  R_WtoC[8]*R_TtoW[5];
    // z axis of target(normal vector) of target in camera coordinates
    R_TtoC[6] = R_WtoC[0]*R_TtoW[6] +  R_WtoC[3]*R_TtoW[7] +  R_WtoC[6]*R_TtoW[8];
    R_TtoC[7] = R_WtoC[1]*R_TtoW[6] +  R_WtoC[4]*R_TtoW[7] +  R_WtoC[7]*R_TtoW[8];
    R_TtoC[8] = R_WtoC[2]*R_TtoW[6] +  R_WtoC[5]*R_TtoW[7] +  R_WtoC[8]*R_TtoW[8];
      
    // NOTE: we assume target is an XY planar target (all points on target have nominal z=0)
    //       in this case, the target plane is defined by the first two columns of R_TtoC
    //       these are the x and y vectors of the target frame expressed in the camera coordinates

    // Projection of distance vector D = [-xp1 -yp1 -zp1]/sqrt(xp1^2+yp1^2+zp1^2) on target plane
    T C2T_dist = sqrt(xp1*xp1 + yp1*yp1 + zp1*zp1); // distance from camera to point on target
    T D_targetx = -(xp1*R_TtoC[0] + yp1*R_TtoC[1] + zp1*R_TtoC[2])/C2T_dist; // projection on target_x
    T D_targety = -(xp1*R_TtoC[3] + yp1*R_TtoC[4] + zp1*R_TtoC[5])/C2T_dist; // projection on target_y
    T D_targetz = -(xp1*R_TtoC[6] + yp1*R_TtoC[7] + zp1*R_TtoC[8])/C2T_dist; // projection on target_z
    T s_theta   = D_targetz; // Note theta's the complemetary angle used to compute delta
    T c_theta   = sqrt( T(1.0) - s_theta*s_theta);

    // projection of D onto target xy plane expressed in camera frame is given by 
    // D_targetx * R_TtoC(1stcol) + D_targety * R_TtoC(2ndcol)
    T r = T(circle_diameter_/2.0);
    T Delta = r*s_theta*(T(1.0)/(C2T_dist-r*c_theta) - T(1.0)/(C2T_dist+r*c_theta))/T(2.0);
    
    // Vx, Vy vector in target plane
    T Vx = D_targetx*R_TtoC[0] + D_targety*R_TtoC[3];
    T Vy = D_targetx*R_TtoC[1] + D_targety*R_TtoC[4];
    T norm_vx = sqrt(Vx*Vx + Vy*Vy);
    Vx = Vx/norm_vx;
    Vy = Vy/norm_vx;
      
    /** scale into the image plane by distance away from camera */
    T xp = xp1 / zp1 + Delta*Vx;
    T yp = yp1 / zp1 + Delta*Vy;
    
    /** perform projection using focal length and camera center into image plane */
    resid[0] = T(fx_) * xp + T(cx_) - T(ox_);
    resid[1] = T(fy_) * yp + T(cy_) - T(oy_);

    return true;
  } // end of operator() 

  class  CircleTargetCameraReprjErrorNoDFixedPoint
  {
  public:
    
    CircleTargetCameraReprjErrorNoDFixedPoint(double ob_x, double ob_y,
					      double c_dia,
					      double fx, 
					      double fy,
					      double cx,
					      double cy,
					      double point_x,
					      double point_y, 
					      double point_z)
      : ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), 
	fx_(fx), fy_(fy), cx_(cx), cy_(cy)
    {
      point_[0] = point_x;
      point_[1] = point_y;
      point_[2] = point_z;
    };

    template<typename T>
    bool operator()(const T* const c_p1, /**< extrinsic parameters [6]*/
		    const T* const c_p2, /**< 6Dof transform of target into world frame [6]*/
		    T* resid) const;
    
    /** Factory to hide the construction of the CostFunction object from the client code*/
    static ceres::CostFunction* Create(const double o_x, const double o_y, 
				       const double c_dia,
				       const double fx, const double fy,
				       const double cx, const double cy,
				       const double point_x, const double point_y, const double point_z)
    {
      return (new ceres::AutoDiffCostFunction<CircleTargetCameraReprjErrorNoDFixedPoint, 2, 6, 6 >
	      (
	       new CircleTargetCameraReprjErrorNoDFixedPoint(o_x, o_y,
							     c_dia, 
							     fx, fy, cx, cy,
							     point_x, point_y, point_z))
	      );
    }
    
  private:
    double ox_; /**< observed x location of object in image */
    double oy_; /**< observed y location of object in image */
    double circle_diameter_; //**< diameter of circle being observed */
    double fx_;			// focal length x
    double fy_;			// focal length y
    double cx_;			// optical center pixel x
    double cy_;			// optical center pixel y
    double point_[3];		// location of point in target frame

  };
  
  // member function definitions
  template<typename T>
  bool industrial_extrinsic_cal::CircleTargetCameraReprjErrorNoDFixedPoint::operator()
    (
     const T* const c_p1,	/**< extrinsic parameters [6]*/
     const T* const c_p2,	/**< 6Dof transform of target into world frame [6]*/
     T* resid
     ) const
  {
    T camera_aa[3];		/**< angle axis  */
    T camera_loc[3];		/**< point rotated */
    T target_aa[3];		/**< angle axis for target */
    T target_loc[3];		/**< location of target */
    
    /** extract the variables from parameter blocks  */
    int q = 0; /**< a temporary variable */
    camera_aa[0]  = c_p1[q++]; /**<  angle_axis x for rotation of camera		 */
    camera_aa[1]  = c_p1[q++]; /**<  angle_axis y for rotation of camera */
    camera_aa[2]  = c_p1[q++]; /**<  angle_axis z for rotation of camera */
    camera_loc[0] = c_p1[q++]; /**<  translation of camera x (actualy of world in camera frame)*/
    camera_loc[1] = c_p1[q++]; /**<  translation of camera y (actualy of world in camera frame)*/
    camera_loc[2] = c_p1[q++]; /**<  translation of camera z (actualy of world in camera frame)*/

    /** extract target pose block of parameters */
    q = 0;
    target_aa[0]  = c_p2[q++]; /**<  target's ax angle axis value */
    target_aa[1]  = c_p2[q++]; /**<  target's ay angle axis value */
    target_aa[2]  = c_p2[q++]; /**<  target's az angle axis value */
    target_loc[0] = c_p2[q++]; /**<  target's x location */
    target_loc[1] = c_p2[q++]; /**<  target's y location */
    target_loc[2] = c_p2[q++]; /**<  target's z location */

    /** rotate and translate point into world frame */
    T world_point_loc[3];
    T point[3];
    point[0] = T(point_[0]);// need to convert double to template type for auto-jacobian
    point[1] = T(point_[1]);
    point[2] = T(point_[2]);
    ceres::AngleAxisRotatePoint(target_aa, point, world_point_loc);

    /** apply target translation */
    world_point_loc[0] = world_point_loc[0] + target_loc[0];
    world_point_loc[1] = world_point_loc[1] + target_loc[1];
    world_point_loc[2] = world_point_loc[2] + target_loc[2];

    /** rotate and translate points into camera frame */
    /*  Note that camera transform is from world into camera frame, not vise versa */
    T camera_point_loc[3];
    ceres::AngleAxisRotatePoint(camera_aa, world_point_loc, camera_point_loc);

    /** apply camera translation */
    camera_point_loc[0] = camera_point_loc[0] + camera_loc[0]; /**< point rotated and translated */
    camera_point_loc[1] = camera_point_loc[1] + camera_loc[1];
    camera_point_loc[2] = camera_point_loc[2] + camera_loc[2];

    T xp1 = camera_point_loc[0]; // shorten variable names to make equations easier to read
    T yp1 = camera_point_loc[1];
    T zp1 = camera_point_loc[2];

    // Circle Delta is the difference between the projection of the center of the circle
    // and the center of the projected ellipse 

    // find rotation from target coordinates into camera coordinates
    // The 3 columns represent:
    // 1. the x-axis of the target in camera coordinates
    // 2. the y-axis of the target in camera coordinates
    // 3. the z-axis (normal of the target plane) in camera coordinates

    T R_TtoW[9];// rotation from target to world
    T R_WtoC[9];// rotation from world to camera
    T R_TtoC[9];// rotation from target to camera (computed as R_TtoC = R_WtoC*R_TtoW)
    ceres::AngleAxisToRotationMatrix(target_aa,R_TtoW);// output in column major order
    ceres::AngleAxisToRotationMatrix(camera_aa,R_WtoC);// output in column major order

    // x-axis of target in camera coordinates
    R_TtoC[0] = R_WtoC[0]*R_TtoW[0] +  R_WtoC[3]*R_TtoW[1] +  R_WtoC[6]*R_TtoW[2];
    R_TtoC[1] = R_WtoC[1]*R_TtoW[0] +  R_WtoC[4]*R_TtoW[1] +  R_WtoC[7]*R_TtoW[2];
    R_TtoC[2] = R_WtoC[2]*R_TtoW[0] +  R_WtoC[5]*R_TtoW[1] +  R_WtoC[8]*R_TtoW[2];
    // y axis of target in camera coordinates
    R_TtoC[3] = R_WtoC[0]*R_TtoW[3] +  R_WtoC[3]*R_TtoW[4] +  R_WtoC[6]*R_TtoW[5];
    R_TtoC[4] = R_WtoC[1]*R_TtoW[3] +  R_WtoC[4]*R_TtoW[4] +  R_WtoC[7]*R_TtoW[5];
    R_TtoC[5] = R_WtoC[2]*R_TtoW[3] +  R_WtoC[5]*R_TtoW[4] +  R_WtoC[8]*R_TtoW[5];
    // z axis of target(normal vector) of target in camera coordinates
    R_TtoC[6] = R_WtoC[0]*R_TtoW[6] +  R_WtoC[3]*R_TtoW[7] +  R_WtoC[6]*R_TtoW[8];
    R_TtoC[7] = R_WtoC[1]*R_TtoW[6] +  R_WtoC[4]*R_TtoW[7] +  R_WtoC[7]*R_TtoW[8];
    R_TtoC[8] = R_WtoC[2]*R_TtoW[6] +  R_WtoC[5]*R_TtoW[7] +  R_WtoC[8]*R_TtoW[8];
      
    // NOTE: we assume target is an XY planar target (all points on target have nominal z=0)
    //       in this case, the target plane is defined by the first two columns of R_TtoC
    //       these are the x and y vectors of the target frame expressed in the camera coordinates

    // Projection of distance vector D = [-xp1 -yp1 -zp1]/sqrt(xp1^2+yp1^2+zp1^2) on target plane
    T C2T_dist = sqrt(xp1*xp1 + yp1*yp1 + zp1*zp1); // distance from camera to point on target
    T D_targetx = -(xp1*R_TtoC[0] + yp1*R_TtoC[1] + zp1*R_TtoC[2])/C2T_dist; // projection on target_x
    T D_targety = -(xp1*R_TtoC[3] + yp1*R_TtoC[4] + zp1*R_TtoC[5])/C2T_dist; // projection on target_y
    T D_targetz = -(xp1*R_TtoC[6] + yp1*R_TtoC[7] + zp1*R_TtoC[8])/C2T_dist; // projection on target_z
    T s_theta   = D_targetz; // Note theta's the complemetary angle used to compute delta
    T c_theta   = sqrt( T(1.0) - s_theta*s_theta);

    // projection of D onto target xy plane expressed in camera frame is given by 
    // D_targetx * R_TtoC(1stcol) + D_targety * R_TtoC(2ndcol)
    T r = T(circle_diameter_/2.0);
    T Delta = r*s_theta*(T(1.0)/(C2T_dist-r*c_theta) - T(1.0)/(C2T_dist+r*c_theta))/T(2.0);
    
    // Vx, Vy vector in target plane
    T Vx = D_targetx*R_TtoC[0] + D_targety*R_TtoC[3];
    T Vy = D_targetx*R_TtoC[1] + D_targety*R_TtoC[4];
    T norm_vx = sqrt(Vx*Vx + Vy*Vy);
    Vx = Vx/norm_vx;
    Vy = Vy/norm_vx;
      
    /** scale into the image plane by distance away from camera */
    T xp = xp1 / zp1 + Delta*Vx;
    T yp = yp1 / zp1 + Delta*Vy;
    
    /** perform projection using focal length and camera center into image plane */
    resid[0] = T(fx_) * xp + T(cx_) - T(ox_);
    resid[1] = T(fy_) * yp + T(cy_) - T(oy_);

    return true;
  } // end of operator() 

}

#endif
