#include <industrial_extrinsic_cal/Swri_IRD_license.h>
#ifndef CERES_COSTS_UTILS_HPP_
#define CERES_COSTS_UTILS_HPP_

namespace industrial_extrinsic_cal {

/*! \brief local prototypes of helper functions */
void print_QTasH(double qx, double qy, double qz, double qw, double tx, double ty, double tz);
void print_AATasH(double x, double y, double z, double tx, double ty, double tz);
void print_AATasHI(double x, double y, double z, double tx, double ty, double tz);
void print_AAasEuler(double x, double y, double z);
void print_camera(Camera C, std::string words);
observation project_point(Camera C, point P);

/*! \brief  computes image of point in cameras image plane 
 *  \param  C both intrinsic and extrinsic camera parameters
 *  \param  P the point to be projected into image
*/
observation project_point(CameraParameters C, Point3d P)
{
  double p[3];
  double pt[3];
  pt[0] = P.x;
  pt[1] = P.y;
  pt[2] = P.z;
 
  /* transform point into camera frame */
  /* note, camera transform takes points from camera frame into world frame */
  ceres::AngleAxisRotatePoint(C.aa,pt,p);

  p[0] += C.pos[0];
  p[1] += C.pos[1];
  p[2] += C.pos[2];

  double xp = p[0]/p[2];			
  double yp = p[1]/p[2];

  double r2 = xp*xp + yp*yp;
  double r4 = r2*r2;
  double r6 = r2*r4;

  double xp2 = xp*xp; /* temporary variables square of others */
  double yp2 = yp*yp;

  /* apply the distortion coefficients to refine pixel location */
  double xpp = xp + C.k1*r2*xp + C.k2*r4*xp + C.k3*r6*xp + C.p2*(r2 + 2*xp2) + 2*C.p1*xp*yp;
  double ypp = yp + C.k1*r2*yp + C.k2*r4*yp + C.k3*r6*yp + C.p1*(r2 + 2*yp2) + 2*C.p2*xp*yp;

  /* perform projection using focal length and camera center into image plane */
  observation O;
  O.p_id = 0;
  O.x = C.fx*xpp + C.cx;
  O.y = C.fy*ypp + C.cy;
  return(O);
}

struct Camera_reprj_error{
  Camera_reprj_error(double ob_x, double ob_y)
    : Ox(ob_x),Oy(ob_y){}
  
  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters */
		  const T* c_p2,       /** intrinsic parameters */
		  const T* point,      /** point being projected, yes this is has 3 parameters */
		  T* resid) const {
    /** extract the variables from the camera parameters */
    int q=0; /** extrinsic block of parameters */
    const T& x    = c_p1[q++]; /**  angle_axis x for rotation of camera		 */
    const T& y    = c_p1[q++]; /**  angle_axis y for rotation of camera */
    const T& z    = c_p1[q++]; /**  angle_axis z for rotation of camera */
    const T& tx   = c_p1[q++]; /**  translation of camera x */
    const T& ty   = c_p1[q++]; /**  translation of camera y */
    const T& tz   = c_p1[q++]; /**  translation of camera z */

    q=0; /** intrinsic block of parameters */
    const T& fx   = c_p2[q++]; /**  focal length x */
    const T& fy   = c_p2[q++]; /**  focal length x */
    const T& cx   = c_p2[q++]; /**  center point x */
    const T& cy   = c_p2[q++]; /**  center point y */
    const T& k1   = c_p2[q++]; /**  distortion coefficient on 2nd order terms */
    const T& k2   = c_p2[q++]; /**  distortion coefficient on 4th order terms */
    const T& k3   = c_p2[q++]; /**  distortion coefficient on 6th order terms */
    const T& p1   = c_p2[q++]; /**  tangential distortion coefficient x */
    const T& p2   = c_p2[q++]; /**  tangential distortion coefficient y */

    /** rotate and translate points into camera frame */
    T aa[3];/** angle axis  */
    T p[3]; /** point rotated */
    aa[0] = x; 
    aa[1] = y; 
    aa[2] = z; 
    ceres::AngleAxisRotatePoint(aa,point,p);

    /** apply camera translation */
    T xp1 = p[0] + tx; /** point rotated and translated */
    T yp1 = p[1] + ty;
    T zp1 = p[2] + tz;

    /** scale into the image plane by distance away from camera */
    T xp = xp1/zp1;			
    T yp = yp1/zp1;

    /** calculate terms for polynomial distortion */
    T r2 = xp*xp + yp*yp;
    T r4 = r2*r2;
    T r6 = r2*r4;

    T xp2 = xp*xp; /** temporary variables square of others */
    T yp2 = yp*yp;
    /**apply the distortion coefficients to refine pixel location */
    T xpp = xp + k1*r2*xp + k2*r4*xp + k3*r6*xp + p2*(r2 + T(2.0)*xp2) + T(2.0)*p1*xp*yp;
    T ypp = yp + k1*r2*yp + k2*r4*yp + k3*r6*yp + p1*(r2 + T(2.0)*yp2) + T(2.0)*p2*xp*yp;
    /** perform projection using focal length and camera center into image plane */
    resid[0] = fx*xpp + cx - Ox; 
    resid[1] = fy*ypp + cy - Oy;

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y) {
    return (
	    new ceres::AutoDiffCostFunction<Camera_reprj_error, 2, 6, 9, 3>
	    (
	     new Camera_reprj_error(o_x, o_y)
	     )
            );
  }
  double Ox; /** observed x location of object in image */
  double Oy; /** observed y location of object in image */
};

/** print a quaternion plus position as a homogeneous transform */
void print_QTasH(double qx, double qy, double qz, double qw, double tx, double ty, double tz)
{
  double Rs11 = qw*qw + qx*qx - qy*qy - qz*qz;
  double Rs21 = 2.0*qx*qy + 2.0*qw*qz;
  double Rs31 = 2.0*qx*qz - 2.0*qw*qy;
  
  double Rs12 = 2.0*qx*qy - 2.0*qw*qz;
  double Rs22 = qw*qw - qx*qx + qy*qy - qz*qz;
  double Rs32 = 2.0*qy*qz + 2.0*qw*qx;
  
  double Rs13 = 2.0*qx*qz + 2.0*qw*qy;
  double Rs23 = 2.0*qy*qz - 2.0*qw*qx;
  double Rs33 = qw*qw - qx*qx - qy*qy + qz*qz;
  
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n",Rs11,Rs12,Rs13,tx);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n",Rs21,Rs22,Rs23,ty);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n",Rs31,Rs32,Rs33,tz);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n",0.0,0.0,0.0,1.0);
}

/** angle axis to homogeneous transform inverted */
void print_AATasH(double x, double y, double z, double tx, double ty, double tz)
{
  double R[9];
  double aa[3];
  aa[0] = x;
  aa[1] = y;
  aa[2] = z;
  ceres::AngleAxisToRotationMatrix(aa,R);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n",R[0],R[3],R[6],tx);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n",R[1],R[4],R[7],ty);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n",R[2],R[5],R[8],tz);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n",0.0,0.0,0.0,1.0);
}

/** angle axis to homogeneous transform */
void print_AATasHI(double x, double y, double z, double tx, double ty, double tz)
{
  double R[9];
  double aa[3];
  aa[0] = x;
  aa[1] = y;
  aa[2] = z;
  ceres::AngleAxisToRotationMatrix(aa,R);
  double ix = -(tx*R[0] + ty*R[1] + tz*R[2]);
  double iy = -(tx*R[3] + ty*R[4] + tz*R[5]);
  double iz = -(tx*R[6] + ty*R[7] + tz*R[8]);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n",R[0],R[1],R[2],ix);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n",R[3],R[4],R[5],iy);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n",R[6],R[7],R[8],iz);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n",0.0,0.0,0.0,1.0);
}

void print_AAasEuler(double x, double y, double z)
{
  double R[9];
  double aa[3];
  aa[0] = x;
  aa[1] = y;
  aa[2] = z;
  ceres::AngleAxisToRotationMatrix(aa,R);
  double rx = atan2(R[7],R[8]);
  double ry = atan2(-R[6],sqrt(R[7]*R[7] + R[8]*R[8]));
  double rz = atan2(R[3],R[0]);
  printf("rpy = %8.4f %8.4f %8.4f\n",rx,ry,rz);
}
void print_camera(Camera C,std::string words)
{
  printf("%s\n",words.c_str());
  printf("Camera to World Transform:\n");
  print_AATasHI(C.aa[0],C.aa[1],C.aa[2],C.pos[0],C.pos[1],C.pos[2]);      

  printf("World to Camera\n");
  print_AATasH(C.aa[0],C.aa[1],C.aa[2],C.pos[0],C.pos[1],C.pos[2]);      
  print_AAasEuler(C.aa[0],C.aa[1],C.aa[2]);
  printf("fx = %8.3lf fy = %8.3lf\n",C.fx,C.fy);
  printf("k1 = %8.3lf k2 = %8.3lf k3 = %8.3lf\n",C.k1, C.k2, C.k3);
  printf("p1 = %8.3lf p2 = %8.3lf\n", C.p1, C.p2);
  printf("cx = %8.3lf cy = %8.3lf\n", C.cx, C.cy);
}
} // end of namespace
#endif
