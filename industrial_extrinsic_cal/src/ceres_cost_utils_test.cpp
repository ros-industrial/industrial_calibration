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

#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/observation_scene.h>

namespace industrial_extrinsic_cal
{
Observation projectPoint(CameraParameters C, Point3d P)
{
  double p[3];
  double pt[3];
  pt[0] = P.x;
  pt[1] = P.y;
  pt[2] = P.z;

  /* transform point into camera frame */
  /* note, camera transform takes points from camera frame into world frame */
  double aa[3];
  aa[0] = C.pb_extrinsics[0];
  aa[0] = C.pb_extrinsics[1];
  aa[0] = C.pb_extrinsics[2];
  ceres::AngleAxisRotatePoint(aa, pt, p);

  p[0] += C.pb_extrinsics[3];
  p[1] += C.pb_extrinsics[4];
  p[2] += C.pb_extrinsics[5];

  double xp = p[0] / p[2];
  double yp = p[1] / p[2];

  // calculate terms for polynomial distortion
  double r2 = xp * xp + yp * yp;
  double r4 = r2 * r2;
  double r6 = r2 * r4;

  double xp2 = xp * xp; /* temporary variables square of others */
  double yp2 = yp * yp;

  /* apply the distortion coefficients to refine pixel location */
  double xpp = xp + C.distortion_k1 * r2 * xp + C.distortion_k2 * r4 * xp + C.distortion_k3 * r6 * xp +
               C.distortion_p2 * (r2 + 2 * xp2) + 2 * C.distortion_p1 * xp * yp;
  double ypp = yp + C.distortion_k1 * r2 * yp + C.distortion_k2 * r4 * yp + C.distortion_k3 * r6 * yp +
               C.distortion_p1 * (r2 + 2 * yp2) + 2 * C.distortion_p2 * xp * yp;

  /* perform projection using focal length and camera center into image plane */
  Observation O;
  O.point_id = 0;
  O.image_loc_x = C.focal_length_x * xpp + C.center_x;
  O.image_loc_y = C.focal_length_y * ypp + C.center_y;
  return (O);
}

struct CameraReprjErrorWithDistortion
{
  CameraReprjErrorWithDistortion(double ob_x, double ob_y) : ox_(ob_x), oy_(ob_y)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters */
                  const T* c_p2,       /** intrinsic parameters */
                  const T* point,      /** point being projected, yes this is has 3 parameters */
                  T* resid) const
  {
    /** extract the variables from the camera parameters */
    int q = 0;               /** extrinsic block of parameters */
    const T& x = c_p1[q++];  /**  angle_axis x for rotation of camera           */
    const T& y = c_p1[q++];  /**  angle_axis y for rotation of camera */
    const T& z = c_p1[q++];  /**  angle_axis z for rotation of camera */
    const T& tx = c_p1[q++]; /**  translation of camera x */
    const T& ty = c_p1[q++]; /**  translation of camera y */
    const T& tz = c_p1[q++]; /**  translation of camera z */

    q = 0;                   /** intrinsic block of parameters */
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
    T aa[3]; /** angle axis  */
    T p[3];  /** point rotated */
    aa[0] = x;
    aa[1] = y;
    aa[2] = z;
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
    return (new ceres::AutoDiffCostFunction<CameraReprjErrorWithDistortion, 2, 6, 9, 3>(
        new CameraReprjErrorWithDistortion(o_x, o_y)));
  }
  double ox_; /** observed x location of object in image */
  double oy_; /** observed y location of object in image */
};

struct CameraReprjErrorNoDistortion
{
  CameraReprjErrorNoDistortion(double ob_x, double ob_y) : ox_(ob_x), oy_(ob_y)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters */
                  const T* c_p2,       /** intrinsic parameters */
                  const T* point,      /** point being projected, yes this is has 3 parameters */
                  T* resid) const
  {
    /** extract the variables from the camera parameters */
    int q = 0;               /** extrinsic block of parameters */
    const T& x = c_p1[q++];  /**  angle_axis x for rotation of camera		 */
    const T& y = c_p1[q++];  /**  angle_axis y for rotation of camera */
    const T& z = c_p1[q++];  /**  angle_axis z for rotation of camera */
    const T& tx = c_p1[q++]; /**  translation of camera x */
    const T& ty = c_p1[q++]; /**  translation of camera y */
    const T& tz = c_p1[q++]; /**  translation of camera z */

    q = 0;                   /** intrinsic block of parameters */
    const T& fx = c_p2[q++]; /**  focal length x */
    const T& fy = c_p2[q++]; /**  focal length x */
    const T& cx = c_p2[q++]; /**  center point x */
    const T& cy = c_p2[q++]; /**  center point y */

    /** rotate and translate points into camera frame */
    T aa[3]; /** angle axis  */
    T p[3];  /** point rotated */
    aa[0] = x;
    aa[1] = y;
    aa[2] = z;
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
  static ceres::CostFunction* Create(const double o_x, const double o_y)
  {
    return (new ceres::AutoDiffCostFunction<CameraReprjErrorNoDistortion, 2, 6, 4, 3>(
        new CameraReprjErrorNoDistortion(o_x, o_y)));
  }
  double ox_; /** observed x location of object in image */
  double oy_; /** observed y location of object in image */
  double fx_; /*!< known focal length of camera in x */
  double fy_; /*!< known focal length of camera in y */
  double cx_; /*!< known optical center of camera in x */
  double cy_; /*!< known optical center of camera in y */
};

struct TargetCameraReprjErrorNoDistortion
{
  TargetCameraReprjErrorNoDistortion(double ob_x, double ob_y) : ox_(ob_x), oy_(ob_y)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1,  /** extrinsic parameters */
                  const T* const c_p2,  /** intrinsic parameters */
                  const T* const c_p3,  /** 6Dof transform of target points into world frame */
                  const T* const point, /** point described in target frame that is being seen */
                  T* resid) const
  {
    /** extract the variables from parameter blocks  */
    int q = 0;               /** extract extrinsic block of parameters */
    const T& ax = c_p1[q++]; /**  angle_axis x for rotation of camera		 */
    const T& ay = c_p1[q++]; /**  angle_axis y for rotation of camera */
    const T& az = c_p1[q++]; /**  angle_axis z for rotation of camera */
    const T& tx = c_p1[q++]; /**  translation of camera x */
    const T& ty = c_p1[q++]; /**  translation of camera y */
    const T& tz = c_p1[q++]; /**  translation of camera z */

    q = 0;                   /** extract intrinsic block of parameters */
    const T& fx = c_p2[q++]; /**  focal length x */
    const T& fy = c_p2[q++]; /**  focal length x */
    const T& cx = c_p2[q++]; /**  center point x */
    const T& cy = c_p2[q++]; /**  center point y */

    q = 0;                          /** extract target pose block of parameters */
    const T& target_x = c_p3[q++];  /**  target's x location */
    const T& target_y = c_p3[q++];  /**  target's y location */
    const T& target_z = c_p3[q++];  /**  target's z location */
    const T& target_ax = c_p3[q++]; /**  target's ax angle axis value */
    const T& target_ay = c_p3[q++]; /**  target's ay angle axis value */
    const T& target_az = c_p3[q++]; /**  target's az angle axis value */

    /** rotate and translate points into world frame */
    T target_aa[3];       /** angle axis  */
    T world_point_loc[3]; /** point rotated */
    target_aa[0] = target_ax;
    target_aa[1] = target_ay;
    target_aa[2] = target_az;
    ceres::AngleAxisRotatePoint(target_aa, point, world_point_loc);

    /** apply target translation */
    world_point_loc[0] = world_point_loc[0] + target_x;
    world_point_loc[1] = world_point_loc[1] + target_y;
    world_point_loc[2] = world_point_loc[2] + target_z;

    /** rotate and translate points into camera frame */
    /*  Note that camera transform is from world into camera frame, not vise versa */
    T aa[3];               /** angle axis  */
    T camera_point_loc[3]; /** point rotated */
    aa[0] = ax;
    aa[1] = ay;
    aa[2] = az;
    ceres::AngleAxisRotatePoint(aa, point, camera_point_loc);

    /** apply camera translation */
    T xp1 = camera_point_loc[0] + tx; /** point rotated and translated */
    T yp1 = camera_point_loc[1] + ty;
    T zp1 = camera_point_loc[2] + tz;

    /** scale into the image plane by distance away from camera */
    T xp = xp1 / zp1;
    T yp = yp1 / zp1;

    /** perform projection using focal length and camera center into image plane */
    resid[0] = fx * xp + cx - T(ox_);
    resid[1] = fy * yp + cy - T(oy_);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y)
  {
    return (new ceres::AutoDiffCostFunction<TargetCameraReprjErrorNoDistortion, 2, 6, 4, 6, 3>(
        new CameraReprjError(o_x, o_y)));
  }
  double ox_; /** observed x location of object in image */
  double oy_; /** observed y location of object in image */
};

struct TargetCameraReprjErrorProjModelAsClassVars
{
  TargetCameraReprjErrorProjModelAsClassVars(double ob_x, double ob_y, double fx, double fy, double cx, double cy)
    : ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1,  /** extrinsic parameters */
                  const T* const c_p2,  /** 6Dof transform of target points into world frame */
                  const T* const point, /** point described in target frame that is being seen */
                  T* resid) const
  {
    /** extract the variables from parameter blocks  */
    int q = 0;               /** extract extrinsic block of parameters */
    const T& ax = c_p1[q++]; /**  angle_axis x for rotation of camera		 */
    const T& ay = c_p1[q++]; /**  angle_axis y for rotation of camera */
    const T& az = c_p1[q++]; /**  angle_axis z for rotation of camera */
    const T& tx = c_p1[q++]; /**  translation of camera x */
    const T& ty = c_p1[q++]; /**  translation of camera y */
    const T& tz = c_p1[q++]; /**  translation of camera z */

    q = 0;                          /** extract target pose block of parameters */
    const T& target_x = c_p2[q++];  /**  target's x location */
    const T& target_y = c_p2[q++];  /**  target's y location */
    const T& target_z = c_p2[q++];  /**  target's z location */
    const T& target_ax = c_p2[q++]; /**  target's ax angle axis value */
    const T& target_ay = c_p2[q++]; /**  target's ay angle axis value */
    const T& target_az = c_p2[q++]; /**  target's az angle axis value */

    /** rotate and translate points into world frame */
    // create a vector from the location of the point in the target's frame
    T point[3];
    point[0] = pnt_x_;
    point[1] = pnt_y_;
    point[2] = pnt_z_;

    /** rotate and translate points into world frame */
    T target_aa[3];       /** angle axis  */
    T world_point_loc[3]; /** point rotated */
    target_aa[0] = target_ax;
    target_aa[1] = target_ay;
    target_aa[2] = target_az;
    ceres::AngleAxisRotatePoint(target_aa, point, world_point_loc);

    /** apply target translation */
    world_point_loc[0] = world_point_loc[0] + target_x;
    world_point_loc[1] = world_point_loc[1] + target_y;
    world_point_loc[2] = world_point_loc[2] + target_z;

    /** rotate and translate points into camera frame */
    /*  Note that camera transform is from world into camera frame, not vise versa */
    T aa[3];               /** angle axis  */
    T camera_point_loc[3]; /** point rotated */
    aa[0] = ax;
    aa[1] = ay;
    aa[2] = az;
    ceres::AngleAxisRotatePoint(aa, point, camera_point_loc);

    /** apply camera translation */
    T xp1 = camera_point_loc[0] + tx; /** point rotated and translated */
    T yp1 = camera_point_loc[1] + ty;
    T zp1 = camera_point_loc[2] + tz;

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
  static ceres::CostFunction* Create(const double o_x, const double o_y, double fx, double fy, double cx, double cy)
  {
    return (new ceres::AutoDiffCostFunction<TargetCameraReprjErrorProjModelAsClassVars, 2, 6, 6, 3>(
        new CameraReprjError(o_x, o_y, fx, fy, cx, cy)));
  }
  double ox_;    /** observed x location of object in image */
  double oy_;    /** observed y location of object in image */
  double fx_;    /** focal length in x pixels */
  double fy_;    /** focal length in x pixels */
  double cx_;    /** optical center x */
  double cy_;    /** optical center y */
  double pnt_x_; /*!< known location of point in target's reference frame x */
  double pnt_y_; /*!< known location of point in target's reference frame y */
  double pnt_z_; /*!< known location of point in target's reference frame z */
};

void printQTasH(double qx, double qy, double qz, double qw, double tx, double ty, double tz)
{
  double Rs11 = qw * qw + qx * qx - qy * qy - qz * qz;
  double Rs21 = 2.0 * qx * qy + 2.0 * qw * qz;
  double Rs31 = 2.0 * qx * qz - 2.0 * qw * qy;

  double Rs12 = 2.0 * qx * qy - 2.0 * qw * qz;
  double Rs22 = qw * qw - qx * qx + qy * qy - qz * qz;
  double Rs32 = 2.0 * qy * qz + 2.0 * qw * qx;

  double Rs13 = 2.0 * qx * qz + 2.0 * qw * qy;
  double Rs23 = 2.0 * qy * qz - 2.0 * qw * qx;
  double Rs33 = qw * qw - qx * qx - qy * qy + qz * qz;

  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", Rs11, Rs12, Rs13, tx);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", Rs21, Rs22, Rs23, ty);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", Rs31, Rs32, Rs33, tz);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", 0.0, 0.0, 0.0, 1.0);
}

void printAATasH(double x, double y, double z, double tx, double ty, double tz)
{
  double R[9];
  double aa[3];
  aa[0] = x;
  aa[1] = y;
  aa[2] = z;
  ceres::AngleAxisToRotationMatrix(aa, R);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[0], R[3], R[6], tx);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[1], R[4], R[7], ty);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[2], R[5], R[8], tz);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", 0.0, 0.0, 0.0, 1.0);
}

void printAATasHI(double x, double y, double z, double tx, double ty, double tz)
{
  double R[9];
  double aa[3];
  aa[0] = x;
  aa[1] = y;
  aa[2] = z;
  ceres::AngleAxisToRotationMatrix(aa, R);
  double ix = -(tx * R[0] + ty * R[1] + tz * R[2]);
  double iy = -(tx * R[3] + ty * R[4] + tz * R[5]);
  double iz = -(tx * R[6] + ty * R[7] + tz * R[8]);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[0], R[1], R[2], ix);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[3], R[4], R[5], iy);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[6], R[7], R[8], iz);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", 0.0, 0.0, 0.0, 1.0);
}

void printAAasEuler(double x, double y, double z)
{
  double R[9];
  double aa[3];
  aa[0] = x;
  aa[1] = y;
  aa[2] = z;
  ceres::AngleAxisToRotationMatrix(aa, R);
  double rx = atan2(R[7], R[8]);
  double ry = atan2(-R[6], sqrt(R[7] * R[7] + R[8] * R[8]));
  double rz = atan2(R[3], R[0]);
  printf("rpy = %8.4f %8.4f %8.4f\n", rx, ry, rz);
}

void printCameraParameters(CameraParameters C, std::string words)
{
  printf("%s\n", words.c_str());
  printf("Camera to World Transform:\n");
  printAATasHI(C.aa[0], C.aa[1], C.aa[2], C.pos[0], C.pos[1], C.pos[2]);

  printf("World to Camera\n");
  printAATasH(C.aa[0], C.aa[1], C.aa[2], C.pos[0], C.pos[1], C.pos[2]);
  printAAasEuler(C.aa[0], C.aa[1], C.aa[2]);
  printf("fx = %8.3lf fy = %8.3lf\n", C.fx, C.fy);
  printf("k1 = %8.3lf k2 = %8.3lf k3 = %8.3lf\n", C.k1, C.k2, C.k3);
  printf("p1 = %8.3lf p2 = %8.3lf\n", C.p1, C.p2);
  printf("cx = %8.3lf cy = %8.3lf\n", C.cx, C.cy);
}
}  // end of namespace
