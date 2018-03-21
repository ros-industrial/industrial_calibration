#include <stdio.h>
#include <vector>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <iostream>
typedef struct
{
  int p_id;  // point's id
  double x;  // image x
  double y;  // image y
} observation;

typedef struct
{
  union
  {
    struct
    {
      double x;
      double y;
      double z;
    };
    double PB[3];  // parameter block
  };
} point;

typedef struct
{
  union
  {
    struct
    {
      double PB_extrinsics[6];  // parameter block for intrinsics
      double PB_intrinsics[9];  // parameter block for extrinsics
    };
    struct
    {
      double aa[3];   // angle axis data
      double pos[3];  // position data
      double fx;
      double fy;
      double cx;
      double cy;
      double k1;
      double k2;
      double k3;
      double p1;
      double p2;
    };
  };
} Camera;

// local prototypes
void print_QTasH(double qx, double qy, double qz, double qw, double tx, double ty, double tz);
void print_AATasH(double x, double y, double z, double tx, double ty, double tz);
void print_AATasHI(double x, double y, double z, double tx, double ty, double tz);
void print_AAasEuler(double x, double y, double z);
void print_camera(Camera C, std::string words);
observation project_point(Camera C, point P);

// computes image of point in cameras image plane
observation project_point(Camera C, point P)
{
  double p[3];
  double pt[3];
  pt[0] = P.x;
  pt[1] = P.y;
  pt[2] = P.z;

  // transform point into camera frame
  // note, camera transform takes points from camera frame into world frame

  ceres::AngleAxisRotatePoint(C.aa, pt, p);
  printf("point %6.3lf %6.3lf %6.3lf rotated: %6.3lf %6.3lf %6.3lf ", P.x, P.y, P.z, p[0], p[1], p[2]);
  p[0] += C.pos[0];
  p[1] += C.pos[1];
  p[2] += C.pos[2];
  printf("translated: %6.3lf %6.3lf %6.3lf\n", p[0], p[1], p[2]);
  //  printf("PPP %6.3lf  %6.3lf %6.3lf \n",p[0],p[1],p[2]);
  double xp = p[0] / p[2];
  double yp = p[1] / p[2];

  //  printf("xp yp %6.3lf  %6.3lf\n",xp,yp);
  // calculate terms for polynomial distortion
  double r2 = xp * xp + yp * yp;
  double r4 = r2 * r2;
  double r6 = r2 * r4;

  double xp2 = xp * xp;  // temporary variables square of others
  double yp2 = yp * yp;

  // apply the distortion coefficients to refine pixel location
  double xpp = xp + C.k1 * r2 * xp + C.k2 * r4 * xp + C.k3 * r6 * xp + C.p2 * (r2 + 2 * xp2) + 2 * C.p1 * xp * yp;
  double ypp = yp + C.k1 * r2 * yp + C.k2 * r4 * yp + C.k3 * r6 * yp + C.p1 * (r2 + 2 * yp2) + 2 * C.p2 * xp * yp;

  // perform projection using focal length and camera center into image plane
  observation O;
  O.p_id = 0;
  O.x = C.fx * xpp + C.cx;
  O.y = C.fy * ypp + C.cy;
  return (O);
}

struct Camera_reprj_error
{
  Camera_reprj_error(double ob_x, double ob_y) : Ox(ob_x), Oy(ob_y)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1,  // extrinsic parameters
                  const T* c_p2,        // intrinsic parameters
                  const T* point,       // point being projected, yes this is has 3 parameters
                  T* resid) const
  {
    // extract the variables from the camera parameters
    int q = 0;                // extrinsic block of parameters
    const T& x = c_p1[q++];   //  angle_axis x for rotation of camera
    const T& y = c_p1[q++];   //  angle_axis y for rotation of camera
    const T& z = c_p1[q++];   //  angle_axis z for rotation of camera
    const T& tx = c_p1[q++];  //  translation of camera x
    const T& ty = c_p1[q++];  //  translation of camera y
    const T& tz = c_p1[q++];  //  translation of camera z

    q = 0;                    // intrinsic block of parameters
    const T& fx = c_p2[q++];  //  focal length x
    const T& fy = c_p2[q++];  //  focal length x
    const T& cx = c_p2[q++];  //  center point x
    const T& cy = c_p2[q++];  //  center point y
    const T& k1 = c_p2[q++];  //  distortion coefficient on 2nd order terms
    const T& k2 = c_p2[q++];  //  distortion coefficient on 4th order terms
    const T& k3 = c_p2[q++];  //  distortion coefficient on 6th order terms
    const T& p1 = c_p2[q++];  //  tangential distortion coefficient x
    const T& p2 = c_p2[q++];  //  tangential distortion coefficient y

    // rotate and translate points into camera frame
    T aa[3];  // angle axis
    T p[3];   // point rotated
    aa[0] = x;
    aa[1] = y;
    aa[2] = z;
    ceres::AngleAxisRotatePoint(aa, point, p);

    // apply camera translation
    T xp1 = p[0] + tx;  // point rotated and translated
    T yp1 = p[1] + ty;
    T zp1 = p[2] + tz;

    // scale into the image plane by distance away from camera
    T xp = xp1 / zp1;
    T yp = yp1 / zp1;

    // calculate terms for polynomial distortion
    T r2 = xp * xp + yp * yp;
    T r4 = r2 * r2;
    T r6 = r2 * r4;

    T xp2 = xp * xp;  // temporary variables square of others
    T yp2 = yp * yp;
    // apply the distortion coefficients to refine pixel location
    T xpp = xp + k1 * r2 * xp + k2 * r4 * xp + k3 * r6 * xp + p2 * (r2 + T(2.0) * xp2) + T(2.0) * p1 * xp * yp;
    T ypp = yp + k1 * r2 * yp + k2 * r4 * yp + k3 * r6 * yp + p1 * (r2 + T(2.0) * yp2) + T(2.0) * p2 * xp * yp;
    // perform projection using focal length and camera center into image plane
    resid[0] = fx * xpp + cx - Ox;
    resid[1] = fy * ypp + cy - Oy;

    return true;
  }  // end of operator()

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double o_x, const double o_y)
  {
    return (new ceres::AutoDiffCostFunction<Camera_reprj_error, 2, 6, 9, 3>(new Camera_reprj_error(o_x, o_y)));
  }
  double Ox;  // observed x location of object in image
  double Oy;  // observed y location of object in image
};

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  if (argc != 5)
  {
    std::cerr << "usage: monoExCal <3Dpoints_file> <observation_file> <intrinsic_file> <extrinsic_file>\n";
    return 1;
  }
  int num_points;
  int num_observations;
  observation o;

  // this code peforms extrinsic calibration on a monocular camera
  // it assumes that 3D data from a positioning device is available
  // this 3D data could come from an IGPS, a Fero arm, or any robot
  // each 3D point should be observed by the camera, and the image(x,y) position
  // of that observation must be known.
  // It is assumed that the intrinsic calibration is already known
  // The input is provided by 4 files
  // 1. 3D points stored as ascii in the form:
  //   num_points      # read as integer
  //   x[0] y[0] z[0]  # read as double
  //   ...
  //   x[num_points-1] y[num_points-1] z[num_points-1]
  // 2. Observations stored as ascii in the form
  //   num_observations  # read as integer
  //   x[0] y[0]         # read as double
  //   ...
  //   x[num_observations-1] y[num_observations-1]
  // 3. Camera intrisic data stored as ascii in the ROS.ini format
  // 4. Camera initial extrinsic file stored as ascii indicating the homogeneous transform
  //  nx ox ax tx  #read all as double
  //  ny oy ay ty
  //  nz oz az tz
  //  0  0  0  1.0

  // read in the problem
  FILE* points_fp = fopen(argv[1], "r");
  FILE* observations_fp = fopen(argv[2], "r");
  FILE* intrinsics_fp = fopen(argv[3], "r");
  FILE* extrinsics_fp = fopen(argv[4], "r");
  if (points_fp == NULL)
  {
    printf("Could not open file: %s", argv[1]);
    exit(1);
  }
  if (observations_fp == NULL)
  {
    printf("Could not open file: %s", argv[2]);
    exit(1);
  }
  if (intrinsics_fp == NULL)
  {
    printf("Could not open file: %s", argv[3]);
    exit(1);
  }
  if (extrinsics_fp == NULL)
  {
    printf("Could not open file: %s", argv[4]);
    exit(1);
  }

  // first read points file
  if (fscanf(points_fp, "%d", &num_points) != 1)
  {
    printf("couldn't read num_points\n");
    exit(1);
  }
  std::vector<point> Pts;
  for (int i = 0; i < num_points; i++)
  {
    point p;
    if (fscanf(points_fp, "%lf %lf %lf", &p.x, &p.y, &p.z) != 3)
    {
      printf("couldn't read point %d from %s\n", i, argv[1]);
      exit(1);
    }
    Pts.push_back(p);
  }
  fclose(points_fp);

  // Then read in the observations
  if (fscanf(observations_fp, "%d", &num_observations) != 1)
  {
    printf("couldn't read num_observations\n");
    exit(1);
  }
  if (num_observations != num_points)
  {
    printf("WARNING, num_points NOT EQUAL to num_observations\n");
  }
  std::vector<observation> Ob;
  for (int i = 0; i < num_observations; i++)
  {
    if (fscanf(observations_fp, "%lf %lf", &o.x, &o.y) != 2)
    {
      printf("couldn't read observation %d from %s\n", i, argv[2]);
      exit(1);
    }
    o.p_id = i;
    Ob.push_back(o);
  }
  fclose(observations_fp);

  // read camera intrinsics
  Camera C;
  char dum[255];
  double Dum, Dum2, Dum3;
  int image_width;
  int image_height;
  int rtn = 0;
  rtn += fscanf(intrinsics_fp, "%s", dum);  // should be "#"
  rtn += fscanf(intrinsics_fp, "%s", dum);  // should be "Camera"
  rtn += fscanf(intrinsics_fp, "%s", dum);  // should be "intrinsics"
  rtn += fscanf(intrinsics_fp, "%s", dum);  // should be "[image]"
  //  printf("should be [image]: %s\n",dum);
  rtn += fscanf(intrinsics_fp, "%s", dum);  // should be "width"
  //  printf("should be width: %s\n",dum);
  rtn += fscanf(intrinsics_fp, "%d", &image_width);  // should be the image width of provided by camera
  printf("image_width: %d\n", image_width);
  rtn += fscanf(intrinsics_fp, "%s", dum);            // should be "height"
  rtn += fscanf(intrinsics_fp, "%d", &image_height);  // should be the image width of provided by camera
  printf("height: %d\n", image_height);
  rtn += fscanf(intrinsics_fp, "%s", dum);  // should be "[some name]"
  //  printf("[some name]: %s\n",dum);
  rtn += fscanf(intrinsics_fp, "%s", dum);  // should be "camera"
  //  printf("should be camera: %s\n",dum);
  rtn += fscanf(intrinsics_fp, "%s", dum);  // should be "matrix"
  //  printf("should be matrix: %s\n",dum);
  rtn += fscanf(intrinsics_fp, "%lf %lf %lf", &C.fx, &Dum, &C.cx);
  rtn += fscanf(intrinsics_fp, "%lf %lf %lf", &Dum, &C.fy, &C.cy);
  rtn += fscanf(intrinsics_fp, "%lf %lf %lf", &Dum, &Dum2, &Dum3);
  rtn += fscanf(intrinsics_fp, "%s", dum);  // should be "distortion"
  printf("camera matrix:\n");
  printf("%8.3lf %8.3lf %8.3lf\n", C.fx, 0.0, C.cx);
  printf("%8.3lf %8.3lf %8.3lf\n", 0.0, C.fy, C.cy);
  printf("%8.3lf %8.3lf %8.3lf\n", 0.0, 0.0, 0.0);
  //  printf("should be distortion: %s\n",dum);
  rtn += fscanf(intrinsics_fp, "%lf %lf %lf %lf %lf", &C.k1, &C.k2, &C.k3, &C.p1, &C.p2);
  printf("Distortion: [ %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf ]\n", C.k1, C.k2, C.k3, C.p1, C.p2);
  fclose(intrinsics_fp);

  // read camera extrinsics
  double H[4][4];
  rtn = 0;
  rtn += fscanf(extrinsics_fp, "%lf %lf %lf %lf", &H[0][0], &H[0][1], &H[0][2], &H[0][3]);
  rtn += fscanf(extrinsics_fp, "%lf %lf %lf %lf", &H[1][0], &H[1][1], &H[1][2], &H[1][3]);
  rtn += fscanf(extrinsics_fp, "%lf %lf %lf %lf", &H[2][0], &H[2][1], &H[2][2], &H[2][3]);
  rtn += fscanf(extrinsics_fp, "%lf %lf %lf %lf", &H[3][0], &H[3][1], &H[3][2], &H[3][3]);
  if (rtn != 16)
  {
    printf("could not read extrinsics rtn=%d from %s\n", rtn, argv[4]);
    exit(1);
  }
  fclose(extrinsics_fp);

  // use the inverse of transform from camera to world as camera transform
  double HI[9];     // note ceres uses column major order
  HI[0] = H[0][0];  // first column becomes first row
  HI[1] = H[0][1];
  HI[2] = H[0][2];

  HI[3] = H[1][0];  // second column becomes second row
  HI[4] = H[1][1];
  HI[5] = H[1][2];

  HI[6] = H[2][0];  // third column becomes third row
  HI[7] = H[2][1];
  HI[8] = H[2][2];
  C.pos[0] = -(H[0][3] * H[0][0] + H[1][3] * H[1][0] + H[2][3] * H[2][0]);
  C.pos[1] = -(H[0][3] * H[0][1] + H[1][3] * H[1][1] + H[2][3] * H[2][1]);
  C.pos[2] = -(H[0][3] * H[0][2] + H[1][3] * H[1][2] + H[2][3] * H[2][2]);
  printf("C.xyz = %lf %lf %lf\n", C.pos[0], C.pos[1], C.pos[2]);
  ceres::RotationMatrixToAngleAxis(HI, C.aa);

/* used to create sample data with known solution
 FILE *fp6 = fopen("new_observations.txt","w");
 fprintf(fp6,"%d\n",num_points);
 for(int i=0;i<num_points;i++){
 o = project_point(C,Pts[i]);
 fprintf(fp6,"%lf %lf\n",o.x,o.y);
 }
 fclose(fp6);
 exit(1);
 */
#define MONO_EXCAL_DEBUG
#ifdef MONO_EXCAL_DEBUG
  /* Print initial errors */
  /* Save projections and observations to matlab compatible form    */
  FILE* fp_temp1 = fopen("Obs.m", "w");
  FILE* fp_temp2 = fopen("Rep.m", "w");
  FILE* fp_temp3 = fopen("FRep.m", "w");
  fprintf(fp_temp1, "O = [ ");
  fprintf(fp_temp2, "R = [ ");
  fprintf(fp_temp3, "F = [ ");
  for (int i = 0; i < num_points; i++)
  {
    o = project_point(C, Pts[i]);
    printf("Errors %d  = %lf %lf\n", i, Ob[i].x - o.x, Ob[i].y - o.y);
    fprintf(fp_temp1, "%lf %lf;\n", Ob[i].x, Ob[i].y);
    fprintf(fp_temp2, "%lf %lf;\n", o.x, o.y);
  }
  fprintf(fp_temp1, "];\n");
  fprintf(fp_temp2, "];\n");
  fclose(fp_temp1);
  fclose(fp_temp2);
#endif  // MONO_EXCAL_DEBUG
  print_camera(C, "Original Parameters");
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  for (int i = 0; i < num_observations; ++i)
  {
    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.
    ceres::CostFunction* cost_function = Camera_reprj_error::Create(Ob[i].x, Ob[i].y);

    problem.AddResidualBlock(cost_function, NULL, C.PB_extrinsics, C.PB_intrinsics, Pts[i].PB);
    problem.SetParameterBlockConstant(C.PB_intrinsics);
    problem.SetParameterBlockConstant(Pts[i].PB);
    /* DEBUG the reprojection error, this shows how to call reprojection error directly
     Camera_reprj_error CE(Ob[i].x,Ob[i].y);
     double res[2];
     CE(C.PB_extrinsics,C.PB_intrinsics,Pts[i].PB,res);
     printf("residual %d = %9.3lf %9.3lf\n",i,res[0],res[1]);
     */
  }

  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 1000;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

#ifdef MONO_EXCAL_DEBUG
  /* Print final errors */
  for (int i = 0; i < num_points; i++)
  {
    o = project_point(C, Pts[i]);
    printf("%d : Ob= %6.3lf %6.3lf ", i, Ob[i].x, Ob[i].y);
    printf("%d : o= %6.3lf %6.3lf Errors = %10.3lf %10.3lf\n", i, o.x, o.y, Ob[i].x - o.x, Ob[i].y - o.y);
    fprintf(fp_temp3, "%lf %lf;\n", o.x, o.y);
  }
  fprintf(fp_temp3, "];\n");
  fclose(fp_temp3);
#endif  // MONO_EXCAL_DEBUG
  // Print final camera parameters
  print_camera(C, "final parameters");

  // write new extrinsics to a file
  std::string temp(argv[4]);
  std::string new_ex_file = "new_" + temp;
  extrinsics_fp = fopen(new_ex_file.c_str(), "w");
  ceres::AngleAxisToRotationMatrix(C.aa, HI);  // Column Major

  // invert HI to get H
  H[0][0] = HI[0];  // first column of HI is set to first row of H
  H[0][1] = HI[1];
  H[0][2] = HI[2];

  H[1][0] = HI[3];  // second column of HI is set to second row of H
  H[1][1] = HI[4];
  H[1][2] = HI[5];

  H[2][0] = HI[6];  // third column of HI is set to third row of H
  H[2][1] = HI[7];
  H[2][2] = HI[8];

  H[0][3] = -(C.pos[0] * HI[0] + C.pos[1] * HI[1] + C.pos[2] * HI[2]);
  H[1][3] = -(C.pos[0] * HI[3] + C.pos[1] * HI[4] + C.pos[2] * HI[5]);
  H[2][3] = -(C.pos[0] * HI[6] + C.pos[1] * HI[7] + C.pos[2] * HI[8]);

  fprintf(extrinsics_fp, "%9.3lf %9.3lf %9.3lf %9.3lf\n", H[0][0], H[0][1], H[0][2], H[0][3]);
  fprintf(extrinsics_fp, "%9.3lf %9.3lf %9.3lf %9.3lf\n", H[1][0], H[1][1], H[1][2], H[1][3]);
  fprintf(extrinsics_fp, "%9.3lf %9.3lf %9.3lf %9.3lf\n", H[2][0], H[2][1], H[2][2], H[2][3]);
  fprintf(extrinsics_fp, "%9.3lf %9.3lf %9.3lf %9.3lf\n", 0.0, 0.0, 0.0, 1.0);
  fclose(extrinsics_fp);

  return 0;
}

// print a quaternion plus position as a homogeneous transform
void print_QTasH(double qx, double qy, double qz, double qw, double tx, double ty, double tz)
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

// angle axis to homogeneous transform inverted
void print_AATasH(double x, double y, double z, double tx, double ty, double tz)
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

// angle axis to homogeneous transform
void print_AATasHI(double x, double y, double z, double tx, double ty, double tz)
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

void print_AAasEuler(double x, double y, double z)
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
void print_camera(Camera C, std::string words)
{
  printf("%s\n", words.c_str());
  printf("Camera to World Transform:\n");
  print_AATasHI(C.aa[0], C.aa[1], C.aa[2], C.pos[0], C.pos[1], C.pos[2]);

  printf("World to Camera\n");
  print_AATasH(C.aa[0], C.aa[1], C.aa[2], C.pos[0], C.pos[1], C.pos[2]);
  print_AAasEuler(C.aa[0], C.aa[1], C.aa[2]);
  printf("fx = %8.3lf fy = %8.3lf\n", C.fx, C.fy);
  printf("k1 = %8.3lf k2 = %8.3lf k3 = %8.3lf\n", C.k1, C.k2, C.k3);
  printf("p1 = %8.3lf p2 = %8.3lf\n", C.p1, C.p2);
  printf("cx = %8.3lf cy = %8.3lf\n", C.cx, C.cy);
}
