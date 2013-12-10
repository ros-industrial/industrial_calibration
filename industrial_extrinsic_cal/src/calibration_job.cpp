#include <industrial_extrinsic_cal/calibration_job.hpp>
#include <boost/foreach.hpp>


bool CalibrationJob::run()
{
  run_observations();
  run_optimization();
}

bool CalibrationJob::run_observations()
{
  BOOST_FOREACH(ObservationCommand);
}
bool CalibrationJob::run_opimization()
{
  // take all the data collected and create a Ceres optimization problem and run it

  // the following is an example taken from some working code. Clearly it won't work with
  // our data structures, but I included it here as an example

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  BOOST_FOREACH(ObservationList OL_, ol);

    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.
    ceres::CostFunction* cost_function =  Camera_reprj_error::Create(Ob[i].x,Ob[i].y);

    problem.AddResidualBlock(cost_function, NULL , 
			     C.PB_extrinsics, 
			     C.PB_intrinsics, 
			     Pts[i].PB);
    problem.SetParameterBlockConstant(C.PB_intrinsics);
    problem.SetParameterBlockConstant(Pts[i].PB);
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

}
