#include <ros/ros.h>

#include <string>
#include <fstream>

// ros includes
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <std_srvs/Trigger.h>


// ros actionlib includes
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

// intrinsic_cal includes
#include <intrinsic_cal/ical_srv_solve.h>

// industrial_extrinsic_cal includes
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/camera_observer.hpp>
#include <industrial_extrinsic_cal/camera_definition.h>
#include <industrial_extrinsic_cal/observation_scene.h>
#include <industrial_extrinsic_cal/observation_data_point.h>
#include <industrial_extrinsic_cal/ceres_blocks.h>
#include <industrial_extrinsic_cal/ros_camera_observer.h>
#include <industrial_extrinsic_cal/ceres_costs_utils.hpp>
#include <industrial_extrinsic_cal/ceres_costs_utils.h>
#include <industrial_extrinsic_cal/circle_cost_utils.hpp>
#include <industrial_extrinsic_cal/calibration_job_definition.h>
#include <industrial_extrinsic_cal/calibrationAction.h>
#include <industrial_extrinsic_cal/calibrate.h>
#include <industrial_extrinsic_cal/covariance.h>
#include <industrial_extrinsic_cal/ros_target_display.hpp>
#include <industrial_extrinsic_cal/camera_yaml_parser.h>
#include <industrial_extrinsic_cal/targets_yaml_parser.h>


// boost includes
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

using boost::shared_ptr;
using ceres::Solver;
using ceres::CostFunction;
using industrial_extrinsic_cal::Camera;
using industrial_extrinsic_cal::Target;
using industrial_extrinsic_cal::CeresBlocks;
using industrial_extrinsic_cal::Roi;
using industrial_extrinsic_cal::CameraObservations;
using industrial_extrinsic_cal::Point3d;
using industrial_extrinsic_cal::Pose6d;
using industrial_extrinsic_cal::P_BLOCK;

class icalServiceNode
{
public:
  icalServiceNode(const  ros::NodeHandle& nh)
    :nh_(nh), P_(NULL), problem_initialized_(false), total_observations_(0), scene_(0)
  {
    std::string nn = ros::this_node::getName();
    ros::NodeHandle priv_nh("~");

    // load cameras and targets
    priv_nh.getParam("yaml_file_path", yaml_file_path_);
    std::string camera_file, target_file;
    priv_nh.getParam("camera_file", camera_file);
    priv_nh.getParam("target_file", target_file);
    camera_file_ = yaml_file_path_ + camera_file ;
    target_file_ = yaml_file_path_ + target_file ;
    ROS_INFO("yaml_file_path: %s", yaml_file_path_.c_str());
    ROS_INFO("camera_file: %s",  camera_file_.c_str());
    ROS_INFO("target_file: %s",  target_file_.c_str());
    if(!load_camera()){
      ROS_ERROR("can't load the camera from %s", (yaml_file_path_+camera_file_).c_str());
      exit(1);
    }
    if(!load_target()){
      ROS_ERROR("can't load the target from %s", (yaml_file_path_+target_file_).c_str());
      exit(1);
    }

    // load cameras, targets and intiialize ceres blocks
    load_camera();
    load_target();
    init_blocks();

    // advertise services
    start_server_       = nh_.advertiseService( "IcalSrvStart", &icalServiceNode::startCallBack, this);
    observation_server_ = nh_.advertiseService( "IcalSrvObs", &icalServiceNode::observationCallBack, this);
    run_server_         = nh_.advertiseService( "IcalSrvRun", &icalServiceNode::runCallBack, this);
    save_server_        = nh_.advertiseService( "IcalSrvSave", &icalServiceNode::saveCallBack, this);
  };// end of constructor

  void init_blocks()
  {
    // adds the intial block for each camera and target
    for (int i = 0; i < (int)all_cameras_.size(); i++)
      {
	if (all_cameras_[i]->is_moving_)
	  {
	    int scene_ = 0;
	    ceres_blocks_.addMovingCamera(all_cameras_[i], scene_);
	  }
	else
	  {
	    ceres_blocks_.addStaticCamera(all_cameras_[i]);
	  }
      }


    for (int i = 0; i < (int)all_targets_.size(); i++)
      {
	if (all_targets_[i]->pub_rviz_vis_){ // use rviz visualization marker to display the target, currently must be modified circle grid
	  displayRvizTarget(all_targets_[i]);
	}
	if (all_targets_[i]->is_moving_)
	  {
	    int scene_ = 0;
	    ceres_blocks_.addMovingTarget(all_targets_[i], scene_);
	  }
	else
	  {
	    ceres_blocks_.addStaticTarget(all_targets_[i]);
	  }
      }  // end for every target found


    // if loaded camera is the right side of a stereo pair, it contains a pointer to the left one.
    for (int i = 0; i < (int)all_cameras_.size(); i++)
      {
	if(all_cameras_[i]->is_right_stereo_camera_)
	  {
	    all_cameras_[i]->left_stereo_camera_ = ceres_blocks_.getCameraByName(all_cameras_[i]->left_stereo_camera_name_);
	  }
      }

    // set reference frame for all transform interfaces
    ceres_blocks_.setReferenceFrame(all_cameras_[0]->transform_interface_->getTransformFrame());
  }; // end init_blocks()

  // read the camera yaml file
  bool load_camera() 
  {
    bool rtn = true;
    if (!parseCameras(camera_file_, all_cameras_))
      {
	ROS_ERROR("failed to parse cameras from %s", camera_file_.c_str());
	rtn = false;
      }
    return rtn;
  };// end of load_camera()

  // read the target yaml file
  bool load_target()
  {
    bool rtn = true;

    if (!parseTargets(target_file_, all_targets_))
      {
	ROS_ERROR("failed to parse targets from %s", target_file_.c_str());
	rtn = false;
      }
    return rtn;
  }; // end load_target()

  // callback functions
  bool startCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    if(problem_initialized_)	delete(P_);
    P_ = new ceres::Problem;
    problem_initialized_ = true;
    total_observations_  = 0;
    scene_=0;
    ceres_blocks_.clearCamerasTargets(); 
    init_blocks();
    res.message = std::string("Intrinsic calibration service started");
    res.success = true;
    return(true);
  }

  // called to collect observations for the current pose of the scene
  bool observationCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    char msg[100];
    
    if(problem_initialized_ != true ){
      ROS_ERROR("must call start service");
      res.success = false;
      sprintf(msg, "must call start service");
      res.message = std::string(msg);
      return(true);
    }

    for(int i=0; i<all_targets_.size(); i++){
      all_targets_[i]->pullTransform();
    }
    
    for(int i=0; i<all_cameras_.size(); i++){
      // set the roi to the whole image
      Roi roi;
      roi.x_min = 0;
      roi.y_min = 0;
      roi.x_max = all_cameras_[i]->camera_parameters_.width;
      roi.y_max = all_cameras_[i]->camera_parameters_.height;

      // get observations
      all_cameras_[i]->camera_observer_->clearTargets();
      all_cameras_[i]->camera_observer_->clearObservations();
      industrial_extrinsic_cal::Cost_function cost_type = industrial_extrinsic_cal::cost_functions::CameraReprjErrorWithDistortionPK;
      int total_pts=0;
      for(int j=0;j<all_targets_.size();j++){ // add all targets to the camera
	all_cameras_[i]->camera_observer_->addTarget(all_targets_[j], roi, cost_type);
	total_pts += all_targets_[j]->num_points_;
      }
      all_cameras_[i]->camera_observer_->triggerCamera();
      while (!all_cameras_[i]->camera_observer_->observationsDone());
      CameraObservations camera_observations;
      all_cameras_[i]->camera_observer_->getObservations(camera_observations);
      int num_observations = (int)camera_observations.size();
      ROS_INFO("Found %d observations", (int)camera_observations.size());
	
      // add observations to problem
      num_observations = (int)camera_observations.size();
      if (num_observations != total_pts)
	{
	  ROS_ERROR("Target Locator could not find all targets found %d out of %d", num_observations, total_pts);
	}
      else
	{	 // add a new cost to the problem for each observation
	  CostFunction* cost_function[num_observations];  
	  total_observations_ += num_observations;
	  ceres_blocks_.addMovingTarget(camera_observations[0].target, scene_);
	  for (int k = 0; k < num_observations; k++)
	    {
	      shared_ptr<Target> target = camera_observations[k].target;
	      double image_x = camera_observations[k].image_loc_x;
	      double image_y = camera_observations[k].image_loc_y;
	      Point3d point  = target->pts_[k];
	      P_BLOCK intrinsics = ceres_blocks_.getStaticCameraParameterBlockIntrinsics(all_cameras_[i]->camera_name_);
	      P_BLOCK target_pb  = ceres_blocks_.getMovingTargetPoseParameterBlock(target->target_name_,scene_);
	      if(k==0){
		ROS_ERROR("target_pb = %ld intrinsics = %ld",(long int * ) &(target_pb[0]), (long int *) &intrinsics[0]);
		Pose6d P(target_pb[3],target_pb[4],target_pb[5],target_pb[0],target_pb[1],target_pb[2]);
		P.show("Pose of Target");
	      }
	      cost_function[k] = industrial_extrinsic_cal::CameraReprjErrorWithDistortionPK::Create(image_x, image_y, point);
	      //	      cost_function[k] = industrial_extrinsic_cal::CircleCameraReprjErrorWithDistortionPK::Create(image_x, image_y, target->circle_grid_parameters_.circle_diameter, point);
	      P_->AddResidualBlock(cost_function[k], NULL, target_pb, intrinsics);
	    }  // for each observation at this camera_location
	} // end of else (there are some observations to add)
    }// for each camera

    ROS_INFO("now have %d observations after scene %d",total_observations_, scene_);
    scene_++;

    sprintf(msg, "Ical_srv now has %d observations after scene %d",total_observations_, scene_);
    res.message = std::string(msg);
    res.success = true;
    return(true);

  }; // end observation service};

  bool runCallBack( intrinsic_cal::ical_srv_solveRequest &req, intrinsic_cal::ical_srv_solveResponse &res)
  {
    char msg[100];
    ROS_INFO("initial values");
    for(int i=0; i<all_cameras_.size(); i++){
      ROS_INFO("camera_matrix data: [ %lf, 0.0, %lf, 0.0, %lf, %lf, 0.0, 0.0, 1.0]",
	       all_cameras_[i]->camera_parameters_.focal_length_x, all_cameras_[i]->camera_parameters_.center_x,
	       all_cameras_[i]->camera_parameters_.focal_length_y, all_cameras_[i]->camera_parameters_.center_y);
      ROS_INFO("distortion data: [ %lf,  %lf,  %lf,  %lf,  %lf]", all_cameras_[i]->camera_parameters_.distortion_k1,
	       all_cameras_[i]->camera_parameters_.distortion_k2, all_cameras_[i]->camera_parameters_.distortion_p1,
	       all_cameras_[i]->camera_parameters_.distortion_p2, all_cameras_[i]->camera_parameters_.distortion_k3);
    }


    // check for obvious errors
    if(problem_initialized_==false){
      ROS_ERROR("must call start service");
      sprintf(msg, "must call start service to initialized ical service");
      res.message = std::string(msg);
      res.success = false;
      return(true);
    }
    if(total_observations_ == 0){
      ROS_ERROR("must call observations service at least once");
      sprintf(msg, "must call observations service at least once");
      res.message = std::string(msg);
      res.success = false;
      return(true);
    }

    Solver::Options options;
    Solver::Summary summary;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 1000;
    ceres::Solve(options, P_, &summary);
    if (summary.termination_type != ceres::NO_CONVERGENCE)
      {
	double initial_cost = summary.initial_cost / total_observations_;
	double final_cost = summary.final_cost / total_observations_;

	ROS_INFO("Problem solved, initial cost = %lf, final cost = %lf", initial_cost, final_cost);
	for(int i=0; i<all_cameras_.size(); i++){
	  ROS_INFO("camera_matrix data: [ %lf, 0.0, %lf, 0.0, %lf, %lf, 0.0, 0.0, 1.0]",
		   all_cameras_[i]->camera_parameters_.focal_length_x, all_cameras_[i]->camera_parameters_.center_x,
		   all_cameras_[i]->camera_parameters_.focal_length_y, all_cameras_[i]->camera_parameters_.center_y);
	  ROS_INFO("distortion data: [ %lf,  %lf,  %lf,  %lf,  %lf]", all_cameras_[i]->camera_parameters_.distortion_k1,
		   all_cameras_[i]->camera_parameters_.distortion_k2, all_cameras_[i]->camera_parameters_.distortion_p1,
		   all_cameras_[i]->camera_parameters_.distortion_p2, all_cameras_[i]->camera_parameters_.distortion_k3);
	  ROS_INFO("projection_matrix data: [ %lf, 0.0, %lf, 0.0, 0.0, %lf, %lf, 0.0, 0.0, 0.0, 1.0, 0.0]",
		   all_cameras_[i]->camera_parameters_.focal_length_x, all_cameras_[i]->camera_parameters_.center_x,
		   all_cameras_[i]->camera_parameters_.focal_length_y, all_cameras_[i]->camera_parameters_.center_y);
	}
	if (final_cost <= req.allowable_cost_per_observation)
	  {
	    ROS_INFO("calibration was successful");
	  }
	else
	  {
	    res.final_cost_per_observation = final_cost;
	    ROS_ERROR("allowable cost exceeded %f > %f", final_cost, req.allowable_cost_per_observation);
	    sprintf(msg, "allowable cost exceeded %f > %f", final_cost, req.allowable_cost_per_observation);
	    res.message = std::string(msg);
	    res.success = false;
	    return(true);
	  }
	sprintf(msg, "final cost %f", final_cost);
	res.message = std::string(msg);
	res.success = true;
	return(true);
      }
    ROS_ERROR("NO CONVERGENCE");
    sprintf(msg, "NO CONVERGENCE");
    res.message = std::string(msg);
    res.success = false;
    return(true);
  }; // end runCallBack()

  bool saveCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    char msg[100];
    ROS_ERROR("inside saveCallBack()");
    for(int i=0; i<all_cameras_.size(); i++){
      all_cameras_[i]->camera_observer_->pushCameraInfo(
							all_cameras_[i]->camera_parameters_.focal_length_x, all_cameras_[i]->camera_parameters_.focal_length_y,
							all_cameras_[i]->camera_parameters_.center_x, all_cameras_[i]->camera_parameters_.center_y,
							all_cameras_[i]->camera_parameters_.distortion_k1, all_cameras_[i]->camera_parameters_.distortion_k2,
							all_cameras_[i]->camera_parameters_.distortion_k3, all_cameras_[i]->camera_parameters_.distortion_p1,
							all_cameras_[i]->camera_parameters_.distortion_p2);
    }
    sprintf(msg, "intrinsics: %8.2lf %8.2lf %8.2lf %8.2lf, %5.3lf %5.3lf %5.3lf, %5.3lf %5.3lf",
	    all_cameras_[0]->camera_parameters_.focal_length_x, all_cameras_[0]->camera_parameters_.focal_length_y,
	    all_cameras_[0]->camera_parameters_.center_x, all_cameras_[0]->camera_parameters_.center_y,
	    all_cameras_[0]->camera_parameters_.distortion_k1, all_cameras_[0]->camera_parameters_.distortion_k2,
	    all_cameras_[0]->camera_parameters_.distortion_k3, all_cameras_[0]->camera_parameters_.distortion_p1,
	    all_cameras_[0]->camera_parameters_.distortion_p2);

    res.message = std::string(msg);
    res.success = true;
    return(true);

  }// end saveCallback()

private:
  ros::NodeHandle nh_;
  std::vector<shared_ptr<Camera> > all_cameras_;
  std::vector<shared_ptr<Target> > all_targets_;
  std::string yaml_file_path_;
  std::string camera_file_;
  std::string target_file_;
  CeresBlocks ceres_blocks_;                 /*!< This structure maintains the parameter sets for ceres */
  ros::ServiceServer start_server_;
  ros::ServiceServer observation_server_;
  ros::ServiceServer run_server_;
  ros::ServiceServer save_server_;
  ceres::Problem *P_;
  bool problem_initialized_;
  int total_observations_;
  int scene_;
};// end of class icalServiceNode

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ical_cal_service");
  ros::NodeHandle node_handle;
  icalServiceNode ISN(node_handle);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
