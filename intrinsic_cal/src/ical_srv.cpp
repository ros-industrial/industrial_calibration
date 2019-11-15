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
#include <industrial_extrinsic_cal/cal_srv_solve.h>

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
using std::string;

class icalServiceNode
{
public:
  icalServiceNode(const  ros::NodeHandle& nh)
    :nh_(nh), P_(NULL), problem_initialized_(false), total_observations_(0), scene_(0)
  {
    string nn = ros::this_node::getName();
    ros::NodeHandle priv_nh("~");

    // load cameras and targets
    priv_nh.getParam("yaml_file_path", yaml_file_path_);
    string camera_file, target_file;
    priv_nh.getParam("camera_file", camera_file);
    priv_nh.getParam("target_file", target_file);
    priv_nh.getParam("save_data", save_data_);
    camera_file_ = yaml_file_path_ + camera_file ;
    target_file_ = yaml_file_path_ + target_file ;
    ROS_INFO("yaml_file_path: %s", yaml_file_path_.c_str());
    ROS_INFO("camera_file: %s",  camera_file_.c_str());
    ROS_INFO("target_file: %s",  target_file_.c_str());
    if(save_data_) {
      ROS_INFO("saving data");
    }
    else{
      ROS_INFO("NOT saving data");
    }
    if(!load_camera()){
      ROS_ERROR("can't load the camera from %s", (yaml_file_path_+camera_file_).c_str());
      exit(1);
    }

    if(!load_target()){
      ROS_ERROR("can't load the target from %s", (yaml_file_path_+target_file_).c_str());
      exit(1);
    }

    // set all the data directories to the first camera's image_directory_
    for(int i=1; i<all_cameras_.size(); i++){
      all_cameras_[i]->camera_observer_->set_image_directory(all_cameras_[0]->camera_observer_->get_image_directory());
    }
    for(int i=0; i<all_targets_.size(); i++){
      all_targets_[i]->transform_interface_->setDataDirectory(all_cameras_[0]->camera_observer_->get_image_directory());
    }

    // load cameras, targets and intiialize ceres blocks
    init_blocks();

    // advertise services
    start_server_       = nh_.advertiseService( "ICalSrvStart",       &icalServiceNode::startCallBack, this);
    observation_server_ = nh_.advertiseService( "ICalSrvObs",         &icalServiceNode::observationCallBack, this);
    run_server_         = nh_.advertiseService( "ICalSrvRun",         &icalServiceNode::runCallBack, this);
    save_server_        = nh_.advertiseService( "ICalSrvSave",        &icalServiceNode::saveCallBack, this);
    load_server_        = nh_.advertiseService( "ICalSrvLoad",      &icalServiceNode::loadCallBack, this);
    covariance_server_  = nh_.advertiseService( "ICalSrvCov",         &icalServiceNode::covCallBack, this);

  };// end of constructor

  void init_blocks()
  {
    // adds the intial block for each camera and target
    for (int i = 0; i < (int)all_cameras_.size(); i++)
      {
	if (all_cameras_[i]->is_moving_)
	  {
	    scene_ = 0;
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
	    scene_ = 0;
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
    res.message = string("Intrinsic calibration service started");
    res.success = true;
    return(true);
  }

  // called to collect observations for the current pose of the scene
  bool observationCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    char msg[100];
    string null_string("");
    if(problem_initialized_ != true ){
      ROS_INFO("calling start");
      std_srvs::TriggerRequest  sreq;
      std_srvs::TriggerResponse sres;
      startCallBack(sreq,sres);
    }

    for(int i=0; i<all_targets_.size(); i++){
      all_targets_[i]->pullTransform();
      if(save_data_){
	all_targets_[i]->transform_interface_->saveCurrentPose(scene_, null_string);
      }
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

      if(save_data_){
	all_cameras_[i]->camera_observer_->save_current_image(scene_, null_string);
      }

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
      else if(!all_cameras_[i]->camera_observer_->checkObservationProclivity(camera_observations))
        {
          ROS_ERROR("Proclivities Check not successful");
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
		Pose6d P(target_pb[3],target_pb[4],target_pb[5],target_pb[0],target_pb[1],target_pb[2]);
		P.show("Pose of Target");
	      }
	      cost_function[k] = industrial_extrinsic_cal::CameraReprjErrorWithDistortionPK::Create(image_x, image_y, point);
	      if(target_pb == NULL){
		ROS_ERROR("ical_srv attempting to add cost with target_pb = NULL perhaps you don't have a moving target defined");
		continue;
	      }
	      else if(intrinsics == NULL) {
		ROS_ERROR("ical_srv attmpting to add cost with intrinsics = NULL, perhaps you don't have a static camera defined");
		continue;
	      }
	      P_->AddResidualBlock(cost_function[k], NULL, target_pb, intrinsics);
	    }  // for each observation at this camera_location
	} // end of else (there are some observations to add)
    }// for each camera

    ROS_INFO("now have %d observations after scene %d",total_observations_, scene_);
    scene_++;

    sprintf(msg, "Ical_srv now has %d observations after scene %d",total_observations_, scene_);
    res.message = string(msg);
    res.success = true;
    return(true);

  }; // end observation service};

  /**
   * \brief Load and set the calibration data from a previous job.
   */
  bool loadCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    string null_string("");
    if(problem_initialized_ != true ) // scene_id=0, problem re-initialized
      {
	ROS_INFO("calling start");
	std_srvs::TriggerRequest  sreq;
	std_srvs::TriggerResponse sres;
	startCallBack(sreq,sres);
      }


    // for each scene data includes
    //   a. image from each camera
    //   b. pose of each target
    // Note that the target's transform interface should provide the pose between the target and the camera that observes it
    // usually there is only one camera, and one target
    
    
    bool data_read_ok=true;
    while(data_read_ok){ //read all the scene data from image_directory
      for(int i=0; i<all_cameras_.size(); i++){
	data_read_ok &= all_cameras_[0]->camera_observer_->load_current_image(scene_, null_string);
      }
      for(int i=0; i<all_targets_.size(); i++){
	data_read_ok &= all_targets_[0]->transform_interface_->loadPose(scene_, null_string);
      }

      scene_++;

      if(data_read_ok)
	{
	  for(int i=0; i<all_cameras_.size(); i++){
	    // add the target to each camera's observer
	    Roi roi;    
	    roi.x_min = 0;
	    roi.y_min = 0;
	    
	    roi.x_max = all_cameras_[i]->camera_parameters_.width;
	    roi.y_max = all_cameras_[i]->camera_parameters_.height;
	    all_cameras_[i]->camera_observer_->clearTargets();
	    all_cameras_[i]->camera_observer_->clearObservations();
	    for(int j=0; j<all_targets_.size(); j++){
	      all_cameras_[i]->camera_observer_->addTarget(all_targets_[j], roi, industrial_extrinsic_cal::cost_functions::CameraReprjErrorWithDistortionPK);
	    }
	    
	    // wait for observers to do its work
	    while (!all_cameras_[i]->camera_observer_->observationsDone());


	    // get the observations from left and right cameras
	    CameraObservations camera_observations;
	    all_cameras_[i]->camera_observer_->getObservations(camera_observations);
	    
	    // only add if there are enough observations in both
	    int num_observations = (int)camera_observations.size();
	    
	    if(camera_observations.size() == all_targets_[0]->num_points_);
	    ROS_INFO("Found %d camera observations", (int)camera_observations.size());
	    
	    // add the moving target to the blocks
	    shared_ptr<Target> target = camera_observations[0].target;
	    ceres_blocks_.addMovingTarget(all_targets_[0], scene_);
	    P_BLOCK intrinsics  = ceres_blocks_.getStaticCameraParameterBlockIntrinsics(all_cameras_[i]->camera_name_);
	    P_BLOCK target_pb   = ceres_blocks_.getMovingTargetPoseParameterBlock(target->target_name_,scene_);
	  
	    CostFunction* cost_function[num_observations];  
	    for (int k = 0; k < num_observations; k++)
	      {
		shared_ptr<Target> target = camera_observations[k].target;
		double image_x = camera_observations[k].image_loc_x;
		double image_y = camera_observations[k].image_loc_y;
		Point3d point  = target->pts_[k];
		P_BLOCK intrinsics = ceres_blocks_.getStaticCameraParameterBlockIntrinsics(all_cameras_[i]->camera_name_);
		P_BLOCK target_pb  = ceres_blocks_.getMovingTargetPoseParameterBlock(target->target_name_,scene_);
		if(k==0){
		  ROS_ERROR("target_pb = %ld intrinsics = %ld",(long int ) &(target_pb[0]), (long int ) &intrinsics[0]);
		  Pose6d P(target_pb[3],target_pb[4],target_pb[5],target_pb[0],target_pb[1],target_pb[2]);
		  P.show("Pose of Target");
		}
		cost_function[k] = industrial_extrinsic_cal::CameraReprjErrorWithDistortionPK::Create(image_x, image_y, point);
		P_->AddResidualBlock(cost_function[k], NULL, target_pb, intrinsics);
	      }  // for each observation at this camera_location
	  }// end for each camera
	}// end if data_read_ok
    }
  }

  bool covCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    string covariance_file_name("/home/clewis/junk.txt");
    if(!computeCovariance(covariance_file_name)){
      ROS_ERROR("could not compute covariance");
      res.success = false;
      res.message = "failure";
    }
    else{
      res.message = "good job";
      res.success = true;
    }
    return(true);
  };

  bool computeCovariance(string& covariance_file_name)
  {
    FILE* fp;
    if ((fp = fopen(covariance_file_name.c_str(), "w")) != NULL)
    {
      ceres::Covariance::Options covariance_options;
      covariance_options.algorithm_type = ceres::DENSE_SVD;
      ceres::Covariance covariance(covariance_options);

      P_BLOCK extrinsics = ceres_blocks_.getStaticCameraParameterBlockExtrinsics(all_cameras_[1]->camera_name_);

      std::vector<std::pair<const double*, const double*> > covariance_pairs;
      covariance_pairs.push_back(std::make_pair(extrinsics, extrinsics));

      if(covariance.Compute(covariance_pairs, P_))
      {
        fprintf(fp, "stereo covariance block:\n");
        double cov[6*6];
        if(covariance.GetCovarianceBlock(extrinsics, extrinsics, cov)){
          fprintf(fp, "cov is 6x6\n");
          for(int i=0;i<6;i++){
            double sigma_i = sqrt(fabs(cov[i*6+i]));
            for(int j=0;j<6;j++){
              double sigma_j = sqrt(fabs(cov[j*6+j]));
              double value;
              if(i==j){
                value = sigma_i;
              }
              else{
                if(sigma_i==0) sigma_i = 1;
                if(sigma_j==0) sigma_j = 1;
                value = cov[i * 6 + j]/(sigma_i*sigma_j);
              }
              fprintf(fp, "%16.5f ", value);
            }  // end of j loop
            fprintf(fp, "\n");
          }  // end of i loop
        }// end if success getting covariance
        fclose(fp);
        return(true);
      }// end if covariances could be computed
      else{
        ROS_ERROR("could not compute covariance");
      }
    }// end if file opens
    ROS_ERROR("could not open covariance file %s", covariance_file_name.c_str());
    return (false);
  };  // end computeCovariance()

  bool runCallBack( industrial_extrinsic_cal::cal_srv_solveRequest &req, industrial_extrinsic_cal::cal_srv_solveResponse &res)
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
      res.message = string(msg);
      res.success = false;
      return(true);
    }
    if(total_observations_ == 0){
      ROS_ERROR("must call observations service at least once");
      sprintf(msg, "must call observations service at least once");
      res.message = string(msg);
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
	double final_cost = sqrt( 2*summary.final_cost   / total_observations_);
	res.final_cost_per_observation = final_cost;


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
	    ROS_ERROR("allowable cost exceeded %f > %f", final_cost, req.allowable_cost_per_observation);
	    sprintf(msg, "allowable cost exceeded %f > %f", final_cost, req.allowable_cost_per_observation);
	    res.message = string(msg);
	    res.success = false;
	    return(true);
	  }
	sprintf(msg, "final cost %f", final_cost);
	res.message = string(msg);
	res.success = true;
	return(true);
      }
    ROS_ERROR("NO CONVERGENCE");
    sprintf(msg, "NO CONVERGENCE");
    res.message = string(msg);
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

    res.message = string(msg);
    res.success = true;
    return(true);

  }// end saveCallback()

private:
  ros::NodeHandle nh_;
  std::vector<shared_ptr<Camera> > all_cameras_;
  std::vector<shared_ptr<Target> > all_targets_;
  string yaml_file_path_;
  string camera_file_;
  string target_file_;
  CeresBlocks ceres_blocks_;                 /*!< This structure maintains the parameter sets for ceres */
  ros::ServiceServer start_server_;
  ros::ServiceServer observation_server_;
  ros::ServiceServer run_server_;
  ros::ServiceServer save_server_;
  ros::ServiceServer load_server_;
  ros::ServiceServer covariance_server_;
  ceres::Problem *P_;
  bool problem_initialized_;
  int total_observations_;
  int scene_;
  bool save_data_;
  string image_directory_;
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
