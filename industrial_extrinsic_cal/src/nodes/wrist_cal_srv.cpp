#include <ros/ros.h>

#include <string>
#include <fstream>
#include <memory>

// ros includes
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/Image.h>

// ros actionlib includes
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

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
#include <industrial_extrinsic_cal/ros_transform_interface.h>

//defined services
#include <industrial_extrinsic_cal/cal_srv_solve.h>
#include <industrial_extrinsic_cal/FileOp.h>

// boost includes
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>

#include <yaml-cpp/yaml.h>

using boost::shared_ptr;
using boost::filesystem::path;
using boost::dynamic_pointer_cast;

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

class wristCalServiceNode
{
public:
  wristCalServiceNode(const  ros::NodeHandle& nh)
    :nh_(nh), P_(NULL), problem_initialized_(false), total_observations_(0), scene_(0)
  {
    std::string nn = ros::this_node::getName();
    ros::NodeHandle priv_nh("~");
    std::string target_mount_frame;
    std::string camera_mount_frame;
    std::string camera_file, target_file;
    
    // load cameras and targets
    if(!priv_nh.getParam("yaml_file_path", yaml_file_path_)){
      ROS_ERROR("Must set param: yaml_file_path desribing where to find camera_file.yaml and target_file.yaml");
    }
    if(!priv_nh.getParam("camera_file", camera_file)){
      ROS_ERROR("Must set param: camera_file, this defines the camera to calibrate");
    }
    if(!priv_nh.getParam("target_file", target_file)){
      ROS_ERROR("Must set param: target_file, this defines the target used for calibration");
    }

    if(!priv_nh.getParam("target_mount_frame", target_mount_frame)){
      ROS_ERROR("Must set param: target_mount_frame, this defines the tf-frame on which the target is mounted");
    }
    if(!priv_nh.getParam("camera_mount_frame", camera_mount_frame)){
      ROS_ERROR("Must set param: target_mount_frame, this defines the tf-frame on which the target is mounted");
    }
    
    priv_nh.getParam("save_data", save_data_);
    priv_nh.getParam("data_directory", data_directory_);
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

    // initialize the target_mount to camera_mount transform listener
    targetm_to_cameram_TI_ = new industrial_extrinsic_cal::ROSListenerTransInterface(target_mount_frame);
    targetm_to_cameram_TI_->setReferenceFrame(camera_mount_frame);
    targetm_to_cameram_TI_->setDataDirectory(data_directory_);

    // intiialize ceres blocks
    init_blocks();

    // advertise services
    start_server_       = nh_.advertiseService( "WristCalSrvStart",  &wristCalServiceNode::startCallBack, this);
    load_server_        = nh_.advertiseService( "WristCalSrvLoad",   &wristCalServiceNode::loadCallBack, this);
    observation_server_ = nh_.advertiseService( "WristCalSrvObs",    &wristCalServiceNode::observationCallBack, this);
    run_server_         = nh_.advertiseService( "WristCalSrvRun",    &wristCalServiceNode::runCallBack, this);
    save_server_        = nh_.advertiseService( "WristCalSrvSave",   &wristCalServiceNode::saveCallBack, this);
    covariance_server_  = nh_.advertiseService( "WristCalSrvCov",    &wristCalServiceNode::covCallBack, this);

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
    for(int i=0;i<(int)all_cameras_.size();i++){
      all_cameras_[i]->transform_interface_->setDataDirectory(data_directory_);
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
    for(int i=0;i<(int)all_targets_.size();i++){
      all_targets_[i]->transform_interface_->setDataDirectory(data_directory_);
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
    res.message = std::string("Wrist calibration service started");
    res.success = true;
    return(true);
  }

  // called to collect observations for the current pose of the scene
  bool observationCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    char msg[100];
    Pose6d TtoC = targetm_to_cameram_TI_->pullTransform();
    TtoC.show("TtoC");
    
    if(problem_initialized_ != true ){
      std_srvs::TriggerRequest  sreq;
      std_srvs::TriggerResponse sres;
      ROS_INFO("Problem not yet initialized, calling start service");
      startCallBack(sreq, sres);
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
      else if(!all_cameras_[i]->camera_observer_->checkObservationProclivity(camera_observations))
        {
          ROS_ERROR("Proclivities Check not successful");
        }
      else
	{	 // add a new cost to the problem for each observation
	  // CostFunction* cost_function[num_observations];  
	  total_observations_ += num_observations;
	  ceres_blocks_.addMovingTarget(camera_observations[0].target, scene_);
	  for (int k = 0; k < num_observations; k++)
	    {
	      shared_ptr<Target> target = camera_observations[k].target;
	      double image_x = camera_observations[k].image_loc_x;
	      double image_y = camera_observations[k].image_loc_y;
	      Point3d point  = target->pts_[camera_observations[k].point_id];
	      P_BLOCK intrinsics = ceres_blocks_.getStaticCameraParameterBlockIntrinsics(all_cameras_[i]->camera_name_);
	      P_BLOCK extrinsics = ceres_blocks_.getStaticCameraParameterBlockExtrinsics(all_cameras_[i]->camera_name_);
	      P_BLOCK target_pb  = ceres_blocks_.getStaticTargetPoseParameterBlock(target->target_name_);
	      double fx = intrinsics[0];
	      double fy = intrinsics[1];
	      double cx = intrinsics[2];
	      double cy = intrinsics[3];
	      CostFunction* cost_function = industrial_extrinsic_cal::LinkTargetCameraReprjErrorPK::Create(image_x, image_y, fx, fy, cx, cy, TtoC, point);
	      /*
	      if(k<3){
		ROS_ERROR("target_pb = %ld extrinsics %ld intrinsics = %ld",(long int ) &(target_pb[0]),(long int) &extrinsics[0], (long int ) &intrinsics[0]);
		Pose6d P(extrinsics[3],extrinsics[4],extrinsics[5],extrinsics[0],extrinsics[1],extrinsics[2]);
		P.show("camera to world initial conditions");
		Pose6d TP(target_pb[3],target_pb[4],target_pb[5],target_pb[0],target_pb[1],target_pb[2]);
		TP.show("tool0 to Target initial conditions");
		//Pose6d T;
		//T = P*TtoC;
		//	Pose6d T2;
		//	T2 = T*TP;
		//T2.show("full transform");
		double resid[2];
		double *params[2];
		double J[12][12];
		double **JP;
		JP[0] = J[0];
		JP[1] = J[1];
		params[0] = extrinsics;
		params[1] = target_pb;
		bool rtn = cost_function->Evaluate(params, resid, JP);
		ROS_ERROR("Target Point %lf %lf %lf observed %lf %lf Residual = %lf %lf",point.x, point.y, point.z, image_x, image_y, resid[0],resid[1]);
	      }
	      */

	      P_->AddResidualBlock(cost_function, NULL, extrinsics, target_pb);
	    }  // for each observation at this camera_location
	} // end of else (there are some observations to add)
    }// for each camera
    

    if (save_data_){
      char pose_scene_chars[8];
      char image_scene_chars[7];
      sprintf(pose_scene_chars,"_%03d.yaml",scene_);
      sprintf(image_scene_chars,"_%03d.jpg",scene_);
      std::string image_file = all_cameras_[0]->camera_name_ + std::string(image_scene_chars);
      std::string extrinsics_scene_d_yaml = std::string("_extrinsics") + std::string(pose_scene_chars);
      std::string camera_mount_to_target_mount= std::string("Cm_to_Tm") + std::string(pose_scene_chars);  // write pose info to data_directory_/Cm_to_tm_sceneID.yaml
      std::string camera_extrinsics = all_cameras_[0]->camera_name_ + extrinsics_scene_d_yaml; // write pose to data_directory_/camera_name_extrinsics_sceneID.yaml
      std::string target_extrinsics = all_targets_[0]->target_name_ + extrinsics_scene_d_yaml; // write pose to data_directory_/target_name_extrinsics_sceneID.yaml
      all_cameras_[0]->camera_observer_->save_current_image(scene_,image_file);
      targetm_to_cameram_TI_->saveCurrentPose(scene_,camera_mount_to_target_mount);
      all_cameras_[0]->transform_interface_->saveCurrentPose(scene_,camera_extrinsics);
      all_targets_[0]->transform_interface_->saveCurrentPose(scene_,target_extrinsics);
    }// end save_data_
    else{
      ROS_ERROR("save_data_ not set");
    }
    ROS_INFO("now have %d observations after scene %d",total_observations_, scene_);
    scene_++;

    sprintf(msg, "Ical_srv now has %d observations after scene %d",total_observations_, scene_);
    res.message = std::string(msg);
    res.success = true;
    return(true);

  }; // end observation service};

  bool covCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    std::string covariance_file_name("/home/clewis/junk.txt");
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

  bool computeCovariance(std::string& covariance_file_name)
  {
    FILE* fp;
    if ((fp = fopen(covariance_file_name.c_str(), "w")) != NULL)
    {
      ceres::Covariance::Options covariance_options;
      covariance_options.algorithm_type = ceres::DENSE_SVD;
      ceres::Covariance covariance(covariance_options);

      P_BLOCK extrinsics = ceres_blocks_.getStaticCameraParameterBlockExtrinsics(all_cameras_[0]->camera_name_);
      P_BLOCK target_pb  = ceres_blocks_.getStaticTargetPoseParameterBlock(all_targets_[0]->target_name_);

      std::vector<std::pair<const double*, const double*> > covariance_pairs;
      covariance_pairs.push_back(std::make_pair(extrinsics, extrinsics));
      covariance_pairs.push_back(std::make_pair(target_pb, target_pb));
      covariance_pairs.push_back(std::make_pair(extrinsics,target_pb));

      if(covariance.Compute(covariance_pairs, P_))
      {
        fprintf(fp, "covariance blocks:\n");
        double cov_ex_ex[6*6];
	double cov_t_t[6*6];
	double cov_ex_t[6*6];
        if(covariance.GetCovarianceBlock(extrinsics, extrinsics, cov_ex_ex) &&
	   covariance.GetCovarianceBlock(target_pb, target_pb, cov_t_t) &&
	   covariance.GetCovarianceBlock(extrinsics, target_pb, cov_ex_t)){
          fprintf(fp, "cov_ex_ex is 6x6\n");
          for(int i=0;i<6;i++){
            double sigma_i = sqrt(fabs(cov_ex_ex[i*6+i]));
            for(int j=0;j<6;j++){
              double sigma_j = sqrt(fabs(cov_ex_ex[j*6+j]));
              double value;
              if(i==j){
                value = sigma_i;
              }
              else{
                if(sigma_i==0) sigma_i = 1;
                if(sigma_j==0) sigma_j = 1;
                value = cov_ex_ex[i * 6 + j]/(sigma_i*sigma_j);
              }
              fprintf(fp, "%16.5f ", value);
            }  // end of j loop
            fprintf(fp, "\n");
          }  // end of i loop
	   fprintf(fp, "cov_t_t is 6x6\n");
          for(int i=0;i<6;i++){
            double sigma_i = sqrt(fabs(cov_t_t[i*6+i]));
            for(int j=0;j<6;j++){
              double sigma_j = sqrt(fabs(cov_t_t[j*6+j]));
              double value;
              if(i==j){
                value = sigma_i;
              }
              else{
                if(sigma_i==0) sigma_i = 1;
                if(sigma_j==0) sigma_j = 1;
                value = cov_t_t[i * 6 + j]/(sigma_i*sigma_j);
              }
              fprintf(fp, "%16.5f ", value);
            }  // end of j loop
            fprintf(fp, "\n");
          }  // end of i loop
          fprintf(fp, "cov_ex_t is 6x6\n");
          for(int i=0;i<6;i++){
            double sigma_i = sqrt(fabs(cov_ex_ex[i*6+i]));
            for(int j=0;j<6;j++){
              double sigma_j = sqrt(fabs(cov_t_t[j*6+j]));
              double value;
              if(sigma_i==0) sigma_i = 1;
              if(sigma_j==0) sigma_j = 1;
              value = cov_ex_t[i * 6 + j]/(sigma_i*sigma_j);
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

    // Set intitial conditions of camera poses
    for(int i=0; i<all_cameras_.size(); i++){
      all_cameras_[i]->pullTransform();
    }

    // Set intitial conditions of target poses
    for(int i=0; i<all_targets_.size(); i++){
      all_targets_[i]->pullTransform();
    }

    // output initial conditions
    Pose6d P(all_cameras_[0]->camera_parameters_.pb_extrinsics[3],
	     all_cameras_[0]->camera_parameters_.pb_extrinsics[4],
	     all_cameras_[0]->camera_parameters_.pb_extrinsics[5],
	     all_cameras_[0]->camera_parameters_.pb_extrinsics[0],
	     all_cameras_[0]->camera_parameters_.pb_extrinsics[1],
	     all_cameras_[0]->camera_parameters_.pb_extrinsics[2]);
    P.show("initial camera_extrinsics");
    P_BLOCK target_pb  = ceres_blocks_.getStaticTargetPoseParameterBlock(all_targets_[0]->target_name_);
    Pose6d TP(target_pb[3],target_pb[4],target_pb[5],target_pb[0],target_pb[1],target_pb[2]);
    TP.show("initial target extrinsics");


    Solver::Options options;
    Solver::Summary summary;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 1000;
    ceres::Solve(options, P_, &summary);
    if (summary.termination_type != ceres::NO_CONVERGENCE)
      {
	double initial_cost = sqrt(2*summary.initial_cost / total_observations_);
	double final_cost = sqrt(2*summary.final_cost / total_observations_);
	res.final_cost_per_observation = final_cost;

	ROS_INFO("Problem solved, initial cost = %lf, final cost = %lf", initial_cost, final_cost);
	if (final_cost <= req.allowable_cost_per_observation)
	  {
	    ROS_INFO("Wrist calibration was successful");
	  }
	else
	  {
	    ROS_ERROR("allowable cost exceeded %f > %f", final_cost, req.allowable_cost_per_observation);
	    sprintf(msg, "allowable cost exceeded %f > %f", final_cost, req.allowable_cost_per_observation);
	    Pose6d Pf(all_cameras_[0]->camera_parameters_.pb_extrinsics[3],
		     all_cameras_[0]->camera_parameters_.pb_extrinsics[4],
		     all_cameras_[0]->camera_parameters_.pb_extrinsics[5],
		     all_cameras_[0]->camera_parameters_.pb_extrinsics[0],
		     all_cameras_[0]->camera_parameters_.pb_extrinsics[1],
		     all_cameras_[0]->camera_parameters_.pb_extrinsics[2]);
	    Pf.show("final camera_extrinsics");
	    P_BLOCK target_fpb  = ceres_blocks_.getStaticTargetPoseParameterBlock(all_targets_[0]->target_name_);
	    Pose6d FTP(target_fpb[3],target_fpb[4],target_fpb[5],target_fpb[0],target_fpb[1],target_fpb[2]);
	    FTP.show("final target extrinsics");

	    res.message = std::string(msg);
	    res.success = false;
	    return(true);
	  }
	sprintf(msg, "final cost %f", final_cost);
	res.message = std::string(msg);
	res.success = true;
	return(true);
      }
    ROS_ERROR("Wrist Cal NO CONVERGENCE");
    sprintf(msg, "Wrist Cal NO CONVERGENCE");
    res.message = std::string(msg);
    res.success = false;
    return(true);
  }; // end runCallBack()

  bool saveCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    ROS_ERROR("inside saveCallBack()");
    for(int i=0; i<all_cameras_.size(); i++){
      all_cameras_[i]->pushTransform();
    }
    for(int i=0; i<all_targets_.size(); i++){
      all_targets_[i]->pushTransform();
    }

    res.message = "Wrist Cal: pushed transforms to cameras and targets";
    res.success = true;
    return(true);

  }// end saveCallback()

  /**
   * \brief Load and set the calibration data from a previous job.
   */
  bool loadCallBack(industrial_extrinsic_cal::FileOp::Request &req, industrial_extrinsic_cal::FileOp::Response &resp)
  {
    industrial_extrinsic_cal::Cost_function cost_type = industrial_extrinsic_cal::cost_functions::CameraReprjErrorWithDistortionPK;
    Roi roi;
    roi.x_min = 0;
    roi.y_min = 0;
    roi.x_max = all_cameras_[0]->camera_parameters_.width;
    roi.y_max = all_cameras_[0]->camera_parameters_.height;
    int total_pts= all_targets_[0]->num_points_;

    if(problem_initialized_ != true ){ // scene_id=0, problem re-initialized
      ROS_INFO("calling start");
      std_srvs::TriggerRequest  sreq;
      std_srvs::TriggerResponse sres;
      startCallBack(sreq,sres);
    }
    
    bool data_read_ok=true;
    while(data_read_ok){
      char pose_scene_chars[8];
      char image_scene_chars[7];
      std::string image_file = all_cameras_[0]->camera_name_ + std::string(image_scene_chars);
      std::string extrinsics_scene_d_yaml = std::string("_extrinsics") + std::string(pose_scene_chars);
      std::string camera_mount_to_target_mount= std::string("Cm_to_Tm") + std::string(pose_scene_chars);  // write pose info to data_directory_/Cm_to_tm_sceneID.yaml
      std::string camera_extrinsics = all_cameras_[0]->camera_name_ + extrinsics_scene_d_yaml; // write pose to data_directory_/camera_name_extrinsics_sceneID.yaml
      std::string target_extrinsics = all_targets_[0]->target_name_ + extrinsics_scene_d_yaml; // write pose to data_directory_/target_name_extrinsics_sceneID.yaml
      
      data_read_ok &= targetm_to_cameram_TI_->loadPose(scene_,camera_mount_to_target_mount);
      data_read_ok &= all_cameras_[0]->camera_observer_->load_current_image(scene_,image_file);
      
      if(data_read_ok){
	Pose6d TtoC = targetm_to_cameram_TI_->getCurrentPose();
	TtoC.show("TtoC");
	
	
	// set target and get observations from the image
	CameraObservations camera_observations;
	all_cameras_[0]->camera_observer_->clearTargets();
	all_cameras_[0]->camera_observer_->clearObservations();
	all_cameras_[0]->camera_observer_->addTarget(all_targets_[0], roi, cost_type);
	while (!all_cameras_[0]->camera_observer_->observationsDone());
	all_cameras_[0]->camera_observer_->getObservations(camera_observations);
	
	int num_observations = (int)camera_observations.size();
	ROS_INFO("Found %d observations", (int)camera_observations.size());
	
	// add observations to problem
	num_observations = (int)camera_observations.size();
	if (num_observations != total_pts)
	  {
	    ROS_ERROR("Target Locator could not find all targets found %d out of %d", num_observations, total_pts);
	  }
	else	 // add a new cost to the problem for each observation
	  {	 
	    total_observations_ += num_observations;
	    ceres_blocks_.addMovingTarget(camera_observations[0].target, scene_);
	    for (int k = 0; k < num_observations; k++)
	      {
		shared_ptr<Target> target = camera_observations[k].target;
		double image_x = camera_observations[k].image_loc_x;
		double image_y = camera_observations[k].image_loc_y;
		Point3d point  = target->pts_[camera_observations[k].point_id];
		P_BLOCK intrinsics = ceres_blocks_.getStaticCameraParameterBlockIntrinsics(all_cameras_[0]->camera_name_);
		P_BLOCK extrinsics = ceres_blocks_.getStaticCameraParameterBlockExtrinsics(all_cameras_[0]->camera_name_);
		P_BLOCK target_pb  = ceres_blocks_.getStaticTargetPoseParameterBlock(target->target_name_);
		double fx = intrinsics[0];
		double fy = intrinsics[1];
		double cx = intrinsics[2];
		double cy = intrinsics[3];
		CostFunction* cost_function = industrial_extrinsic_cal::LinkTargetCameraReprjErrorPK::Create(image_x, image_y, fx, fy, cx, cy, TtoC, point);
		P_->AddResidualBlock(cost_function, NULL, extrinsics, target_pb);
	      }  // for each observation at this camera_location
	  } // end of else (there are some observations to add)
	scene_++;
      }// end if data_read_ok
    }
    return true;
  }




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
  ros::ServiceServer load_server_;
  ros::ServiceServer covariance_server_;
  ceres::Problem *P_;
  bool problem_initialized_;

  bool save_data_;
  std::string data_directory_;
  std::string save_subdir_;

  int total_observations_;
  int scene_;
  industrial_extrinsic_cal::ROSListenerTransInterface *targetm_to_cameram_TI_;
};// end of class wristCalServiceNode

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wrist_cal_service");
  ros::NodeHandle node_handle;
  wristCalServiceNode WCSN(node_handle);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
