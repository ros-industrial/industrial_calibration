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

class stereoCalServiceNode
{
public:
  stereoCalServiceNode(const  ros::NodeHandle& nh)
    :nh_(nh), P_(NULL), problem_initialized_(false), total_observations_(0), scene_(0)
  {
    std::string nn = ros::this_node::getName();
    ros::NodeHandle priv_nh("~");
    std::string target_mount_frame;
    std::string camera_mount_frame;
    
    // load cameras and targets
    priv_nh.getParam("yaml_file_path", yaml_file_path_);
    std::string camera_file, target_file;
    priv_nh.getParam("camera_file", camera_file);
    priv_nh.getParam("target_file", target_file);
    priv_nh.getParam("save_data", save_data_);
    priv_nh.getParam("data_directory", data_directory_);
    camera_file_ = yaml_file_path_ + camera_file ;
    target_file_ = yaml_file_path_ + target_file ;

    // provide feedback for the launch file developer
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

    // intiialize ceres blocks
    init_blocks();

    // advertise services
    start_server_       = nh_.advertiseService( "StereoCalSrvStart", &stereoCalServiceNode::startCallBack, this);
    observation_server_ = nh_.advertiseService( "StereoCalSrvObs", &stereoCalServiceNode::observationCallBack, this);
    run_server_         = nh_.advertiseService( "StereoCalSrvRun", &stereoCalServiceNode::runCallBack, this);
    save_server_        = nh_.advertiseService( "StereoCalSrvSave", &stereoCalServiceNode::saveCallBack, this);
    load_server_        = nh_.advertiseService( "StereoCalSrvLoad", &stereoCalServiceNode::loadCallBack, this);
    covariance_server_  = nh_.advertiseService( "StereoCalSrvCov",    &stereoCalServiceNode::covCallBack, this);

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
	    //	    ROS_ERROR("adding moving target: %s to scene 0", all_targets_[i]->target_name_.c_str());
	    ceres_blocks_.addMovingTarget(all_targets_[i], scene_);
	  }
	else
	  {
	    //	    ROS_ERROR("adding static target");
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
      all_cameras_[i]->transform_interface_->setDataDirectory(all_cameras_[i]->camera_observer_->image_directory_);
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
      all_targets_[i]->transform_interface_->setDataDirectory(all_cameras_[0]->camera_observer_->image_directory_);
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
    res.message = std::string("Stereo calibration service started");
    res.success = true;
    return(true);
  }

  // called to collect observations for the current pose of the scene
  bool observationCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    char msg[100];

    if(problem_initialized_ != true ){
      std_srvs::TriggerRequest  sreq;
      std_srvs::TriggerResponse sres;
      ROS_INFO("Problem not yet initialized, calling start service");
      startCallBack(sreq, sres);
    }

    // WARNING
    // assume only one target and two cameras 1st camera is the left camera and 2nd right
    // get the intitial conditions for this scene
    all_targets_[0]->pullTransform();

    // add the target to each camera's observer
    Roi roi;    
    roi.x_min = 0;
    roi.y_min = 0;
    
    roi.x_max = all_cameras_[0]->camera_parameters_.width;
    roi.y_max = all_cameras_[0]->camera_parameters_.height;
    all_cameras_[0]->camera_observer_->clearTargets();
    all_cameras_[0]->camera_observer_->clearObservations();
    all_cameras_[0]->camera_observer_->addTarget(all_targets_[0], roi, industrial_extrinsic_cal::cost_functions::WristStereoCal);

    roi.x_max = all_cameras_[1]->camera_parameters_.width;
    roi.y_max = all_cameras_[1]->camera_parameters_.height;
    all_cameras_[1]->camera_observer_->clearTargets();
    all_cameras_[1]->camera_observer_->clearObservations();
    all_cameras_[1]->camera_observer_->addTarget(all_targets_[0], roi, industrial_extrinsic_cal::cost_functions::WristStereoCal);

    // trigger both cameras as close to one another as possible
    all_cameras_[0]->camera_observer_->triggerCamera();
    all_cameras_[1]->camera_observer_->triggerCamera();

    // wait for both observers to get their images
    while (!all_cameras_[0]->camera_observer_->observationsDone());
    while (!all_cameras_[1]->camera_observer_->observationsDone());

    // get the observations from left and right cameras
    CameraObservations left_camera_observations;
    CameraObservations right_camera_observations;
    all_cameras_[0]->camera_observer_->getObservations(left_camera_observations);
    all_cameras_[1]->camera_observer_->getObservations(right_camera_observations);

    // only add if there are enough observations in both
    int num_observations = (int)left_camera_observations.size() + (int)right_camera_observations.size();

    // TODO add check proclivities if(!camera_->camera_observer_->checkObservationProclivity(camera_observations))

    if(left_camera_observations.size() == all_targets_[0]->num_points_ && right_camera_observations.size() == all_targets_[0]->num_points_){
      ROS_INFO("Found %d left camera observations and %d right camera observations", (int)left_camera_observations.size(), (int) right_camera_observations.size());

      // add the moving target to the blocks
      shared_ptr<Target> target = left_camera_observations[0].target;
      ceres_blocks_.addMovingTarget(all_targets_[0], scene_);
      P_BLOCK left_intrinsics  = ceres_blocks_.getStaticCameraParameterBlockIntrinsics(all_cameras_[0]->camera_name_);
      P_BLOCK right_intrinsics = ceres_blocks_.getStaticCameraParameterBlockIntrinsics(all_cameras_[1]->camera_name_);
      P_BLOCK right_extrinsics = ceres_blocks_.getStaticCameraParameterBlockExtrinsics(all_cameras_[1]->camera_name_);
      P_BLOCK target_pb        = ceres_blocks_.getMovingTargetPoseParameterBlock(target->target_name_,scene_);
      double lfx = left_intrinsics[0];
      double lfy = left_intrinsics[1];
      double lcx = left_intrinsics[2];
      double lcy = left_intrinsics[3];
      double rfx = right_intrinsics[0];
      double rfy = right_intrinsics[1];
      double rcx = right_intrinsics[2];
      double rcy = right_intrinsics[3];
	  
      // add observations from left and right camera to problem
      for (int k = 0; k < all_targets_[0]->num_points_; k++)
	{
	  total_observations_ += 2;
	  double left_image_x = left_camera_observations[k].image_loc_x;
	  double left_image_y = left_camera_observations[k].image_loc_y;
	  Point3d left_point  = target->pts_[left_camera_observations[k].point_id];
	  double right_image_x = right_camera_observations[k].image_loc_x;
	  double right_image_y = right_camera_observations[k].image_loc_y;
	  Point3d right_point  = target->pts_[right_camera_observations[k].point_id];
	  CostFunction* cost_function = industrial_extrinsic_cal::WristStereoCal::Create(left_image_x,  left_image_y,  left_point,
											 right_image_x, right_image_y, right_point,
											 lfx, lfy, lcx, lcy,
											 rfx, rfy, rcx, rcy);
	  P_->AddResidualBlock(cost_function, NULL, target_pb, right_extrinsics);
	}  // for each point on the target
      if (save_data_){
	  char pose_scene_chars[8];
	  char image_scene_chars[7];
	  sprintf(pose_scene_chars,"_%03d.yaml",scene_);
	  sprintf(image_scene_chars,"_%03d.jpg",scene_);
	  std::string image1_file = all_cameras_[0]->camera_name_ + std::string(image_scene_chars);
	  std::string image2_file = all_cameras_[1]->camera_name_ + std::string(image_scene_chars);
	  std::string extrinsics_scene_d_yaml = std::string("_extrinsics") + std::string(pose_scene_chars);
	  std::string target_extrinsics = all_targets_[0]->target_name_ + extrinsics_scene_d_yaml;
	  if(!all_cameras_[0]->camera_observer_->save_current_image(scene_,image1_file)){
	    ROS_ERROR("couldn't save image named %s",image1_file.c_str());
	  }
	  else{
	    ROS_INFO("saved image as %s",image1_file.c_str());
	  }
	  if(!all_cameras_[1]->camera_observer_->save_current_image(scene_,image2_file)){
	    ROS_ERROR("couldn't save image named %s",image2_file.c_str());
	  }
	  else{
	    ROS_INFO("saved image as %s",image2_file.c_str());
	  }
	  all_targets_[0]->transform_interface_->saveCurrentPose(scene_, target_extrinsics);
	  if(scene_ == 0){
	    std::string camera_extrinsics = all_cameras_[1]->camera_name_ + extrinsics_scene_d_yaml;
	    all_cameras_[1]->pullTransform();
	    all_cameras_[1]->transform_interface_->saveCurrentPose(scene_,camera_extrinsics);
	  }

      }// end save_data_

      ROS_INFO("now have %d observations after scene %d",total_observations_, scene_);
      scene_++;
      
      sprintf(msg, "stereo_cal_srv now has %d observations after scene %d",total_observations_, scene_);
      res.message = std::string(msg);
      res.success = true;
    }// end if both cameras were abale to observe the target
    else{
      ROS_INFO("Found %d left camera observations and %d right camera observations", (int)left_camera_observations.size(), (int) right_camera_observations.size());
      sprintf(msg, "stereo_cal_srv couldn't find target");
      res.message = std::string(msg);
      res.success = false;
    }
    return(true);
  }; // end observation service};

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
      Pose6d CP = all_cameras_[i]->transform_interface_->pullTransform();
      char msg[100];
      sprintf(msg,"Camera %s initial pose",all_cameras_[i]->camera_name_.c_str());
      CP.show(msg);
    }
    /* show initial conditions  TODO remove this debugging code before PR to upstream master
    for(int i=0;i<scene_;i++){
      P_BLOCK te = ceres_blocks_.getMovingTargetPoseParameterBlock(all_targets_[0]->target_name_,i);
      Pose6d TP(te[3],te[4],te[5],te[0],te[1],te[2]);
      char msg[100];
      sprintf(msg,"Target Pose in Scene %d",i);
      TP.show(msg);
    }
    */

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

	ROS_INFO("Problem solved, initial cost = %lf, final cost = %lf", initial_cost, final_cost);
	res.final_cost_per_observation = final_cost;
	if (final_cost <= req.allowable_cost_per_observation)
	  {
	    ROS_INFO("Stereo calibration was successful");
	  }
	else
	  {
	    ROS_ERROR("allowable cost exceeded %f > %f", final_cost, req.allowable_cost_per_observation);
	    sprintf(msg, "allowable cost exceeded %f > %f", final_cost, req.allowable_cost_per_observation);
	    Pose6d P(all_cameras_[1]->camera_parameters_.pb_extrinsics[3],
		     all_cameras_[1]->camera_parameters_.pb_extrinsics[4],
		     all_cameras_[1]->camera_parameters_.pb_extrinsics[5],
		     all_cameras_[1]->camera_parameters_.pb_extrinsics[0],
		     all_cameras_[1]->camera_parameters_.pb_extrinsics[1],
		     all_cameras_[1]->camera_parameters_.pb_extrinsics[2]);
	    P.show("final right camera_extrinsics");

	    /* show poses of target after optimization TODO remove this debugging code before PR to upstream master
	       for(int i=0;i<scene_;i++){
	       P_BLOCK te = ceres_blocks_.getMovingTargetPoseParameterBlock(all_targets_[0]->target_name_,i);
	       Pose6d TP(te[3],te[4],te[5],te[0],te[1],te[2]);
	       char msg[100];
	       sprintf(msg,"Target Pose in Scene %d",i);
	       TP.show(msg);
	       }
	    */
	    
	    res.message = std::string(msg);
	    res.success = false;
	    return(true);
	  }
	sprintf(msg, "final cost %f", final_cost);
	res.message = std::string(msg);
	res.success = true;
	return(true);
      }
    ROS_ERROR("Stereo Cal NO CONVERGENCE");
    sprintf(msg, "Stereo Cal NO CONVERGENCE");
    res.message = std::string(msg);
    res.success = false;
    return(true);
  }; // end runCallBack()

  bool saveCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    for(int i=0; i<all_cameras_.size(); i++){
      all_cameras_[i]->pushTransform();
    }
    for(int i=0; i<all_targets_.size(); i++){
      all_targets_[i]->pushTransform();
    }

    res.message = "Stereo Cal: pushed transforms to cameras and targets";
    res.success = true;
    return(true);

  }// end saveCallback()

  /**
   * \brief Load and set the calibration data from a previous job.
   */
  bool loadCallBack(industrial_extrinsic_cal::FileOp::Request &req, industrial_extrinsic_cal::FileOp::Response &resp)
  {
    if(problem_initialized_ != true ){ // scene_id=0, problem re-initialized
      ROS_INFO("calling start");
      std_srvs::TriggerRequest  sreq;
      std_srvs::TriggerResponse sres;
      startCallBack(sreq,sres);
    }

      // data includes
      //   initial pose between right camera and left camera
      // for each scene
      //   a. image from left camera
      //   b. image from right camera
      //   c. pose from left camera mount to target mount

    // read initial stereo pose
    std::string null_string("");
    all_cameras_[1]->transform_interface_->loadPose(0,null_string);
    Pose6d initial_stereo_cal = all_cameras_[1]->transform_interface_->getCurrentPose();

    bool data_read_ok=true;
    while(data_read_ok){ //read all the scene data from image_directory
      data_read_ok &= all_cameras_[0]->camera_observer_->load_current_image(scene_, null_string);
      data_read_ok &= all_cameras_[1]->camera_observer_->load_current_image(scene_, null_string);
      data_read_ok &= all_targets_[0]->transform_interface_->loadPose(scene_, null_string);
      scene_++;
      if(data_read_ok)
	{
	  // add the target to each camera's observer
	  Roi roi;    
	  roi.x_min = 0;
	  roi.y_min = 0;
	  
	  roi.x_max = all_cameras_[0]->camera_parameters_.width;
	  roi.y_max = all_cameras_[0]->camera_parameters_.height;
	  all_cameras_[0]->camera_observer_->clearTargets();
	  all_cameras_[0]->camera_observer_->clearObservations();
	  all_cameras_[0]->camera_observer_->addTarget(all_targets_[0], roi, industrial_extrinsic_cal::cost_functions::WristStereoCal);

	  roi.x_max = all_cameras_[1]->camera_parameters_.width;
	  roi.y_max = all_cameras_[1]->camera_parameters_.height;
	  all_cameras_[1]->camera_observer_->clearTargets();
	  all_cameras_[1]->camera_observer_->clearObservations();
	  all_cameras_[1]->camera_observer_->addTarget(all_targets_[0], roi, industrial_extrinsic_cal::cost_functions::WristStereoCal);

	  // wait for both observers to get their images
	  while (!all_cameras_[0]->camera_observer_->observationsDone());
	  while (!all_cameras_[1]->camera_observer_->observationsDone());

	  // get the observations from left and right cameras
	  CameraObservations left_camera_observations;
	  CameraObservations right_camera_observations;
	  all_cameras_[0]->camera_observer_->getObservations(left_camera_observations);
	  all_cameras_[1]->camera_observer_->getObservations(right_camera_observations);

	  // only add if there are enough observations in both
	  int num_observations = (int)left_camera_observations.size() + (int)right_camera_observations.size();

	
	  if(left_camera_observations.size() == all_targets_[0]->num_points_ && right_camera_observations.size() == all_targets_[0]->num_points_){
	    ROS_INFO("Found %d left camera observations and %d right camera observations", (int)left_camera_observations.size(), (int) right_camera_observations.size());

	    // add the moving target to the blocks
	    shared_ptr<Target> target = left_camera_observations[0].target;
	    ceres_blocks_.addMovingTarget(all_targets_[0], scene_);
	    P_BLOCK left_intrinsics  = ceres_blocks_.getStaticCameraParameterBlockIntrinsics(all_cameras_[0]->camera_name_);
	    P_BLOCK right_intrinsics = ceres_blocks_.getStaticCameraParameterBlockIntrinsics(all_cameras_[1]->camera_name_);
	    P_BLOCK right_extrinsics = ceres_blocks_.getStaticCameraParameterBlockExtrinsics(all_cameras_[1]->camera_name_);
	    P_BLOCK target_pb        = ceres_blocks_.getMovingTargetPoseParameterBlock(target->target_name_,scene_);
	    double lfx = left_intrinsics[0];
	    double lfy = left_intrinsics[1];
	    double lcx = left_intrinsics[2];
	    double lcy = left_intrinsics[3];
	    double rfx = right_intrinsics[0];
	    double rfy = right_intrinsics[1];
	    double rcx = right_intrinsics[2];
	    double rcy = right_intrinsics[3];
	  
	    // add observations from left and right camera to problem
	    for (int k = 0; k < all_targets_[0]->num_points_; k++)
	      {
		total_observations_ += 2;
		double left_image_x = left_camera_observations[k].image_loc_x;
		double left_image_y = left_camera_observations[k].image_loc_y;
		Point3d left_point  = target->pts_[left_camera_observations[k].point_id];
		double right_image_x = right_camera_observations[k].image_loc_x;
		double right_image_y = right_camera_observations[k].image_loc_y;
		Point3d right_point  = target->pts_[right_camera_observations[k].point_id];
		CostFunction* cost_function = industrial_extrinsic_cal::WristStereoCal::Create(left_image_x,  left_image_y,  left_point,
											       right_image_x, right_image_y, right_point,
											       lfx, lfy, lcx, lcy,
											       rfx, rfy, rcx, rcy);
		P_->AddResidualBlock(cost_function, NULL, target_pb, right_extrinsics);
	      }  // for each point on the target
	  }// end if enough observations, must have found the target in both images 
	}// end if data_read_ok
    }
  }

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
  industrial_extrinsic_cal::ROSListenerTransInterface *target_to_left_camera_TI_;
};// end of class stereoCalServiceNode

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_cal_service");
  ros::NodeHandle node_handle;
  stereoCalServiceNode SCSN(node_handle);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
