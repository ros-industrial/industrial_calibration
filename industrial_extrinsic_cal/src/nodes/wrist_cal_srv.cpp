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
    covariance_server_  = nh_.advertiseService( "ICalSrvCov",        &wristCalServiceNode::covCallBack, this);

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
      sprintf(msg, "must call start service");
      ROS_ERROR("%s",msg);
      res.message = std::string(msg);
      res.success = false;
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
	if (final_cost <= req.allowable_cost_per_observation)
	  {
	    ROS_INFO("Wrist calibration was successful");
	  }
	else
	  {
	    res.final_cost_per_observation = final_cost;
	    ROS_ERROR("allowable cost exceeded %f > %f", final_cost, req.allowable_cost_per_observation);
	    sprintf(msg, "allowable cost exceeded %f > %f", final_cost, req.allowable_cost_per_observation);
	    Pose6d P(all_cameras_[0]->camera_parameters_.pb_extrinsics[3],
		     all_cameras_[0]->camera_parameters_.pb_extrinsics[4],
		     all_cameras_[0]->camera_parameters_.pb_extrinsics[5],
		     all_cameras_[0]->camera_parameters_.pb_extrinsics[0],
		     all_cameras_[0]->camera_parameters_.pb_extrinsics[1],
		     all_cameras_[0]->camera_parameters_.pb_extrinsics[2]);
	    P.show("final camera_extrinsics");
	    P_BLOCK target_pb  = ceres_blocks_.getStaticTargetPoseParameterBlock(all_targets_[0]->target_name_);
	    Pose6d TP(target_pb[3],target_pb[4],target_pb[5],target_pb[0],target_pb[1],target_pb[2]);
	    TP.show("tool0 to Target final conditions");

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
    
    char current_image_scene_chars[255];
    sprintf(current_image_scene_chars,"_%03d_%03d.jpg",scene_,0); // add rail distance index to image, scene_ will automatically be added by camera_observer.save_current_image()
    std::string image_file_currently = camera_->camera_name_ + std::string(current_image_scene_chars);
    std::string image_file_now = current_image_file(scene_,image_file_currently);

    while(camera_->camera_observer_->exists_test(image_file_now)){


    path directory = path(data_directory_) / req.name;
    ROS_INFO_STREAM("Loading from directory: " << directory);
    int scene_idx = 1;
    bool finished = false;
    while (!finished)
    {
      artemis_msgs::GetStereoImages img_srv_data;
      std::map<std::string, Pose6d> transforms;
      if (loadSceneData(scene_idx, directory.string(), img_srv_data, transforms))
      {
        ROS_INFO_STREAM("Loaded scene " << scene_idx);
        observeScene(img_srv_data, transforms);
        scene_idx++;
      }
      else
        finished = true;
    }
    auto res_str = boost::format("Loaded data from %d scenes") % (scene_idx-1);
    resp.result = res_str.str();
    */
    return true;
  }

  bool loadSceneData(

      int scene_idx,
      const std::string dir,
      sensor_msgs::Image &left_image,
      sensor_msgs::Image &right_image,
      std::map<std::string, Pose6d> &transforms)
  {
    /*
    char tmp_name[255];
    std::string side("left");
    sprintf(tmp_name,"%scapture_%03d_%s.jpg",path(dir)scene_idx, side);
    std::string get_image_file(tmp_name);
    
    //Function to create the file string for each image
    auto get_image_file = [&](std::string side) -> path
    {
      auto img_file = boost::format("capture_%03d_%s.jpg") % scene_idx % side;
      path img_path = path(dir) / img_file.str();
      return img_path;
    };
    //Load left image
    auto left_image_path = get_image_file("left");
    if (! boost::filesystem::is_regular_file(left_image_path))
    {
      return false;
    }
    loadImage(left_image_path, img_srv_data.response.left_image);
    //Load right image
    auto right_image_path = get_image_file("right");
    if (! boost::filesystem::is_regular_file(right_image_path))
    {
      return false;
    }
    loadImage(right_image_path, img_srv_data.response.right_image);
    //Create transforms file string and load YAML
    auto tf_file = boost::format("capture_%03d_transform.yaml") % scene_idx;
    path tf_file_path = path(dir) / tf_file.str();
    if (! boost::filesystem::is_regular_file(tf_file_path))
    {
      return false;
    }
    if (! loadTransformYAML(tf_file_path, transforms))
    {
      ROS_ERROR_STREAM("Problem loading transform yaml file at scene " << scene_idx);
      return false;
    }
		     */
    return true;
  }

  void saveObservationData(
      int scene_idx,
      const sensor_msgs::Image &left_image,
      const sensor_msgs::Image &right_image,
      const std::map<std::string, Pose6d> &transforms)
  {
    if (save_subdir_.empty())
    {
      ROS_WARN("No calibration data subdir set, cannot save observation data");
      return;
    }

    path save_dir = path(data_directory_) / save_subdir_;
    if (! boost::filesystem::is_directory(save_dir))
    {
      try
      {
        boost::filesystem::create_directories(save_dir);
      }
      catch (boost::filesystem::filesystem_error &ex)
      {
        ROS_ERROR_STREAM("Could not create subdir for calibration data: " << save_dir.string());
        save_subdir_ = "";
        return;
      }
    }

    boost::format left_image_name = boost::format("capture_%03d_left.jpg") % scene_idx;
    path left_image_path = path(data_directory_) / save_subdir_ / left_image_name.str();
    writeImage(left_image, left_image_path);

    boost::format right_image_name = boost::format("capture_%03d_right.jpg") % scene_idx;
    path right_image_path = path(data_directory_) / save_subdir_ / right_image_name.str();
    writeImage(right_image, right_image_path);

    boost::format transform_file_name = boost::format("capture_%03d_transform.yaml") % scene_idx;
    path transform_path = path(data_directory_) / save_subdir_ / transform_file_name.str();
    writeTransformYAML(transforms, transform_path);
  }

  /**
   * \brief Creates and sets a new timestamp-based subdirectory to store calibration data.
   */
  void setNewSaveSubdir(const std::string &base_dir)
  {
    auto t = std::time(nullptr);
    auto tm = std::localtime(&t);
    std::ostringstream ss;
    ss << std::put_time(tm, "%y%m%d_%H%M%S");
    save_subdir_ = ss.str();

    //This directory gets created only when we need to write the first observation into it
    path data_dir = path(base_dir) / save_subdir_;
    ROS_INFO_STREAM("Storing calibration info in new directory: " << data_dir.string());
  }

  void loadImage(path &file, sensor_msgs::Image &output)
  {
    ROS_INFO_STREAM("Loading image from file: " << file);
    cv_bridge::CvImage::Ptr cv_img(new cv_bridge::CvImage);
    cv_img->image = cv::imread(file.string(), CV_LOAD_IMAGE_GRAYSCALE);
    cv_img->encoding = "mono8";
    cv_img->toImageMsg(output);
  }

  void writeImage(const sensor_msgs::Image &img, path &file)
  {
    cv_bridge::CvImage::Ptr cv_img = cv_bridge::toCvCopy(img, "mono8");
    cv::imwrite(file.string(), cv_img->image);
  }

  bool loadTransformYAML(path &file, std::map<std::string, Pose6d> &transforms)
  {
    ROS_INFO_STREAM("Loading yaml from file: " << file);
    auto load_transform_from_yaml = [](const YAML::Node &n) -> Pose6d
    {
      Pose6d t;
      t.x = n["x"].as<double>();
      t.y = n["y"].as<double>();
      t.z = n["z"].as<double>();
      t.ax = n["ax"].as<double>();
      t.ay = n["ay"].as<double>();
      t.az = n["az"].as<double>();
      return t;
    };

    try
    {
      YAML::Node n = YAML::LoadFile(file.string());
      for (auto it = n.begin(); it != n.end(); ++it)
      {
        transforms[it->first.as<std::string>()] = load_transform_from_yaml(it->second);
      }
    }
    catch (YAML::Exception &ex)
    {
      ROS_ERROR_STREAM("YAML exception: " << ex.what());
      return false;
    }
    return true;
  }

  void writeTransformYAML(const std::map<std::string, Pose6d> &transforms, path &file)
  {
    std::ofstream outfile(file.string());
    auto write_transform = [&](const std::string &name, const Pose6d &transform)
    {
      outfile << name << ":" << std::endl;
      outfile << "  x: " << transform.x << std::endl;
      outfile << "  y: " << transform.y << std::endl;
      outfile << "  z: " << transform.z << std::endl;
      outfile << "  ax: " << transform.ax << std::endl;
      outfile << "  ay: " << transform.ay << std::endl;
      outfile << "  az: " << transform.az << std::endl;
    };

    for (auto it = transforms.begin(); it != transforms.end(); ++it)
      write_transform(it->first, it->second);
    outfile.close();
  }

  //construct image file from scene name
  std::string current_image_file (int scene, std::string& filename){
    std::string present_image_file_path_name;
    char scene_chars[8];
    sprintf(scene_chars,"_%03d.jpg",scene);
    if(filename == ""){ // build file name from image_directory_,
      present_image_file_path_name  = image_directory_ + std::string("/") +  camera_name_ + std::string(scene_chars);
    }
    else{
      present_image_file_path_name  = image_directory_ + "/" +  filename;
    }
    return present_image_file_path_name;
  }; // end of current_image_file()


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
