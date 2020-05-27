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

class gantryCalServiceNode
{
public:
  gantryCalServiceNode(const  ros::NodeHandle& nh)
    :nh_(nh), P_(NULL), problem_initialized_(false), total_observations_(0), scene_(0)
  {
    std::string nn = ros::this_node::getName();
    ros::NodeHandle priv_nh("~");
    
    // load global prameters
    if(!priv_nh.getParam("camera_file_name", camera_file_)){
      ROS_ERROR("Must set param: camera_file, this defines the camera to calibrate");
    }
    if(!priv_nh.getParam("target_file_name", target_file_)){
      ROS_ERROR("Must set param: target_file, this defines the target used for calibration");
    }
    // load frame parameters
    if(!priv_nh.getParam("robot_mount_frame", robot_mount_frame_)){
      ROS_ERROR("Must set param: robot_mount_frame, this defines the tf-frame on which the robot base is mounted");
    }
    if(!priv_nh.getParam("robot_base_frame", robot_base_frame_)){
      ROS_ERROR("Must set param: robot_base_frame, this defines the tf-frame the robot base");
    }


    priv_nh.getParam("save_data", save_data_);
    priv_nh.getParam("data_directory", data_directory_);
    
    ROS_INFO("camera_file: %s",  camera_file_.c_str());
    ROS_INFO("target_file: %s",  target_file_.c_str());
    if(save_data_){
      ROS_INFO("saving data to %s", data_directory_.c_str());
    }
    else{
      ROS_WARN("Not saving data for gantry calibration");
    }

    // Load cameras from yaml file. There should only be one static camera
    if(!load_camera()){
      ROS_ERROR("can't load the camera from %s", camera_file_.c_str());
      exit(1);
    }
    else{ // get camera's optical frame for a transform listener
      camera_optical_frame_ = all_cameras_[0]->transform_interface_->getTransformFrame();    
      fx_ = all_cameras_[0]->camera_parameters_.focal_length_x;
      fy_ = all_cameras_[0]->camera_parameters_.focal_length_y;
      cx_ = all_cameras_[0]->camera_parameters_.center_x;
      cy_ = all_cameras_[0]->camera_parameters_.center_y;
      roi_.x_min = 0;
      roi_.y_min = 0;
      roi_.x_max = all_cameras_[0]->camera_parameters_.width;
      roi_.y_max = all_cameras_[0]->camera_parameters_.height;

      ROS_INFO("loaded camera from %s with optical_frame = %s intrinsics:[ %f %f %f %f ]", camera_file_.c_str(),
	       camera_optical_frame_.c_str(),
	       fx_, fy_, cx_, cy_);
    }
    
    // Load targets from yaml file. There should only be one static target
    if(!load_target()){
      ROS_ERROR("can't load the target from %s", target_file_.c_str());
      exit(1);
    }


    // initialize the target to robot_mount transform listener
    target_to_robotm_TI_ = new industrial_extrinsic_cal::ROSListenerTransInterface(all_targets_[0]->target_frame_);
    target_to_robotm_TI_->setReferenceFrame(robot_mount_frame_);
    target_to_robotm_TI_->setDataDirectory(data_directory_);

    // initialize the target to robot_mount transform listener
    robot_base_to_optical_TI_ = new industrial_extrinsic_cal::ROSListenerTransInterface(robot_base_frame_);
    robot_base_to_optical_TI_->setReferenceFrame(camera_optical_frame_);
    robot_base_to_optical_TI_->setDataDirectory(data_directory_);

    // advertise services
    start_server_       = nh_.advertiseService( "GantryCalSrvStart",  &gantryCalServiceNode::startCallBack, this);
    load_server_        = nh_.advertiseService( "GantryCalSrvLoad",   &gantryCalServiceNode::loadCallBack, this);
    observation_server_ = nh_.advertiseService( "GantryCalSrvObs",    &gantryCalServiceNode::observationCallBack, this);
    run_server_         = nh_.advertiseService( "GantryCalSrvRun",    &gantryCalServiceNode::runCallBack, this);
    save_server_        = nh_.advertiseService( "GantryCalSrvSave",   &gantryCalServiceNode::saveCallBack, this);
    covariance_server_  = nh_.advertiseService( "GantryCalSrvCov",    &gantryCalServiceNode::covCallBack, this);

  };// end of constructor


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
      all_cameras_[i]->transform_interface_->setReferenceFrame(robot_mount_frame_);
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
    res.message = std::string("Gantry calibration service started");
    res.success = true;
    return(true);
  }

  // called to collect observations for the current pose of the scene
  bool observationCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    char msg[100];
    Pose6d TtoRm = target_to_robotm_TI_->pullTransform();
    Pose6d RbtoO = robot_base_to_optical_TI_->pullTransform();
    TtoRm.show("Target_2 Robot_Mount");
    RbtoO.show("Robot_base_2_Optical");
    
    if(problem_initialized_ != true ){
      std_srvs::TriggerRequest  sreq;
      std_srvs::TriggerResponse sres;
      ROS_INFO("Problem not yet initialized, calling start service");
      startCallBack(sreq, sres);
    }

    // get observations
    all_cameras_[0]->camera_observer_->clearTargets();
    all_cameras_[0]->camera_observer_->clearObservations();
    industrial_extrinsic_cal::Cost_function cost_type = industrial_extrinsic_cal::cost_functions::GantryCal;
    int total_pts=0;
    // add target to camera observer
    all_cameras_[0]->camera_observer_->addTarget(all_targets_[0], roi_, cost_type);
    total_pts += all_targets_[0]->num_points_;
    all_cameras_[0]->camera_observer_->triggerCamera();
    while (!all_cameras_[0]->camera_observer_->observationsDone());
    CameraObservations camera_observations;
    all_cameras_[0]->camera_observer_->getObservations(camera_observations);
    int num_observations = (int)camera_observations.size();
    ROS_INFO("Found %d observations", (int)camera_observations.size());
	
    // add observations to problem
    num_observations = (int)camera_observations.size();
    if (num_observations != total_pts)
      {
	ROS_ERROR("Target Locator could not find all targets found %d out of %d", num_observations, total_pts);
      }
    else if(!all_cameras_[0]->camera_observer_->checkObservationProclivity(camera_observations))
      {
	ROS_ERROR("Proclivities Check not successful");
      }
    else
      {	 // add a new cost to the problem for each observation
	total_observations_ += num_observations;
	for (int k = 0; k < num_observations; k++)
	  {
	    shared_ptr<Target> target = camera_observations[k].target;
	    double image_x = camera_observations[k].image_loc_x;
	    double image_y = camera_observations[k].image_loc_y;
	    Point3d point  = target->pts_[camera_observations[k].point_id];
	    CostFunction* cost_function = industrial_extrinsic_cal::GantryCal::Create(image_x, image_y, fx_, fy_, cx_, cy_, TtoRm, RbtoO, point);
	    P_->AddResidualBlock(cost_function, NULL, gantrycal_pose_.pb_pose);
	  }  // for each observation at this camera_location
      } // end of else (there are some observations to add)
    

    if (save_data_){
      char pose_scene_chars[20];
      char image_scene_chars[20];
      sprintf(pose_scene_chars,"_%03d.yaml",scene_);
      sprintf(image_scene_chars,"_%03d.jpg",scene_);
      std::string image_file = all_cameras_[0]->camera_name_ + std::string(image_scene_chars);
      std::string target_to_robot_mount = std::string("t_to_rm") + std::string(pose_scene_chars);  // save2 to data_directory_/t_to_rm_sceneID.yaml
      std::string robot_base_to_optical = std::string("rb_to_o") + std::string(pose_scene_chars);  // save2 to data_directory_/t_to_rm_sceneID.yaml
      all_cameras_[0]->camera_observer_->save_current_image(scene_,image_file);
      target_to_robotm_TI_->saveCurrentPose(scene_,target_to_robot_mount);
      robot_base_to_optical_TI_->saveCurrentPose(scene_,robot_base_to_optical);
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
    std::string covariance_file_name = data_directory_ + "/gantry_cov.txt";
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
	
	std::vector<std::pair<const double*, const double*> > covariance_pairs;
	covariance_pairs.push_back(std::make_pair(gantrycal_pose_.pb_pose, gantrycal_pose_.pb_pose));
	
	if(covariance.Compute(covariance_pairs, P_))
	  {
	    fprintf(fp, "covariance blocks:\n");
	    double cov[6*6];

	    if(covariance.GetCovarianceBlock(gantrycal_pose_.pb_pose, gantrycal_pose_.pb_pose, cov))
	      {
		fprintf(fp, "cov is 6x6\n");
		for(int i=0;i<6;i++)
		  {
		    double sigma_i = sqrt(fabs(cov[i*6+i]));
		    for(int j=0;j<6;j++)
		      {
			double sigma_j = sqrt(fabs(cov[j*6+j]));
			double value;
			if(i==j)
			  {
			    value = sigma_i;
			  } // end if i==j
			else
			  {
			    if(sigma_i==0) sigma_i = 1;
			    if(sigma_j==0) sigma_j = 1;
			    value = cov[i * 6 + j]/(sigma_i*sigma_j);
			  } // end i!=j
			fprintf(fp, "%16.5f ", value);
		      }  // end of j loop
		    fprintf(fp, "\n");
		  }  // end of i loop
	      }// end if covariance could be gotten
	    else{
	      ROS_ERROR("could not get covariance");
	      fclose(fp);
	      return false;
	    }
	  }// end if covariance could be computed
	else
	  {
	    ROS_ERROR("covariance file could not be opened");
	    return false;
	  }
      }// end if cov file could open
    fclose(fp);
    return true;
  }
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

    // output initial conditions
    gantrycal_pose_.show("initial gantrycal extrinsics");

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
	    ROS_INFO("Gantry calibration was successful");
	  }
	else
	  {
	    ROS_ERROR("allowable cost exceeded %f > %f", final_cost, req.allowable_cost_per_observation);
	    sprintf(msg, "allowable cost exceeded %f > %f", final_cost, req.allowable_cost_per_observation);
	    gantrycal_pose_.show("final gantrycal pose");
	    res.message = std::string(msg);
	    res.success = false;
	    return(true);
	  }
	sprintf(msg, "final cost %f", final_cost);
	res.message = std::string(msg);
	res.success = true;
	return(true);
      }
    ROS_ERROR("Gantry Cal NO CONVERGENCE");
    sprintf(msg, "Gantry Cal NO CONVERGENCE");
    res.message = std::string(msg);
    res.success = false;
    return(true);
  }; // end runCallBack()

  bool saveCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    ROS_ERROR("it is expected that the pose computed by gantrycal is part of a urdf, please copy this info there");
    gantrycal_pose_.show("final gantrycal pose");
    res.success = true;
    return(true);
  }// end saveCallback()

  /**
   * \brief Load and set the calibration data from a previous job.
   */
  bool loadCallBack(industrial_extrinsic_cal::FileOp::Request &req, industrial_extrinsic_cal::FileOp::Response &resp)
  {
    industrial_extrinsic_cal::Cost_function cost_type = industrial_extrinsic_cal::cost_functions::CameraReprjErrorWithDistortionPK;
    int total_pts= all_targets_[0]->num_points_;

    if(problem_initialized_ != true ){ // scene_id=0, problem re-initialized
      ROS_INFO("calling start");
      std_srvs::TriggerRequest  sreq;
      std_srvs::TriggerResponse sres;
      startCallBack(sreq,sres);
    }
    
    bool data_read_ok=true;
    while(data_read_ok){
      char pose_scene_chars[20];
      char image_scene_chars[20];
      sprintf(pose_scene_chars,"_%03d.yaml",scene_);
      sprintf(image_scene_chars,"_%03d.jpg",scene_);
      std::string image_file = all_cameras_[0]->camera_name_ + std::string(image_scene_chars);
      std::string target_to_robot_mount = std::string("t_to_rm") + std::string(pose_scene_chars);  
      std::string robot_base_to_optical = std::string("rb_to_o") + std::string(pose_scene_chars);  
      data_read_ok &= all_cameras_[0]->camera_observer_->load_current_image(scene_,image_file);
      data_read_ok &= target_to_robotm_TI_->loadPose(scene_,target_to_robot_mount);
      data_read_ok &= robot_base_to_optical_TI_->loadPose(scene_,robot_base_to_optical);
      
      if(data_read_ok){
	Pose6d TtoRm = target_to_robotm_TI_->getCurrentPose();
	Pose6d RbtoO = robot_base_to_optical_TI_->getCurrentPose();
	TtoRm.show("TtoRm");
	RbtoO.show("RbtoO");
	
	// set target and get observations from the image
	CameraObservations camera_observations;
	all_cameras_[0]->camera_observer_->clearTargets();
	all_cameras_[0]->camera_observer_->clearObservations();
	all_cameras_[0]->camera_observer_->addTarget(all_targets_[0], roi_, cost_type);
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
	    for (int k = 0; k < num_observations; k++)
	      {
		shared_ptr<Target> target = camera_observations[k].target;
		double image_x = camera_observations[k].image_loc_x;
		double image_y = camera_observations[k].image_loc_y;
		Point3d point  = target->pts_[camera_observations[k].point_id];
		CostFunction* cost_function = industrial_extrinsic_cal::GantryCal::Create(image_x, image_y, fx_, fy_, cx_, cy_, TtoRm, RbtoO, point);
		P_->AddResidualBlock(cost_function, NULL, gantrycal_pose_.pb_pose);
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
  std::string robot_mount_frame_;        // frame where the robot is mounted
  std::string robot_base_frame_;         // robot base frame
  std::string camera_optical_frame_;


  ros::ServiceServer start_server_;
  ros::ServiceServer observation_server_;
  ros::ServiceServer run_server_;
  ros::ServiceServer save_server_;
  ros::ServiceServer load_server_;
  ros::ServiceServer covariance_server_;
  Pose6d robotm_to_robotb_pose;
  Pose6d gantrycal_pose_;
  ceres::Problem *P_;
  bool problem_initialized_;

  bool save_data_;
  std::string data_directory_;

  int total_observations_;
  int scene_;
  industrial_extrinsic_cal::ROSListenerTransInterface *target_to_robotm_TI_;
  industrial_extrinsic_cal::ROSListenerTransInterface *robot_base_to_optical_TI_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  Roi roi_;

};// end of class gantryCalServiceNode

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gantry_cal_service");
  ros::NodeHandle node_handle;
  gantryCalServiceNode WCSN(node_handle);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
