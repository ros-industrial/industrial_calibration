#include <ros/ros.h>

#include <string>
#include <fstream>

// ros includes
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <std_srvs/Trigger.h>
#include <dynamic_reconfigure/server.h>
#include <intrinsic_cal/observeConfig.h>

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
#include <industrial_extrinsic_cal/camera_definition.h>
#include <robo_cylinder/HomeCmd.h>
#include <robo_cylinder/MoveMeters.h>
#include <robo_cylinder/MovePulses.h>
#include <robo_cylinder/PowerIO.h>
#include <robo_cylinder/StatusUpdate.h>
#include <robo_cylinder/VelAcc.h>

// boost includes
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

// opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// other includes
#include <sys/stat.h>


using boost::shared_ptr;
using ceres::Solver;
using ceres::CostFunction;
using industrial_extrinsic_cal::Camera;
using industrial_extrinsic_cal::Target;
using industrial_extrinsic_cal::Roi;
using industrial_extrinsic_cal::CameraObservations;
using industrial_extrinsic_cal::Point3d;
using industrial_extrinsic_cal::Pose6d;
using industrial_extrinsic_cal::P_BLOCK;
using industrial_extrinsic_cal::CameraParameters;
using industrial_extrinsic_cal::NoWaitTrigger;
using industrial_extrinsic_cal::ROSCameraObserver;
using std::string;

class robocyl_ical_8D
{
public:
  void dynReConfCallBack(intrinsic_cal::observeConfig& config, uint32_t level)
  {
    // get ready for taking images
    camera_->camera_observer_->clearTargets();
    camera_->camera_observer_->clearObservations();
    
    Roi roi;
    roi.x_min = 0;
    roi.y_min = 0;
    roi.x_max = image_width_;
    roi.y_max = image_height_;

    industrial_extrinsic_cal::Cost_function cost_type = industrial_extrinsic_cal::cost_functions::RailICal5;
    
    camera_->camera_observer_->addTarget(target_, roi, cost_type);
    // collect images/observations
    camera_->camera_observer_->triggerCamera();
    while (!camera_->camera_observer_->observationsDone());
    CameraObservations camera_observations;
    camera_->camera_observer_->getObservations(camera_observations);
    ROS_ERROR("done");
  }

  robocyl_ical_8D(const  ros::NodeHandle& nh)
    :nh_(nh), P_(NULL), problem_initialized_(false), total_observations_(0), scene_(0)
  {
    ros::NodeHandle priv_nh("~");

    // load parameters pertaining to camera and target
    if(!priv_nh.getParam( "target_rows", target_rows_)){
      ROS_ERROR("Must set param:  target_rows");
    }
    if(!priv_nh.getParam( "target_cols", target_cols_)){
      ROS_ERROR("Must set param:  target_cols");
    }
    if(!priv_nh.getParam( "target_circle_dia", circle_diameter_)){
      ROS_ERROR("Must set param:  target_circle_dia");
    }
    if(!priv_nh.getParam( "target_spacing", circle_spacing_)){
      ROS_ERROR("Must set param:  target_spacing");
    }
    if(!priv_nh.getParam( "camera_name", camera_name_)){
      ROS_ERROR("Must set param:  camera_name_");
    }
    if(!priv_nh.getParam( "image_topic", image_topic_)){
      ROS_ERROR("Must set param:  image_topic_");
    }
    if(!priv_nh.getParam( "image_height", image_height_)){
      ROS_ERROR("Must set param:  image_height_");
    }
    if(!priv_nh.getParam( "image_width", image_width_)){
      ROS_ERROR("Must set param:  image_width_");
    }
    save_data_ = false;
    priv_nh.getParam( "save_data", save_data_);
    if(save_data_){
      if(!priv_nh.getParam("image_directory",image_directory_)){
	ROS_ERROR("save_data set true, but no image_directory defined");
      }
      else{
	ROS_ERROR("robocyl_ical_8D is saving image and rail position data in image_directory %s", image_directory_.c_str());
      }
    }
    bool is_moving = true;
    camera_ =  shared_ptr<industrial_extrinsic_cal::Camera>(new industrial_extrinsic_cal::Camera("my_camera", camera_parameters_, is_moving));
    camera_->trigger_ = shared_ptr<NoWaitTrigger>(new NoWaitTrigger());
    camera_->camera_observer_ = shared_ptr<ROSCameraObserver>(new ROSCameraObserver(image_topic_, camera_name_));
    if(save_data_) camera_->camera_observer_->set_image_directory(image_directory_);
    sleep(5); // wait for camera to come up or else this will fail
    if(!camera_->camera_observer_->pullCameraInfo(camera_->camera_parameters_.focal_length_x,
						  camera_->camera_parameters_.focal_length_y,
						  camera_->camera_parameters_.center_x,
						  camera_->camera_parameters_.center_y,
						  camera_->camera_parameters_.distortion_k1,
						  camera_->camera_parameters_.distortion_k2,
						  camera_->camera_parameters_.distortion_k3,
						  camera_->camera_parameters_.distortion_p1,
						  camera_->camera_parameters_.distortion_p2,
						  image_width_, image_height_))  {
      ROS_FATAL("Could not get camera information for %s from topic %s. Shutting down node.", camera_name_.c_str(), image_topic_.c_str());
      ros::shutdown();
    }
    ROS_INFO("initial camera info focal:%f %f center:%f %f  radial:%f %f %f tang: %f %f",
	     camera_->camera_parameters_.focal_length_x,
	     camera_->camera_parameters_.focal_length_y,
	     camera_->camera_parameters_.center_x,
	     camera_->camera_parameters_.center_y,
	     camera_->camera_parameters_.distortion_k1,
	     camera_->camera_parameters_.distortion_k2,
	     camera_->camera_parameters_.distortion_k3,
	     camera_->camera_parameters_.distortion_p1,
	     camera_->camera_parameters_.distortion_p2);
    
    initMCircleTarget(target_rows_, target_cols_, circle_diameter_, circle_spacing_);
    
    // get rail specific parameters
    if(!priv_nh.getParam( "num_camera_locations", num_camera_locations_)){
      ROS_ERROR("Must set param:  num_camera_locations");
    }
    if(!priv_nh.getParam( "camera_spacing", camera_spacing_)){
      ROS_ERROR("Must set param:  camera_spacing");
    }
    if(!priv_nh.getParam( "rail_start_position", rail_start_)){
      ROS_ERROR("Must set param:  target_to_rail_distance");
    }
    string move_client_name("/move_meters");
    if(!priv_nh.getParam( "move_client", move_client_name)){
      ROS_WARN("move_client = %s", move_client_name.c_str());
    }
    string power_client_name("/power_io");
    if(!priv_nh.getParam( "power_client", power_client_name)){
      ROS_WARN("power_client = %s", power_client_name.c_str());
    }
    string home_client_name("/home");
    if(!priv_nh.getParam( "home_client", home_client_name)){
      ROS_WARN("home_client = %s", home_client_name.c_str());
    }
    
    // initialize stage power it on, and send it home
    move_client_   = nh_.serviceClient<robo_cylinder::MoveMeters>(move_client_name);
    power_client_  = nh_.serviceClient<robo_cylinder::PowerIO>(power_client_name);
    home_client_   = nh_.serviceClient<robo_cylinder::HomeCmd>(home_client_name);
    pio_request_.io = 1;
    power_client_.call(pio_request_, pio_response_);
    home_client_.call(hc_request_, hc_response_);

    // advertise services
    start_server_       = nh_.advertiseService( "RailICalSrvStart", &robocyl_ical_8D::startCallBack, this);
    load_server_        = nh_.advertiseService( "RailICalSrvLoad",  &robocyl_ical_8D::loadCallBack, this);
    observation_server_ = nh_.advertiseService( "RailICalSrvObs",   &robocyl_ical_8D::observationCallBack, this);
    run_server_         = nh_.advertiseService( "RailICalSrvRun",   &robocyl_ical_8D::runCallBack, this);
    save_server_        = nh_.advertiseService( "RailICalSrvSave",  &robocyl_ical_8D::saveCallBack, this);
    covariance_server_  = nh_.advertiseService( "RailICalSrvCov",   &robocyl_ical_8D::covCallBack, this);


    std::string recon_node_name = "~/observe";
    pnh_ = ros::NodeHandle(recon_node_name.c_str());
    server_.reset(new dynamic_reconfigure::Server<intrinsic_cal::observeConfig>(pnh_));

    dynamic_reconfigure::Server<intrinsic_cal::observeConfig>::CallbackType f;
    
    f = boost::bind(&robocyl_ical_8D::dynReConfCallBack, this, _1, _2);
    server_->setCallback(f);
  };// end of constructor


  // initialize all the points in the target 
  void initMCircleTarget(int rows, int cols, double circle_dia, double spacing)
  {
    target_ =  shared_ptr<industrial_extrinsic_cal::Target>(new industrial_extrinsic_cal::Target());
    target_->is_moving_ = true;
    target_->target_name_ = "modified_circle_target";
    target_->target_frame_ = "target_frame";
    target_->target_type_ =  pattern_options::ModifiedCircleGrid;
    target_->circle_grid_parameters_.pattern_rows = rows;
    target_->circle_grid_parameters_.pattern_cols = cols;
    target_->circle_grid_parameters_.spacing      = spacing;
    target_->circle_grid_parameters_.circle_diameter = circle_dia;
    target_->circle_grid_parameters_.is_symmetric = true; 
    // create a grid of points
    target_->pose_.setOrigin(0,0,.8);
    tf::Matrix3x3 R(-1, 0, 0, 0, 1, 0, 0, 0, -1);
    target_->pose_.setBasis(R);
    target_->num_points_ = rows*cols;
    target_->generatePoints();
  };// end initMCircleTarget
  

    // callback functions
  bool startCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    if(problem_initialized_)	delete(P_);
    P_ = new ceres::Problem;
    target_to_camera_poses.clear();
    target_to_camera_poses.reserve(100);
    problem_initialized_ = true;
    total_observations_  = 0;
    scene_=0;
    ax_ay_[0] = 0.0;// ideally the motion of rail is along z axis of camera
    ax_ay_[1] = 0.0; 
    res.message = string("Intrinsic calibration service started");
    res.success = true;
    return(true);
  }

  // called to load previously collected observations
  bool loadCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    char msg[100];
    industrial_extrinsic_cal::Cost_function cost_type = industrial_extrinsic_cal::cost_functions::RailICal5;

    char current_image_scene_chars[255];
    sprintf(current_image_scene_chars,"_%03d_%03d.jpg",scene_,0); // add rail distance index to image, scene_ will automatically be added by camera_observer.save_current_image()
    std::string image_file_currently = camera_->camera_name_ + std::string(current_image_scene_chars);
    std::string image_file_now = current_image_file(scene_,image_file_currently);

    while(camera_->camera_observer_->exists_test(image_file_now)){
      //reset everything for each individual scene
      if(problem_initialized_ != true ){
        ROS_INFO("calling start");
        std_srvs::TriggerRequest  sreq;
        std_srvs::TriggerResponse sres;
        startCallBack(sreq,sres);
      }
      // add a new target pose for each scene_, initialized to something reasonable for our standard setup
      Pose6d P;
      P.setOrigin(0,0,.8);
      tf::Matrix3x3 R(-1, 0, 0, 0, 1, 0, 0, 0, -1);
      P.setBasis(R);
      target_to_camera_poses.push_back(P);

      // iether creates a place to hold all usefull information about images or groups all of the valuable information into p_blocks
      P_BLOCK intrinsics = camera_->camera_parameters_.pb_intrinsics;
      P_BLOCK extrinsics = target_to_camera_poses[scene_].pb_pose;
      for(int i = 0; i<(num_camera_locations_);i++){
        // todo load files before you preform observations
        char image_scene_chars[255];
        sprintf(image_scene_chars,"_%03d_%03d.jpg",scene_,i); // add rail distance index to image, scene_ will automatically be added by camera_observer.save_current_image()
        std::string image_file = camera_->camera_name_ + std::string(image_scene_chars);

        camera_->camera_observer_->load_current_image(scene_,image_file);

        double Dist = i*camera_spacing_;
        // set the roi to the whole image
        Roi roi;
        roi.x_min = 0;
        roi.y_min = 0;
        roi.x_max = image_width_;
        roi.y_max = image_height_;


        // get ready for taking Observations
        camera_->camera_observer_->clearTargets();
        camera_->camera_observer_->clearObservations();

        //creates a target in region of interest and adds up points in target
        int total_pts=0;
        camera_->camera_observer_->addTarget(target_, roi, cost_type);
        total_pts += target_->num_points_;


        // preform observations
        CameraObservations camera_observations;

        camera_->camera_observer_->getObservations(camera_observations);
        int num_observations = (int)camera_observations.size();
        ROS_INFO("Found %d observations", (int)camera_observations.size());

        // add observations to problem
        num_observations = (int)camera_observations.size();
        if (num_observations != total_pts)
        {
          ROS_ERROR("Target Locator could not find all targets found %d out of %d", num_observations, total_pts);
        }
        else if(!camera_->camera_observer_->checkObservationProclivity(camera_observations))
        {
          ROS_ERROR("Proclivities Check not successful");
        }
        else
        {
          // add a new cost to the problem for each observation
          total_observations_ += num_observations;
          for (int k = 0; k < num_observations; k++)
          {
            double image_x = camera_observations[k].image_loc_x;
            double image_y = camera_observations[k].image_loc_y;
            Point3d point  = camera_observations[k].target->pts_[camera_observations[k].point_id];
            CostFunction *cost_function = industrial_extrinsic_cal::RailICal5::Create(image_x, image_y, Dist, point);
            P_->AddResidualBlock(cost_function, NULL, intrinsics, extrinsics, ax_ay_);
          }  // for each observation at this camera_location
        } // end of else (there are some observations to add)

      }
      ROS_INFO("now have %d observations after scene %d",total_observations_, scene_);
      scene_++;
      sprintf(current_image_scene_chars,"_%03d_%03d.jpg",scene_,0); // add rail distance index to image, scene_ will automatically be added by camera_observer.save_current_image()
      image_file_currently = camera_->camera_name_ + std::string(current_image_scene_chars);
      image_file_now = current_image_file(scene_,image_file_currently);
    }
    sprintf(msg, "RailIcal_srv now has %d observations after scene %d",total_observations_, scene_);

    res.message = string(msg);
    res.success = true;

    return(true);
  }; // end of load service};
  
  // called to collect observations for the current pose of the scene
  bool observationCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    char msg[100];
    industrial_extrinsic_cal::Cost_function cost_type = industrial_extrinsic_cal::cost_functions::RailICal5;

    if(problem_initialized_ != true ){
      ROS_INFO("calling start");
      std_srvs::TriggerRequest  sreq;
      std_srvs::TriggerResponse sres;
      startCallBack(sreq,sres);
    }

    // add a new target pose for each scene_, initialized to something reasonable for our standard setup
    Pose6d P;
    P.setOrigin(0,0,.8);
    tf::Matrix3x3 R(-1, 0, 0, 0, 1, 0, 0, 0, -1);
    P.setBasis(R);
    target_to_camera_poses.push_back(P);

    P_BLOCK intrinsics = camera_->camera_parameters_.pb_intrinsics;
    P_BLOCK extrinsics = target_to_camera_poses[scene_].pb_pose;

    // This is a bit abnormal,
    // a single scene has a sequence of camera locations
    // it is expected that the initial pose at the first pose is changed each scene
    // therefore, the target   , new_image_collected_(false)must be defined to be "moving"
    // We get a new set of extrinsic parameters for the target with each call to observationCallBack()
    for(int i=0; i<num_camera_locations_; i++){
      
      // move the rail into position, and wait a bit for any vibrations to settle
      double Dist = i*camera_spacing_;
      ROS_INFO("moving to %lf",0.8 - Dist);
      mm_request_.meters = 0.8 - Dist;// note the length of stage is 0.8, so this moves from front to back
      
      move_client_.call(mm_request_, mm_response_); // this call blocks until camera is moved
      sleep(2.0); // wait for oscillations to die out
      
      // set the roi to the whole image
      Roi roi;
      roi.x_min = 0;
      roi.y_min = 0;
      roi.x_max = image_width_;
      roi.y_max = image_height_;
      
      // get ready for taking images
      camera_->camera_observer_->clearTargets();
      camera_->camera_observer_->clearObservations();

      int total_pts=0;
      camera_->camera_observer_->addTarget(target_, roi, cost_type);
      total_pts += target_->num_points_;
      
      // collect images/observations
      camera_->camera_observer_->triggerCamera();
      while (!camera_->camera_observer_->observationsDone());
      CameraObservations camera_observations;
      camera_->camera_observer_->getObservations(camera_observations);
      int num_observations = (int)camera_observations.size();
      ROS_INFO("Found %d observations", (int)camera_observations.size());
      
      // add observations to problem
      num_observations = (int)camera_observations.size();
      if (save_data_){
        char image_scene_chars[255];
        sprintf(image_scene_chars,"_%03d_%03d.jpg",scene_,i); // add rail distance index to image, scene_ will automatically be added by camera_observer.save_current_image()
        std::string image_file = camera_->camera_name_ + std::string(image_scene_chars);
        camera_->camera_observer_->save_current_image(scene_,image_file);
      }// end if save_data_
      if (num_observations != total_pts)
      {
        ROS_ERROR("Target Locator could not find all targets found %d out of %d", num_observations, total_pts);
      }
      else if(!camera_->camera_observer_->checkObservationProclivity(camera_observations))
      {
        ROS_ERROR("Proclivities Check not successful");
      }
      else
      {
        // add a new cost to the problem for each observation
        total_observations_ += num_observations;
        for (int k = 0; k < num_observations; k++)
        {
          double image_x = camera_observations[k].image_loc_x;
          double image_y = camera_observations[k].image_loc_y;
          Point3d point  = camera_observations[k].target->pts_[camera_observations[k].point_id];
          if(k==10) ROS_ERROR("target point %d = %8.3lf %8.3lf %8.3lf observed at %8.3lf %8.3lf",camera_observations[k].point_id, point.x, point.y, point.z, image_x, image_y);
          CostFunction *cost_function = industrial_extrinsic_cal::RailICal5::Create(image_x, image_y, Dist, point);
          P_->AddResidualBlock(cost_function, NULL, intrinsics, extrinsics, ax_ay_);
        }  // for each observation at this camera_location
      } // end of else (there are some observations to add)
    }// for each linear rail position

    ROS_INFO("now have %d observations after scene %d",total_observations_, scene_);
    scene_++;

    // move back to start so that camera may be adjusted where it is most critial
    mm_request_.meters = 0.8;// note the length of stage is 0.8, so this moves from front to back
    move_client_.call(mm_request_, mm_response_); // this call blocks until camera is moved

    sprintf(msg, "RailIcal_srv now has %d observations after scene %d",total_observations_, scene_);

    res.message = string(msg);
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
    for(int i=0;i<target_to_camera_poses.size();i++){
      sprintf(msg,"initial target_pose %d",i);
      target_to_camera_poses[i].show(msg);
    }

    ROS_INFO("intial camera_matrix data: [ %lf, 0.0, %lf, 0.0, %lf, %lf, 0.0, 0.0, 1.0]",
	     camera_->camera_parameters_.focal_length_x, camera_->camera_parameters_.center_x,
	     camera_->camera_parameters_.focal_length_y, camera_->camera_parameters_.center_y);
    ROS_INFO("initial distortion data: [ %lf,  %lf,  %lf,  %lf,  %lf]", camera_->camera_parameters_.distortion_k1,
	     camera_->camera_parameters_.distortion_k2, camera_->camera_parameters_.distortion_p1,
	     camera_->camera_parameters_.distortion_p2, camera_->camera_parameters_.distortion_k3);
    ROS_INFO("initial rotation of axis of motion %lf %lf", ax_ay_[0], ax_ay_[1]);


    Solver::Options options;
    Solver::Summary summary;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 1000;
    ceres::Solve(options, P_, &summary);
    if (summary.termination_type != ceres::NO_CONVERGENCE)
      {
	double initial_cost = sqrt( 2*summary.initial_cost / total_observations_);
	double final_cost   = sqrt( 2*summary.final_cost   / total_observations_);

	ROS_INFO("Problem solved, initial cost = %lf, final cost = %lf", initial_cost, final_cost);
	for(int i=0;i<target_to_camera_poses.size();i++){
	  sprintf(msg,"final target_pose %d",i);
	  target_to_camera_poses[i].show(msg);
	}

	ROS_INFO("camera_matrix data: [ %lf, 0.0, %lf, 0.0, %lf, %lf, 0.0, 0.0, 1.0]",
		 camera_->camera_parameters_.focal_length_x, camera_->camera_parameters_.center_x,
		 camera_->camera_parameters_.focal_length_y, camera_->camera_parameters_.center_y);
	ROS_INFO("distortion data: [ %lf,  %lf,  %lf,  %lf,  %lf]", camera_->camera_parameters_.distortion_k1,
		 camera_->camera_parameters_.distortion_k2, camera_->camera_parameters_.distortion_p1,
		 camera_->camera_parameters_.distortion_p2, camera_->camera_parameters_.distortion_k3);
	ROS_INFO("projection_matrix data: [ %lf, 0.0, %lf, 0.0, 0.0, %lf, %lf, 0.0, 0.0, 0.0, 1.0, 0.0]",
		 camera_->camera_parameters_.focal_length_x, camera_->camera_parameters_.center_x,
		 camera_->camera_parameters_.focal_length_y, camera_->camera_parameters_.center_y);
	ROS_INFO("final rotation of axis of motion %lf %lf", ax_ay_[0], ax_ay_[1]);
	if (final_cost <= req.allowable_cost_per_observation)
	  {
	    ROS_INFO("calibration was successful");
	  }
	else
	  {
	    res.final_cost_per_observation = final_cost;
	    ROS_ERROR("allowable cost exceeded %f > %f", final_cost, req.allowable_cost_per_observation);
	    sprintf(msg, "allowable cost exceeded %f > %f", final_cost, req.allowable_cost_per_observation);
	    res.message = string(msg);
	    res.success = false;
	    return(true);
	  }
	sprintf(msg, "final cost %f", final_cost);
	res.final_cost_per_observation = final_cost;
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
    camera_->camera_observer_->pushCameraInfo(
					      camera_->camera_parameters_.focal_length_x, camera_->camera_parameters_.focal_length_y,
					      camera_->camera_parameters_.center_x, camera_->camera_parameters_.center_y,
					      camera_->camera_parameters_.distortion_k1, camera_->camera_parameters_.distortion_k2,
					      camera_->camera_parameters_.distortion_k3, camera_->camera_parameters_.distortion_p1,
					      camera_->camera_parameters_.distortion_p2);
    sprintf(msg, "intrinsics: %8.2lf %8.2lf %8.2lf %8.2lf, %5.3lf %5.3lf %5.3lf, %5.3lf %5.3lf",
	    camera_->camera_parameters_.focal_length_x, camera_->camera_parameters_.focal_length_y,
	    camera_->camera_parameters_.center_x, camera_->camera_parameters_.center_y,
	    camera_->camera_parameters_.distortion_k1, camera_->camera_parameters_.distortion_k2,
	    camera_->camera_parameters_.distortion_k3, camera_->camera_parameters_.distortion_p1,
	    camera_->camera_parameters_.distortion_p2);

    res.message = string(msg);
    res.success = true;
    return(true);

  }// end saveCallback()
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

  bool computeCovariance(string& covariance_file_name)
  {
    FILE* fp;
    if ((fp = fopen(covariance_file_name.c_str(), "w")) != NULL)
    {
      ceres::Covariance::Options covariance_options;
      covariance_options.algorithm_type = ceres::DENSE_SVD;
      ceres::Covariance covariance(covariance_options);

      std::vector<std::pair<const double*, const double*> > covariance_pairs;
      covariance_pairs.push_back(std::make_pair(camera_->camera_parameters_.pb_intrinsics, camera_->camera_parameters_.pb_intrinsics));
      covariance_pairs.push_back(std::make_pair(ax_ay_,ax_ay_));
      covariance_pairs.push_back(std::make_pair(camera_->camera_parameters_.pb_intrinsics, ax_ay_));

      if(covariance.Compute(covariance_pairs, P_))
      {
        fprintf(fp, "covariance blocks:\n");
        double cov_in_in[9*9];
        double cov_axis_axis[2*2];
        double cov_in_axis[9*2];
        if(covariance.GetCovarianceBlock(camera_->camera_parameters_.pb_intrinsics, camera_->camera_parameters_.pb_intrinsics, cov_in_in) &&
           covariance.GetCovarianceBlock(ax_ay_, ax_ay_, cov_axis_axis) &&
           covariance.GetCovarianceBlock(camera_->camera_parameters_.pb_intrinsics, ax_ay_, cov_in_axis)){
          fprintf(fp, "cov_in_in is 9x9\n");
          for(int i=0;i<9;i++){
            double sigma_i = sqrt(fabs(cov_in_in[i*9+i]));
            for(int j=0;j<9;j++){
              double sigma_j = sqrt(fabs(cov_in_in[j*9+j]));
              double value;
              if(i==j){
                value = sigma_i;
              }
              else{
                if(sigma_i==0) sigma_i = 1;
                if(sigma_j==0) sigma_j = 1;
                value = cov_in_in[i * 9 + j]/(sigma_i*sigma_j);
              }
              fprintf(fp, "%16.5f ", value);
            }  // end of j loop
            fprintf(fp, "\n");
          }  // end of i loop
          fprintf(fp, "cov_axis_axis is 2x2\n");
          for(int i=0;i<2;i++){
            double sigma_i = sqrt(fabs(cov_axis_axis[i*2+i]));
            for(int j=0;j<2;j++){
              double sigma_j = sqrt(fabs(cov_axis_axis[j*2+j]));
              double value;
              if(i==j){
                value = sigma_i;
              }
              else{
                if(sigma_i==0) sigma_i = 1;
                if(sigma_j==0) sigma_j = 1;
                value = cov_axis_axis[i * 2 + j]/(sigma_i*sigma_j);
              }
              fprintf(fp, "%16.5f ", value);
            }  // end of j loop
            fprintf(fp, "\n");
          }  // end of i loop
          fprintf(fp, "cov_in_axis is 9x2\n");
          for(int i=0;i<9;i++){
            double sigma_i = sqrt(fabs(cov_in_in[i*9+i]));
            for(int j=0;j<2;j++){
              double sigma_j = sqrt(fabs(cov_axis_axis[j*2+j]));
              double value;
              if(sigma_i==0) sigma_i = 1;
              if(sigma_j==0) sigma_j = 1;
              value = cov_in_axis[i * 2 + j]/(sigma_i*sigma_j);
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
  boost::shared_ptr<dynamic_reconfigure::Server<intrinsic_cal::observeConfig> > server_;
  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;
  double ax_ay_[2];                           /*!< The ax and ay parameters defining the axis of motion relative to optical axis */ 
  ros::ServiceServer start_server_;
  ros::ServiceServer load_server_;
  ros::ServiceServer observation_server_;
  ros::ServiceServer run_server_;
  ros::ServiceServer save_server_;
  ros::ServiceServer covariance_server_;
  ceres::Problem *P_;
  bool problem_initialized_;
  int total_observations_;
  int scene_;
  int num_camera_locations_;
  double camera_spacing_;
  double rail_start_;
  bool save_data_;
  string image_directory_;
  robo_cylinder::MoveMeters::Request mm_request_;   /**< request when transform is part of a mutable set */
  robo_cylinder::MoveMeters::Response mm_response_; /**< request when transform is part of a mutable set */
  robo_cylinder::PowerIO::Request pio_request_;     /**< request when transform is part of a mutable set */
  robo_cylinder::PowerIO::Response pio_response_;   /**< request when transform is part of a mutable set */
  robo_cylinder::HomeCmd::Request hc_request_;      /**< request when transform is part of a mutable set */
  robo_cylinder::HomeCmd::Response hc_response_;    /**< request when transform is part of a mutable set */
  // robo-cylinder clients
  ros::ServiceClient move_client_; /**< a client for calling the service to move the robo-cylinder to a new location */
  ros::ServiceClient power_client_; /**< a client for calling the service to turn on the robo-cylinder */
  ros::ServiceClient home_client_; /**< a client for calling the service to move robo-cylinder to its home position */
  shared_ptr<Target> target_;
  shared_ptr<Camera> camera_;
  std::vector<Pose6d> target_to_camera_poses;
  int target_rows_;
  int target_cols_;
  double circle_spacing_;
  double circle_diameter_;
  std::string camera_name_;
  std::string image_topic_;
  int image_height_;
  int image_width_;
  CameraParameters camera_parameters_;
};// end of class robocyl_ical_8D

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ical_cal_service");
  ros::NodeHandle node_handle;
  robocyl_ical_8D cal_object(node_handle);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
