/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
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

#include <industrial_extrinsic_cal/calibration_job_definition.h>
#include <industrial_extrinsic_cal/ros_transform_interface.h>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <industrial_extrinsic_cal/manual_triggerAction.h>
#include <industrial_extrinsic_cal/trigger.h>
#include <industrial_extrinsic_cal/ros_triggers.h>
#include <industrial_extrinsic_cal/ceres_costs_utils.h>
#include <industrial_extrinsic_cal/camera_yaml_parser.h>
#include <industrial_extrinsic_cal/targets_yaml_parser.h>
#include <industrial_extrinsic_cal/caljob_yaml_parser.h>
#include <industrial_extrinsic_cal/ros_target_display.hpp>

using std::string;
using boost::shared_ptr;
using boost::make_shared;
using ceres::CostFunction;
using industrial_extrinsic_cal::covariance_requests::CovarianceRequestType;

namespace industrial_extrinsic_cal
{
/** @brief, a function used for debugging, and generating output for post processing */
void writeObservationData(std::string file_name, std::vector<ObservationDataPointList> odl)
{
  FILE* fp = NULL;
  fp = fopen(file_name.c_str(), "w");
  if (fp == NULL)
  {
    ROS_ERROR("Could not open %s", file_name.c_str());
    return;
  }

  int scene_id = -1;
  for (int i = 0; (int)i < odl.size(); i++)
  {
    if (odl[i].items_.size() > 0)
    {
      scene_id = odl[i].items_[0].scene_id_;
      fprintf(fp, "scene_id = %d \n", scene_id);
      Pose6d t_pose;
      t_pose.ax = odl[i].items_[0].target_pose_[0];
      t_pose.ay = odl[i].items_[0].target_pose_[1];
      t_pose.az = odl[i].items_[0].target_pose_[2];
      t_pose.x = odl[i].items_[0].target_pose_[3];
      t_pose.y = odl[i].items_[0].target_pose_[4];
      t_pose.z = odl[i].items_[0].target_pose_[5];
      tf::Matrix3x3 basis = t_pose.getBasis();
      fprintf(fp, "target_pose = [ %f %f %f %f;\n %f %f %f %f;\n %f %f %f %f;\n %f %f %f %f];\n", basis[0][0],
              basis[0][1], basis[0][2], t_pose.x, basis[1][0], basis[1][1], basis[1][2], t_pose.y, basis[2][0],
              basis[2][1], basis[2][2], t_pose.z, 0.0, 0.0, 0.0, 1.0);
      fprintf(fp, "cameras = [ ");
      std::string camera_name("NULL");
      for (int j = 0; (int)j < odl[i].items_.size(); j++)
      {
        if (camera_name != odl[i].items_[j].camera_name_)
        {
          fprintf(fp, "%s ", odl[i].items_[j].camera_name_.c_str());
        }
        camera_name = odl[i].items_[j].camera_name_;
      }
      fprintf(fp, "]\n");
    }
  }
  fclose(fp);
}
CovarianceRequestType intToCovRequest(int request)
{
  switch (request)
  {
    case 0:
      return covariance_requests::StaticCameraIntrinsicParams;
      break;
    case 1:
      return covariance_requests::StaticCameraExtrinsicParams;
      break;
    case 2:
      return covariance_requests::MovingCameraIntrinsicParams;
      break;
    case 3:
      return covariance_requests::MovingCameraExtrinsicParams;
      break;
    case 4:
      return covariance_requests::StaticTargetPoseParams;
      break;
    case 5:
      return covariance_requests::MovingTargetPoseParams;
      break;
    default:
      return covariance_requests::DefaultInvalid;
      break;
  }
}  // end of intToCovRequestType

bool CalibrationJob::load()
{
  if (!CalibrationJob::loadCamera())
  {
    ROS_ERROR_STREAM("Camera file parsing failed");
    return false;
  }
  if (!CalibrationJob::loadTarget())
  {
    ROS_ERROR_STREAM("Target file parsing failed");
    return false;
  }
  if (!CalibrationJob::loadCalJob())
  {
    ROS_ERROR_STREAM("Calibration Job file parsing failed");
    return false;
  }

  return (true);
}  // end load()

void CalibrationJob::postProcessingOn(std::string post_proc_file_name)
{
  post_proc_on_ = true;
  post_proc_data_file_ = post_proc_file_name;
}

void CalibrationJob::postProcessingOff()
{
  post_proc_on_ = false;
}

bool CalibrationJob::loadCamera()
{
  bool rtn = true;
  std::vector<shared_ptr<Camera> > all_cameras;
  if (!parseCameras(camera_def_file_name_, all_cameras))
  {
    ROS_ERROR("failed to parse cameras from %s", camera_def_file_name_.c_str());
    rtn = false;
  }
  for (int i = 0; i < (int)all_cameras.size(); i++)
  {
    if (all_cameras[i]->is_moving_)
    {
      int scene_id = 0;
      ceres_blocks_.addMovingCamera(all_cameras[i], scene_id);
    }
    else
    {
      ceres_blocks_.addStaticCamera(all_cameras[i]);
    }
  }
  return rtn;
}

bool CalibrationJob::loadTarget()
{
  bool rtn = true;
  std::vector<shared_ptr<Target> > all_targets;
  if (!parseTargets(target_def_file_name_, all_targets))
  {
    ROS_ERROR("failed to parse targets from %s", target_def_file_name_.c_str());
    rtn = false;
  }
  else
  {  // target file parses ok
    for (int i = 0; i < (int)all_targets.size(); i++)
    {
      if (all_targets[i]->pub_rviz_vis_){ // use rviz visualization marker to display the target, currently must be modified circle grid
	displayRvizTarget(all_targets[i]);
      }
      if (all_targets[i]->is_moving_)
      {
        int scene_id = 0;
        ceres_blocks_.addMovingTarget(all_targets[i], scene_id);
      }
      else
      {
        ceres_blocks_.addStaticTarget(all_targets[i]);
      }
    }  // end for every target found
  }    // end else target file parsed ok
  return rtn;
}

bool CalibrationJob::loadCalJob()
{
  bool rtn = true;
  std::string reference_frame;
  rtn = parseCaljob(caljob_def_file_name_, scene_list_, reference_frame, ceres_blocks_);
  if (rtn) ceres_blocks_.setReferenceFrame(reference_frame);
  return rtn;
}

bool CalibrationJob::run()
{
  ROS_INFO("Collecting observations");
  runObservations();
  ROS_INFO("Running optimization");
  solved_ = runOptimization();
  if (solved_)
  {
    pushTransforms();  // sends updated transforms to their intefaces
  }
  else
  {
    ROS_ERROR("Optimization failed");
  }
  return (solved_);
}

bool CalibrationJob::runObservations()
{
  // the result of this function are twofold
  // First, it fills up observation_data_point_list_ with lists of observationspercamera
  // Second it adds parameter blocks to the ceres_blocks
  // extrinsics and intrinsics for each static camera
  // The whole target for once every static target (parameter blocks are in  Pose6d and an array of points)
  // The whole target once a scene for each moving target
  observation_data_point_list_.clear();  // clear previously recorded observations

  // For each scene
  BOOST_FOREACH (ObservationScene current_scene, scene_list_)
  {
    int scene_id = current_scene.get_id();
    ROS_DEBUG_STREAM("Processing Scene " << scene_id + 1 << " of " << scene_list_.size());
    current_scene.get_trigger()->waitForTrigger();  // this indicates scene is ready to capture
    pullTransforms(scene_id);                       // gets transforms of targets and cameras from their interfaces
    BOOST_FOREACH (shared_ptr<Camera> current_camera, current_scene.cameras_in_scene_)
    {  // clear camera of existing observations

      current_camera->camera_observer_->clearObservations();  // clear any recorded data
      current_camera->camera_observer_->clearTargets();       // clear all targets
    }
    BOOST_FOREACH (ObservationCmd o_command, current_scene.observation_command_list_)
    {  // add each target and roi each camera's list of observations

      o_command.camera->camera_observer_->addTarget(o_command.target, o_command.roi, o_command.cost_type);
    }
    BOOST_FOREACH (shared_ptr<Camera> current_camera, current_scene.cameras_in_scene_)
    {  // trigger the cameras
      current_camera->camera_observer_->triggerCamera();
    }
    // collect results
    P_BLOCK intrinsics;
    P_BLOCK extrinsics;
    P_BLOCK target_pose;
    P_BLOCK pnt_pos;
    std::string camera_name;
    std::string target_name;
    unsigned int target_type;
    Cost_function cost_type;

    // for each camera in scene get a list of observations, and add camera parameters to ceres_blocks
    ObservationDataPointList listpercamera;
    BOOST_FOREACH (shared_ptr<Camera> camera, current_scene.cameras_in_scene_)
    {
      // wait until observation is done
      while (!camera->camera_observer_->observationsDone())
        ;
      camera_name = camera->camera_name_;
      if (camera->isMoving())
      {
        // next line does nothing if camera already exist in blocks
        ceres_blocks_.addMovingCamera(camera, scene_id);
        // TODO why is there a second call to pull transforms here?
        pullTransforms(scene_id);  // gets transforms of targets and cameras from their interfaces
        intrinsics = ceres_blocks_.getMovingCameraParameterBlockIntrinsics(camera_name);
        extrinsics = ceres_blocks_.getMovingCameraParameterBlockExtrinsics(camera_name, scene_id);
      }
      else
      {
        // next line does nothing if camera already exist in blocks
        ceres_blocks_.addStaticCamera(camera);
        intrinsics = ceres_blocks_.getStaticCameraParameterBlockIntrinsics(camera_name);
        extrinsics = ceres_blocks_.getStaticCameraParameterBlockExtrinsics(camera_name);
      }
      // Get the observations from this camera whose P_BLOCKs are intrinsics and extrinsics
      CameraObservations camera_observations;
      int number_returned;
      number_returned = camera->getObservations(camera_observations);
      ROS_DEBUG_STREAM("Processing " << camera_observations.size() << " Observations");
      BOOST_FOREACH (Observation observation, camera_observations)
      {
        target_name = observation.target->target_name_;
        target_type = observation.target->target_type_;
        cost_type = observation.cost_type;
        double circle_dia = 0.0;
        if (target_type == pattern_options::CircleGrid || target_type == pattern_options::ModifiedCircleGrid)
        {
          circle_dia = observation.target->circle_grid_parameters_.circle_diameter;
        }
        int pnt_id = observation.point_id;
        double observation_x = observation.image_loc_x;
        double observation_y = observation.image_loc_y;
        if (observation.target->is_moving_)
        {
          ceres_blocks_.addMovingTarget(observation.target, scene_id);
          target_pose = ceres_blocks_.getMovingTargetPoseParameterBlock(target_name, scene_id);
          pnt_pos = ceres_blocks_.getMovingTargetPointParameterBlock(target_name, pnt_id);
        }
        else
        {
          ceres_blocks_.addStaticTarget(observation.target);  // if exist, does nothing
          target_pose = ceres_blocks_.getStaticTargetPoseParameterBlock(target_name);
          pnt_pos = ceres_blocks_.getStaticTargetPointParameterBlock(target_name, pnt_id);
        }
        ObservationDataPoint temp_ODP(camera_name, target_name, target_type, scene_id, intrinsics, extrinsics, pnt_id,
                                      target_pose, pnt_pos, observation_x, observation_y, cost_type,
                                      observation.intermediate_frame, circle_dia);
        listpercamera.addObservationPoint(temp_ODP);
      }  // end for each observed point
    }    // end for each camera
    observation_data_point_list_.push_back(listpercamera);
  }  // end for each scene
  return true;
}

bool CalibrationJob::runOptimization()
{
  if (post_proc_on_) writeObservationData(post_proc_data_file_, observation_data_point_list_);

  // problem is declared here because we can't clear it as far as I can tell from the ceres documentation
  if (problem_ != NULL)
  {
    ROS_INFO("Deleting old problem.");
    delete (problem_);
  }
  problem_ = new ceres::Problem; /*!< This is the object which solves non-linear optimization problems */

  total_observations_ = 0;
  for (int i = 0; i < observation_data_point_list_.size(); i++)
  {
    total_observations_ += observation_data_point_list_[i].items_.size();
    std::stringstream observations_ss;
    for (int pntIdx = 0; pntIdx < observation_data_point_list_[i].items_.size(); pntIdx++)
    {
      const ObservationDataPoint& odp = observation_data_point_list_[i].items_[pntIdx];
      ROS_DEBUG("%d: id=%d pos=[%f, %f, %f] img=[%f, %f]", pntIdx, odp.point_id_, odp.point_position_[0],
                odp.point_position_[1], odp.point_position_[2], odp.image_x_, odp.image_y_);
      observations_ss << "[" << odp.image_x_ << ", " << odp.image_y_ << "],";
    }
    ROS_DEBUG("project_points2d: %s", observations_ss.str().c_str());
  }
  if (total_observations_ == 0)
  {  // TODO really need more than number of parameters being computed
    ROS_ERROR("Too few observations: %d", total_observations_);
    return (false);
  }
  else
  {
    ROS_INFO("total observations = %d", total_observations_);
  }

  // TODO remove commented code     ceres_blocks_.displayMovingCameras();

  // take all the data collected and create a Ceres optimization problem and run it
  ROS_INFO("Running Optimization with %d scenes", (int)scene_list_.size());
  ROS_DEBUG_STREAM("Optimizing " << scene_list_.size() << " scenes");
  BOOST_FOREACH (ObservationScene current_scene, scene_list_)
  {
    int scene_id = current_scene.get_id();
    ROS_DEBUG_STREAM(
        "Current observation data point list size: " << observation_data_point_list_.at(scene_id).items_.size());
    // take all the data collected and create a Ceres optimization problem and run it
    P_BLOCK extrinsics;
    P_BLOCK intrinsics;
    P_BLOCK target_pose_params;
    P_BLOCK point_position;
    BOOST_FOREACH (ObservationDataPoint ODP, observation_data_point_list_.at(scene_id).items_)
    {
      // create cost function
      // there are several options
      // 1. the complete reprojection error cost function "Create(obs_x,obs_y)"
      //    this cost function has the following parameters:
      //      a. camera intrinsics
      //      b. camera extrinsics
      //      c. target pose
      //      d. point location in target frame
      // 2. the same as 1, but without d  "Create(obs_x,obs_y,t_pnt_x, t_pnt_y, t_pnt_z)
      // 3. the same as 1, but without a  "Create(obs_x,obs_y,fx,fy,cx,cy,cz)"
      //    Note that this one assumes we are using rectified images to compute the observations
      // 4. the same as 3, point location fixed too "Create(obs_x,obs_y,fx,fy,cx,cy,cz,t_x,t_y,t_z)"
      //        implemented in TargetCameraReprjErrorNoDistortion
      // 5. the same as 4, but with target in known location
      //    "Create(obs_x,obs_y,fx,fy,cx,cy,cz,t_x,t_y,t_z,p_tx,p_ty,p_tz,p_ax,p_ay,p_az)"
      // pull out the constants from the observation point data
      intrinsics = ODP.camera_intrinsics_;
      double focal_length_x = ODP.camera_intrinsics_[0];  // TODO, make this not so ugly
      double focal_length_y = ODP.camera_intrinsics_[1];
      double center_x = ODP.camera_intrinsics_[2];
      double center_y = ODP.camera_intrinsics_[3];
      double image_x = ODP.image_x_;
      double image_y = ODP.image_y_;
      Point3d point;
      Pose6d camera_mounting_pose = ODP.intermediate_frame_;  // identity except when camera mounted on robot
      point.x = ODP.point_position_[0];                       // location of point within target frame
      point.y = ODP.point_position_[1];
      point.z = ODP.point_position_[2];
      unsigned int target_type = ODP.target_type_;
      double circle_dia = ODP.circle_dia_;  // sometimes this is not needed

      // pull out pointers to the parameter blocks in the observation point data
      extrinsics = ODP.camera_extrinsics_;
      target_pose_params = ODP.target_pose_;
      Pose6d target_pose;
      target_pose.setAngleAxis(target_pose_params[0], target_pose_params[1], target_pose_params[2]);
      target_pose.setOrigin(target_pose_params[3], target_pose_params[4], target_pose_params[5]);
      point_position = ODP.point_position_;
      bool point_zero = false;  // Set true to enable some debugging

      if (0)
      {  // modify for debugging
        if (point.x == 0.0 && point.y == 0.0 && point.z == 0.0)
        {
          point_zero = true;
          //		ROS_ERROR("Observing Target Origin");
          //		showPose(target_pose_params, "target");
          target_pose.show("target_pose");
          showPose(extrinsics, "extrinsics");
          ROS_ERROR("u,v %6.3f %6.3f", image_x, image_y);
          //		showPose((P_BLOCK) &camera_mounting_pose.pb_pose[0], "camera_mounting_pose");
        }
      }

      switch (ODP.cost_type_)
      {
        case cost_functions::CameraReprjErrorWithDistortion:
        {
          CostFunction* cost_function = CameraReprjErrorWithDistortion::Create(image_x, image_y);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics, intrinsics, point.pb);
        }
        break;
        case cost_functions::CameraReprjErrorWithDistortionPK:
        {
          CostFunction* cost_function = CameraReprjErrorWithDistortionPK::Create(image_x, image_y, point);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics, intrinsics);
        }
        break;
        case cost_functions::CameraReprjError:
        {
          CostFunction* cost_function =
              CameraReprjError::Create(image_x, image_y, focal_length_x, focal_length_y, center_x, center_y);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics, point.pb);
        }
        break;
        case cost_functions::CameraReprjErrorPK:
        {
          CostFunction* cost_function =
              CameraReprjErrorPK::Create(image_x, image_y, focal_length_x, focal_length_y, center_x, center_y, point);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics);
        }
        break;
        case cost_functions::TargetCameraReprjError:
        {
          CostFunction* cost_function =
              TargetCameraReprjError::Create(image_x, image_y, focal_length_x, focal_length_y, center_x, center_y);

          problem_->AddResidualBlock(cost_function, NULL, extrinsics, target_pose_params, point.pb);
        }
        break;
        case cost_functions::TargetCameraReprjErrorPK:
        {
          CostFunction* cost_function = TargetCameraReprjErrorPK::Create(image_x, image_y, focal_length_x,
                                                                         focal_length_y, center_x, center_y, point);
          // add it as a residual using parameter blocks
          problem_->AddResidualBlock(cost_function, NULL, extrinsics, target_pose_params);
        }
        break;
        case cost_functions::LinkTargetCameraReprjError:
        {
          CostFunction* cost_function = LinkTargetCameraReprjError::Create(
              image_x, image_y, focal_length_x, focal_length_y, center_x, center_y, camera_mounting_pose);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics, target_pose_params, point.pb);
        }
        break;
        case cost_functions::LinkTargetCameraReprjErrorPK:
        {
          CostFunction* cost_function = LinkTargetCameraReprjErrorPK::Create(
              image_x, image_y, focal_length_x, focal_length_y, center_x, center_y, camera_mounting_pose, point);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics, target_pose_params);
        }
        break;
        case cost_functions::PosedTargetCameraReprjErrorPK:
        {
          if (point_zero)
          {
            ROS_ERROR("u,v %f %f x,y,x %f %f %f", image_x, image_y, point.x, point.y, point.z);
          }
          CostFunction* cost_function = PosedTargetCameraReprjErrorPK::Create(
              image_x, image_y, focal_length_x, focal_length_y, center_x, center_y, target_pose, point);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics);
        }
        break;
        case cost_functions::LinkCameraTargetReprjError:
        {
          CostFunction* cost_function = LinkCameraTargetReprjError::Create(
              image_x, image_y, focal_length_x, focal_length_y, center_x, center_y, camera_mounting_pose);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics, target_pose_params, point.pb);
        }
        break;
        case cost_functions::LinkCameraTargetReprjErrorPK:
        {
          CostFunction* cost_function = LinkCameraTargetReprjErrorPK::Create(
              image_x, image_y, focal_length_x, focal_length_y, center_x, center_y, camera_mounting_pose, point);

          problem_->AddResidualBlock(cost_function, NULL, extrinsics, target_pose_params);
        }
        break;
        case cost_functions::CircleCameraReprjErrorWithDistortion:
        {
          CostFunction* cost_function = CircleCameraReprjErrorWithDistortion::Create(image_x, image_y, circle_dia);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics, intrinsics, point.pb);
        }
        break;
        case cost_functions::CircleCameraReprjErrorWithDistortionPK:
        {
          CostFunction* cost_function =
              CircleCameraReprjErrorWithDistortionPK::Create(image_x, image_y, circle_dia, point);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics, intrinsics, point.pb);
        }
        break;
        case cost_functions::CircleCameraReprjError:
        {
          CostFunction* cost_function = CircleCameraReprjError::Create(image_x, image_y, circle_dia, focal_length_x,
                                                                       focal_length_y, center_x, center_y);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics, point.pb);
        }
        break;
        case cost_functions::CircleCameraReprjErrorPK:
        {
          CostFunction* cost_function = CircleCameraReprjErrorPK::Create(image_x, image_y, circle_dia, focal_length_x,
                                                                         focal_length_y, center_x, center_y, point);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics);
        }
        break;
        case cost_functions::CircleTargetCameraReprjErrorWithDistortion:
        {
          CostFunction* cost_function =
              CircleTargetCameraReprjErrorWithDistortion::Create(image_x, image_y, circle_dia);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics, intrinsics, point.pb);
        }
        break;
        case cost_functions::CircleTargetCameraReprjErrorWithDistortionPK:
        {
          CostFunction* cost_function =
              CircleTargetCameraReprjErrorWithDistortionPK::Create(image_x, image_y, circle_dia, point);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics, intrinsics, target_pose_params);
        }
        break;
        case cost_functions::FixedCircleTargetCameraReprjErrorWithDistortionPK:
        {
          CostFunction* cost_function =
              FixedCircleTargetCameraReprjErrorWithDistortionPK::Create(image_x, image_y, circle_dia, point);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics, intrinsics, target_pose_params);
        }
        break;
        case cost_functions::SimpleCircleTargetCameraReprjErrorWithDistortionPK:
        {
          CostFunction* cost_function =
              SimpleCircleTargetCameraReprjErrorWithDistortionPK::Create(image_x, image_y, circle_dia, point);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics, intrinsics);
        }
        break;
        case cost_functions::CircleTargetCameraReprjErrorPK:
        {
          CostFunction* cost_function = CircleTargetCameraReprjErrorPK::Create(
              image_x, image_y, circle_dia, focal_length_x, focal_length_y, center_x, center_y, point);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics, target_pose_params);
        }
        break;
        case cost_functions::LinkCircleTargetCameraReprjError:
        {
          CostFunction* cost_function = LinkCircleTargetCameraReprjError::Create(
              image_x, image_y, circle_dia, focal_length_x, focal_length_y, center_x, center_y, camera_mounting_pose);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics, target_pose_params, point.pb);
        }
        break;
        case cost_functions::LinkCircleTargetCameraReprjErrorPK:
        {
          CostFunction* cost_function =
              LinkCircleTargetCameraReprjErrorPK::Create(image_x, image_y, circle_dia, focal_length_x, focal_length_y,
                                                         center_x, center_y, camera_mounting_pose, point);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics, target_pose_params);
        }
        break;
        case cost_functions::LinkCameraCircleTargetReprjError:
        {
          CostFunction* cost_function = LinkCameraCircleTargetReprjError::Create(
              image_x, image_y, circle_dia, focal_length_x, focal_length_y, center_x, center_y, camera_mounting_pose);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics, target_pose_params, point.pb);
        }
        break;
        case cost_functions::LinkCameraCircleTargetReprjErrorPK:
        {
          CostFunction* cost_function =
              LinkCameraCircleTargetReprjErrorPK::Create(image_x, image_y, circle_dia, focal_length_x, focal_length_y,
                                                         center_x, center_y, camera_mounting_pose, point);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics, target_pose_params);
          if (point_zero)
          {
            double residual[2];
            double* params[2];
            params[0] = &extrinsics[0];
            params[1] = &target_pose_params[0];
            cost_function->Evaluate(params, residual, NULL);
            ROS_INFO("Initial residual %6.3lf %6.3lf ix,iy = %6.3lf %6.3lf px,py = %6.3lf %6.3lf", residual[0],
                     residual[1], image_x, image_y, residual[0] + image_x, residual[0] + image_y);
            point_zero = false;
            LinkCameraCircleTargetReprjErrorPK testIt(image_x, image_y, circle_dia, focal_length_x, focal_length_y,
                                                      center_x, center_y, camera_mounting_pose, point);
            testIt.test_residual(extrinsics, target_pose_params, residual);
          }
        }
        break;
        case cost_functions::FixedCircleTargetCameraReprjErrorPK:
        {
          CostFunction* cost_function =
              FixedCircleTargetCameraReprjErrorPK::Create(image_x, image_y, circle_dia, focal_length_x, focal_length_y,
                                                          center_x, center_y, target_pose, camera_mounting_pose, point);
          problem_->AddResidualBlock(cost_function, NULL, extrinsics);
          if (point_zero)
          {
            double residual[2];
            double* params[1];
            showPose(extrinsics, "extrinsics");
            params[0] = &extrinsics[0];
            cost_function->Evaluate(params, residual, NULL);
            ROS_ERROR("Initial residual %6.3lf %6.3lf ix,iy = %6.3lf %6.3lf px,py = %6.3lf %6.3lf", residual[0],
                      residual[1], image_x, image_y, residual[0] + image_x, residual[0] + image_y);
            point_zero = false;
            FixedCircleTargetCameraReprjErrorPK testIt(image_x, image_y, circle_dia, focal_length_x, focal_length_y,
                                                       center_x, center_y, target_pose, camera_mounting_pose, point);
            testIt.test_residual(extrinsics, residual);
          }
        }
        break;
        case cost_functions::TriangulationError:
        {
          double x, y, z, ax, ay, az;
          extractCameraExtrinsics(extrinsics, x, y, z, ax, ay, az);
          Pose6d camera_pose(x, y, z, ax, ay, az);
          CostFunction* cost_function = TriangulationError::Create(image_x, image_y, focal_length_x, focal_length_y,
                                                                   center_x, center_y, camera_pose);
          problem_->AddResidualBlock(cost_function, NULL, point.pb);
        }
        break;
        default:
        {
          std::string cost_type_string = costType2String(ODP.cost_type_);
          ROS_ERROR("No cost function of type %s", cost_type_string.c_str());
        }
        break;
      }  // end of switch
    }    // for each observation
  }      // for each scene
  ROS_INFO("total observations: %d ", total_observations_);

  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 1000;
  ceres::Solve(options, problem_, &ceres_summary_);

  if (ceres_summary_.termination_type != ceres::NO_CONVERGENCE)
  {
    ROS_INFO("Problem Solved");
    double error_per_observation = ceres_summary_.initial_cost / total_observations_;

    return true;
  }
  else
  {
    ROS_ERROR("Problem Not Solved termination type = %d success = %d", ceres_summary_.termination_type,
              ceres::USER_SUCCESS);
  }

}  // end runOptimization

bool CalibrationJob::computeCovariance(std::vector<CovarianceVariableRequest>& variables,
                                       std::string& covariance_file_name)
{
  FILE* fp;
  if ((fp = fopen(covariance_file_name.c_str(), "w")) != NULL)
  {
    ceres::Covariance::Options covariance_options;
    covariance_options.algorithm_type = ceres::DENSE_SVD;
    ceres::Covariance covariance(covariance_options);
    std::vector<const double*> covariance_blocks;
    std::vector<int> block_sizes;
    std::vector<std::string> block_names;
    std::vector<std::pair<const double*, const double*> > covariance_pairs;

    BOOST_FOREACH (CovarianceVariableRequest req, variables)
    {
      P_BLOCK intrinsics, extrinsics, pose_params;
      switch (req.request_type)
      {
        case covariance_requests::StaticCameraIntrinsicParams:
          intrinsics = ceres_blocks_.getStaticCameraParameterBlockIntrinsics(req.object_name.c_str());
          covariance_blocks.push_back(intrinsics);
          block_sizes.push_back(9);
          block_names.push_back(req.object_name.c_str());
          break;
        case covariance_requests::StaticCameraExtrinsicParams:
          extrinsics = ceres_blocks_.getStaticCameraParameterBlockExtrinsics(req.object_name.c_str());
          covariance_blocks.push_back(extrinsics);
          block_sizes.push_back(6);
          block_names.push_back(req.object_name.c_str());
          break;
        case covariance_requests::MovingCameraIntrinsicParams:
          intrinsics = ceres_blocks_.getMovingCameraParameterBlockIntrinsics(req.object_name.c_str());
          covariance_blocks.push_back(intrinsics);
          block_sizes.push_back(9);
          block_names.push_back(req.object_name.c_str());
          break;
        case covariance_requests::MovingCameraExtrinsicParams:
          extrinsics = ceres_blocks_.getMovingCameraParameterBlockExtrinsics(req.object_name.c_str(), req.scene_id);
          covariance_blocks.push_back(extrinsics);
          block_sizes.push_back(6);
          block_names.push_back(req.object_name.c_str());
          break;
        case covariance_requests::StaticTargetPoseParams:
          pose_params = ceres_blocks_.getStaticTargetPoseParameterBlock(req.object_name.c_str());
          covariance_blocks.push_back(pose_params);
          block_sizes.push_back(6);
          block_names.push_back(req.object_name.c_str());
          break;
        case covariance_requests::MovingTargetPoseParams:
          pose_params = ceres_blocks_.getMovingTargetPoseParameterBlock(req.object_name.c_str(), req.scene_id);
          covariance_blocks.push_back(pose_params);
          block_sizes.push_back(6);
          block_names.push_back(req.object_name.c_str());
          break;
        default:
          ROS_ERROR("unknown type of request");
          return (false);
          break;
      }  // end of switch for request type
    }    // end of for each request

    // create pairs from every combination of blocks in request
    for (int i = 0; i < (int)covariance_blocks.size(); i++)
    {
      for (int j = i; j < (int)covariance_blocks.size(); j++)
      {
        covariance_pairs.push_back(std::make_pair(covariance_blocks[i], covariance_blocks[j]));
      }
    }
    covariance.Compute(covariance_pairs, problem_);

    fprintf(fp, "covariance blocks:\n");
    for (int i = 0; i < (int)covariance_blocks.size(); i++)
    {
      for (int j = i; j < (int)covariance_blocks.size(); j++)
      {
        fprintf(fp, "Cov[%s, %s]\n", block_names[i].c_str(), block_names[j].c_str());
        int N = block_sizes[i];
        int M = block_sizes[j];
        double ij_cov_block[N * M];
        covariance.GetCovarianceBlock(covariance_blocks[i], covariance_blocks[j], ij_cov_block);
        for (int q = 0; q < N; i++)
        {
          for (int k = 0; k < M; k++)
          {
            double sigma_i = sqrt(ij_cov_block[q * N + q]);
            double sigma_j = sqrt(ij_cov_block[k * N + k]);
            if (q == k)
            {
              fprintf(fp, "%6.3f ", sigma_i);
            }
            else
            {
              fprintf(fp, "%6.3lf ", ij_cov_block[q * N + k] / (sigma_i * sigma_j));
            }
          }  // end of k loop
          fprintf(fp, "\n");
        }  // end of q loop
      }    // end of j loop
    }      // end of i loop
    fclose(fp);
  }  // end of if file opens
  else
  {
    ROS_ERROR("could not open covariance file %s", covariance_file_name.c_str());
    return (false);
  }
  return (true);
}  // end computeCovariance

bool CalibrationJob::store(std::string filePath)
{
  bool rnt = ceres_blocks_.writeAllStaticTransforms(filePath);
  bool rtn = true;
  return rtn;
}

void CalibrationJob::show()
{
  ceres_blocks_.pullTransforms(
      -1);  // since we don't know which scene for any moving objects, only pull static transforms
  ceres_blocks_.displayAllCamerasAndTargets();
}
void CalibrationJob::pullTransforms(int scene_id)
{
  ceres_blocks_.pullTransforms(scene_id);
}
void CalibrationJob::pushTransforms()
{
  ceres_blocks_.pushTransforms();
}
double CalibrationJob::finalCostPerObservation()
{
  if (!solved_)
  {
    ROS_ERROR("Can't call costPerObservation prior to solving");
    return (-1.0);
  }
  return (ceres_summary_.final_cost / total_observations_);
}

double CalibrationJob::initialCostPerObservation()
{
  if (!solved_)
  {
    ROS_ERROR("Can't call costPerObservation prior to solving");
    return (-1.0);
  }
  return (ceres_summary_.initial_cost / total_observations_);
}

}  // end namespace industrial_extrinsic_cal
