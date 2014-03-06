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

#ifndef CALIBRATION_JOB_DEFINITION_H_
#define CALIBRATION_JOB_DEFINITION_H_


#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/camera_observer.hpp>
#include <industrial_extrinsic_cal/camera_definition.h>
#include <industrial_extrinsic_cal/observation_scene.h>
#include <industrial_extrinsic_cal/observation_data_point.h>
#include <industrial_extrinsic_cal/ceres_blocks.h>
#include <industrial_extrinsic_cal/ros_camera_observer.h>
#include <industrial_extrinsic_cal/ceres_costs_utils.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <ros/console.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

namespace industrial_extrinsic_cal
{

/*! @brief defines and executes the calibration script */
class CalibrationJob
{
public:
  /** @brief constructor */
  CalibrationJob(std::string camera_fn, std::string target_fn, std::string caljob_fn) :
      camera_def_file_name_(camera_fn), target_def_file_name_(target_fn), caljob_def_file_name_(caljob_fn)
  {
  }
  ;

  /** @brief default destructor */
  ~CalibrationJob()
  {
  }
  ;

  /** @brief reads input files to create a calibration job
   * @return true if successful
   */
  bool load();

  /** @brief stores calibration job as 3 files
   * @return true if successful
   */
  bool store();

  /** @brief runs both data collection and optimization
   * @return true if successful
   */
  bool run();

  /** @brief removes all camera observers from job
   *  @return true if successful
   */
  bool clearJobCameraObservers();

  /** @brief removes all targets from job
   *  @return true if successful
   */
  bool clearJobTargets();

  /** @brief clears all previously collected data
   *  @return true if successful
   */
  bool clearObservationData();

  /**
   * @brief get the private member extrinsics_
   * @return a parameter block of the optimized extrinsics of calibration_job
   */
  const std::vector<P_BLOCK> getExtrinsics() const
  {
    return extrinsics_;
  }

  /**
   * @brief get the private member original_extrinsics_
   * @return a parameter block of the original extrinsics of calibration_job
   */
  const std::vector<P_BLOCK> getOriginalExtrinsics() const
  {
    return original_extrinsics_;
  }

  /**
   * @brief get the private member target_pose_
   * @return a parameter block of the optimized target_pose of calibration_job
   */
  const std::vector<P_BLOCK> getTargetPose() const
  {
    return target_pose_;
  }

  const std::vector<std::string>& getCameraIntermediateFrame() const
  {
    return camera_intermediate_frames_;
  }

  const std::vector<std::string>& getCameraOpticalFrame() const
  {
    return camera_optical_frames_;
  }

  const std::string& getReferenceFrame() const
  {
    return reference_frame_;
  }

  const std::vector<std::string>& getTargetFrames() const
  {
    return target_frames_;
  }

  //    ::std::ostream& operator<<(::std::ostream& os, const CalibrationJob& C){ return os<< "TODO";}
protected:
  /*!
   * \brief reads target input files to create static and moving targets
   * @return true if successfully loaded target file
   */
  bool loadTarget();

  /*!
   * \brief reads camera input files to create static and moving cameras
   * @return true if successfully loaded camera file
   */
  bool loadCamera();

  /*!
   * \brief reads target input files to create a calibration job
   * @return true if successfully loaded caljob file
   */
  bool loadCalJob();

  /** @brief runs the data collection portion of the job
   * @return true if successful
   */
  bool runObservations();

  /** @brief runs the optimization portion of the job
   * @return true if successful
   */
  bool runOptimization();

  /** @brief Adds a new camera
   *  @param camera_to_add camera to add
   *  @return true if successful
   */
  bool addCameraObserver(Camera camera_to_add);

  /** @brief Adds a new target to defined target set
   *  @param target_to_add target to add
   *  @return true if successful
   */
  bool addTarget(Target target_to_add);

  /** @brief select active scene for modification
   *  @return true if successful
   */
  bool selectScene(int scene_id);

  /** @brief adds an observation from a particular camera to current scene
   *  @param camera_observer a pointer to the camera observer
   *  @param target a pointer to the target
   *  @return true if successful
   */
  bool addObservationToCurrentScene(boost::shared_ptr<ROSCameraObserver> camera_observer,
                                    boost::shared_ptr<Target> target);

  /** @brief removes all cameras and targets from current scene
   *  @return true if successful
   */
  bool clearCurrentScene();

  /** @brief adds a new scene to end of scene list. New scene becomes current scene.
   *  @param trig the trigger type to use for this scene
   *  @return true if successful
   */
  bool appendNewScene(Trigger trig);

private:
  std::vector<ObservationDataPointList> observation_data_point_list_;
  std::vector<ObservationScene> scene_list_; /*!< contains list of scenes which define the job */
  std::string camera_def_file_name_; /*!< this file describes all cameras in job */
  std::string target_def_file_name_; /*!< this file describes all targets in job */
  std::string caljob_def_file_name_; /*!< this file describes all observations in job */
  std::string reference_frame_; /*!< this the frame to which the camera is being calibrated (and to which the target is positioned) */
  std::vector<std::string> target_frames_; /*!< this the frame of the target points */
  std::vector<std::string> camera_optical_frames_; /*!< this the frame in which observations were made */
  std::vector<std::string> camera_intermediate_frames_; /*!< this the frame which links camera optical frame to reference frame */
  int current_scene_; /*!< id of current scene under review or construction */
  std::vector<ROSCameraObserver> camera_observers_; /*!< interface to images from cameras */
  std::vector<Target> defined_target_set_; /*!< TODO Not sure if I'll use this one */
  CeresBlocks ceres_blocks_; /*!< This structure maintains the parameter sets for ceres */
  ceres::Problem problem_; /*!< This is the object which solves non-linear optimization problems */
  std::vector<P_BLOCK> extrinsics_; /*!< This is the parameter block which holds the optimized camera extrinsics solution */
  std::vector<P_BLOCK> original_extrinsics_; /*!< This is the parameter block which holds the original camera extrinsics */
  std::vector<P_BLOCK> target_pose_; /*!< This is the parameter block which holds the optimized target pose solution */

};//end class

}//end namespace industrial_extrinsic_cal

#endif /* CALIBRATION_JOB_DEFINITION_H_ */
