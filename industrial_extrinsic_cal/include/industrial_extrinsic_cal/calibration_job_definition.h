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
#include <industrial_extrinsic_cal/circle_cost_utils.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/types.h"
#include <ros/console.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

namespace industrial_extrinsic_cal
{
namespace covariance_requests
{
enum CovarianceRequestType
{
  DefaultInvalid = 0,
  StaticCameraIntrinsicParams,
  StaticCameraExtrinsicParams,
  MovingCameraIntrinsicParams,
  MovingCameraExtrinsicParams,
  StaticTargetPoseParams,
  MovingTargetPoseParams
};
}  // end of namespace covariance_requests
struct CovarianceVariableRequest
{
  covariance_requests::CovarianceRequestType request_type;
  std::string object_name;
  int scene_id;
};

/*! @brief converts an integer to a covariance request type
 * @param request the integer request type
 * @returns the covariance request type
 */
covariance_requests::CovarianceRequestType intToCovRequest(int request);

/*! @brief defines and executes the calibration script */
class CalibrationJob
{
public:
  /** @brief constructor */
  CalibrationJob(std::string camera_fn, std::string target_fn, std::string caljob_fn)
    : camera_def_file_name_(camera_fn)
    , target_def_file_name_(target_fn)
    , caljob_def_file_name_(caljob_fn)
    , solved_(false)
    , problem_(NULL)
    , post_proc_on_(false){};

  /** @brief default destructor */
  ~CalibrationJob(){};

  /** @brief reads input files to create a calibration job
   * @return true if successful
   */
  bool load();

  /** @brief stores calibration job as 3 files
   * @param filePath is the complete path and name of the file inwhich to save the transform data
   * @return true if successful
   */
  bool store(std::string filePath);

  /** @brief shows the current poses of all cameras and targets
   * @return true if successful
   */
  void show();

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

  const std::string& getReferenceFrame() const
  {
    return ceres_blocks_.reference_frame_;
  }

  /** @brief get cost per observation
   *   @returns average cost per observation after optimization
   **/
  double finalCostPerObservation();

  /** @brief get initial cost per observation
   *   @returns cost per observation before optimization
   **/
  double initialCostPerObservation();

  /** @brief This is a diagnostics routine to compute the covariance of the results for the requested variables
   *    @param variables a list of cameras and targets
   *    @param covariance_file_name name of file to store the resulting matrix in
   */
  bool computeCovariance(std::vector<CovarianceVariableRequest>& variables, std::string& covariance_file_name);

  /** @brief set the flag to save observation data to a file indicated for post processing
   *    @param post_proc_file_name the name of the file to put the post processing data
   */
  void postProcessingOn(std::string post_proc_file_name);

  /** @brief clears the flag that saves observation data to a file for post processing */
  void postProcessingOff();

  /*@brief get pointer to the blocks moving and static cameras and targets */
  CeresBlocks* getBlocks()
  {
    return &ceres_blocks_;
  };

  /*@brief get pointer to list of scenes*/
  std::vector<ObservationScene>* getScenes()
  {
    return &scene_list_;
  };
  
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
  bool appendNewScene(boost::shared_ptr<Trigger> trig);

  /** @brief each camera and each target have a transform interface, push the current values to the interface */
  void pushTransforms();

  /** @brief each camera and each target have a transform interface, pull the current values from the interface
   *    @param scene_id the id for the scene, only pull transforms from targets and cameras of this scene
*/
  void pullTransforms(int scene_id);

private:
  std::vector<ObservationDataPointList> observation_data_point_list_; /*!< a list of observation data points */
  std::vector<ObservationScene> scene_list_; /*!< contains list of scenes which define the job */
  std::string camera_def_file_name_;         /*!< this file describes all cameras in job */
  std::string target_def_file_name_;         /*!< this file describes all targets in job */
  std::string caljob_def_file_name_;         /*!< this file describes all observations in job */
  int current_scene_;                        /*!< id of current scene under review or construction */
  CeresBlocks ceres_blocks_;                 /*!< This structure maintains the parameter sets for ceres */
  ceres::Problem* problem_;              /*!< this is the object used to define the optimization problem for ceres */
  ceres::Solver::Summary ceres_summary_; /*!< object for displaying solver results */
  int total_observations_;               /*< number of observations/cost elements in problem */
  bool solved_;                          /*< set once the problem has been solved, allows covariance to be computed*/
  bool post_proc_on_;                    /*< flag indicating to save the observation data for post processing */
  std::string post_proc_data_file_;      /*< file name for observation data for post processing */
};                                       // end class

}  // end namespace industrial_extrinsic_cal

#endif /* CALIBRATION_JOB_DEFINITION_H_ */
