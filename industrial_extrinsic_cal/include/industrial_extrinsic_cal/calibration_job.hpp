/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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

#ifndef CALIBRATION_JOB_HPP_
#define CALIBRATION_JOB_HPP_

#include <iostream>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/camera_observer.hpp>

//ROS include, for ROS macros (ROS_INFO etc.)
#include <ros/console.h>

namespace industrial_extrinsic_cal
{

/*! \brief a high level camera wrapper including its parameters, and its observer */
class Camera
{
public:
  /*! \brief default Constructor */
  Camera()
  {
    camera_name_ = "NONE";
    is_moving_ = false;
  }
  ;

  /*! \brief Constructor
   *  \param name name of camera
   *  \param camera_parameters intrinsic and extrinsic camera parameters
   *  \param is_moving static camera = false, moving camera=true
   */
  Camera(std::string name, CameraParameters camera_parameters, bool is_moving) :
      camera_name_(name), camera_parameters_(camera_parameters_), is_moving_(is_moving)
  {
  }
  ;

  /*! \brief default destructor, nothing to do really */
  ~Camera()
  {
  }
  ;

  /*!  is the camera's location fixed or not?
   moving cameras get multiple pose parameters
   static cameras get but one set of pose parameters
   */
  bool isMoving()
  {
    return (is_moving_);
  }
  ;
  boost::shared_ptr<DummyCameraObserver> camera_observer_;/*!< processes images, does CameraObservations */
  CameraParameters camera_parameters_;/*!< The intrinsic and extrinsic parameters */
  //    ::std::ostream& operator<<(::std::ostream& os, const Camera& C){ return os<< "TODO";};

  std::string camera_name_; /*!< string camera_name_ unique name of a camera */
private:
  bool is_moving_; /*!< bool is_moving_  false for static cameras */
};
// end of class Camera

/*! @brief what kind of trigger initiates the collection of data for this scene */
typedef struct
{ /** Trigger */
  int trigger_type;
  std::string trigger_popup_msg;
} Trigger;

/*! @brief unique observation command */
typedef struct
{
  boost::shared_ptr<DummyCameraObserver> camera;
  boost::shared_ptr<Target> target;
  Roi roi;
} ObservationCmd;

/*! \brief a command to take a set of observations from a group of cameras upon a trigger event */
class ObservationScene
{
public:
  /*! \brief Constructor,
   *   \param Trigger Trig the type of trigger to initiate this observation
   */
  ObservationScene(Trigger trig, int scene_id) :
      trig_(trig), scene_id_(scene_id)
  {
  }
  ;

  /*! \brief destructor, clears observation command list*/
  ObservationScene()
  {
    observation_command_list_.clear();
  }
  ;

  /*! \brief  Clears all observations from the command */
  void clear();

  /*! \brief Adds and observation of a target by a specific camera in a region of interest
   *  \param observation_command: the camera to make the observation
   *  \param target: the target to be observed
   *  \param roi:    the region of interest in the camera's field of view to look for target
   */
  void addObservationToScene(ObservationCmd observation_command);
private:
  std::vector<ObservationCmd> observation_command_list_; /*!< list of observations for a scene */
  Trigger trig_; /*!< event to trigger the observations in this command */
  int scene_id_; /*!< unique identifier of this scene */
};
// end of class ObservationScene

/*! \brief moving cameras need a new pose with each scene in which they are used */
typedef struct MovingCamera
{
  Camera cam;			// must hold a copy of the camera extrinsic parameters
  int scene_id;		// but copy of intrinsics may be and unused duplicate
} MovingCamera;

/*! \brief moving  need a new pose with each scene in which they are used */
typedef struct MovingTarget
{
  Target targ;		// must hold a copy of the target pose parameters,
  int scene_id;		// but point parameters may be unused duplicates
} MovingTarget;

/** \brief These blocks of data hold the ceres parameters upon which the optimizaition proceeds
 *   Static cameras have a block of parameters for their 6Dof Pose
 *                  they have a block of 4 parameters for pinhole projection model intrinsics
 *                  they have a block of 10 parameters for their distortion projection intrinsics
 *                  The 1st 4 parameters of the 10 distortion model are the same variables
 *   Moving cameras have identical sets of */
class CeresBlocks
{
public:
  /** \brief Constructor */
  CeresBlocks()
  {
  }
  ;
  /** \brief Destructor */
  ~CeresBlocks()
  {
    static_cameras_.clear();
    moving_cameras_.clear();
    static_targets_.clear();
    moving_targets_.clear();
  }
  ;

  /*! \brief adds a static camera to job's list of cameras
   *  \param camera_to_add this is the camera added to the list
   *  \return true on success
   */
  bool addStaticCamera(Camera camera_to_add);

  /*! \brief adds a static target to job's list of static targets
   *  \param target_to_add this is the target added to the list
   *  \return true on success
   */
  bool addStaticTarget(Target target_to_add);

  /*! \brief adds a moving camera to job's list of cameras
   *  \param camera_to_add this is the camera added to the list
   *  \param scene_id the scene's id, only one camera of given name existe in each scene
   *  \return true on success
   */
  bool addMovingCamera(Camera camera_to_add, int scene_id);

  /*! \brief adds a static target to job's list of static targets
   *  \param target_to_add this is the target added to the list
   *  \param scene_id the scene's id, only one target of given name exist in each scene
   *  \return true on success
   */
  bool addMovingTarget(Target target_to_add, int scene_id);

  /*! @brief gets a pointer to the intrinsic parameters of a static camera
   *  @param camera_name the camera's name
   *  @return pointer to the only existing set of intrinsics for this camera
   */
  double * getStaticCameraParameterBlockIntrinsics(std::string camera_name);

  /*! @brief gets a pointer to the intrinsic parameters of a moving camera
   *  @param camera_name the camera's name
   *  @return pointer to the intrinsic parameters for this camera, note that
   *          a number of copies of the intrinsics exist, because the camera
   *          was duplicated in each scene it made an observation
   *          however, the first scene in which the camera was used contains the
   *          only set of intrinsics to be adjusted, and therefore is the only one returned
   */
  double * getMovingCameraParameterBlockIntrinsics(std::string camera_name);

  /*! @brief gets a pointer to the extrinsics parameters of a static camera
   *  @param camera_name the camera's name
   *  @return pointer to the only existing set of extrinsics for this camera
   */
  double * getStaticCameraParameterBlockExtrinsics(std::string camera_name);

  /*! @brief gets a pointer to the extrisic parameters of a moving camera
   *  @param camera_name the camera's name
   *  @return pointer to the intrinsic parameters for this camera, note that
   *          a number of copies of the intrinsics exist, because the camera
   *          was duplicated in each scene it made an observation.
   *          Therefore, the extrinsic parameters must be the ones from
   *          the correct scene.
   */
  double * getMovingCameraParameterBlockExtrinsics(std::string camera_name, int scene_id);

  /*! @brief gets a pointer to the pose parameters of a static target
   *  @param target_name the target's name
   *  @return pointer to the only existing set of pose parameters for this target
   */
  double * getStaticTargetPoseParameterBlock(std::string target_name);

  /*! @brief gets a pointer to the position parameters of a static target's point
   *  @param target_name the target's name
   *  @param point_id id of point on target
   *  @return pointer to the only existing set of position parameters for this point
   */
  double * getStaticTargetPointParameterBlock(std::string target_name, int point_id);

  /*! @brief gets a pointer to the targets pose parameters
   *  @param target_name moving target's name
   *  @param point_id id of point on target
   *  @param scene_id id of scene where target was imaged
   *  @return pointer to the pose parameters of the target
   *          a number of copies of the pose parameters might exist for a moving target
   *          because the target was duplicated in each scene it was observed.
   *          Therefore, the desired set is the only copy with the given scene_id
   */
  double * getMovingTargetPoseParameterBlock(std::string target_name, int scene_id);

  /*! @brief gets a pointer to the point's position parameters
   *  @param target_name moving target's name
   *  @param point_id id of point on target
   *  @return pointer to the position parameters of the point
   *          a number of copies of the point parameters might exist
   *          because the target was duplicated in each scene it was observed.
   *          However, the coordinates of the point in the target frame should not change
   *          Therefore, only the first occurance (first scene) is used
   */
  double * getMovingTargetPointParameterBlock(std::string target_name, int point_id, int scene_id);

private:
  std::vector<Camera> static_cameras_; /*!< all non-moving cameras in job */
  std::vector<MovingCamera> moving_cameras_; /*! only one camera of a given name per scene */
  std::vector<Target> static_targets_; /*!< all non-moving targets in job */
  std::vector<MovingTarget> moving_targets_; /*! only one target of a given name per scene */
};

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

  /** @brief select active scene for modification
   *  @return true if sucessful
   */
  bool selectScene(int scene_id);

  /** @brief adds an observation from a particular camera to current scene
   *  @param camera_observer a pointer to the camera observer
   *  @param target a pointer to the target
   *  @return true if sucessful
   */
  bool addObservationToCurrentScene(boost::shared_ptr<DummyCameraObserver> camera_observer,
                                    boost::shared_ptr<Target> target);

  /** @brief removes all cameras and targets from current scene
   *  @return true if sucessful
   */
  bool clearCurrentScene();

  /** @brief adds a new scene to end of scene list. New scene becomes current scene.
   *  @param trig the trigger type to use for this scene
   *  @return true if successful
   */
  bool appendNewScene(Trigger trig);

  //    ::std::ostream& operator<<(::std::ostream& os, const CalibrationJob& C){ return os<< "TODO";}
private:
  std::string camera_def_file_name_; /*!< this file describes all cameras in job */
  std::string target_def_file_name_; /*!< this file describes all targets in job */
  std::string caljob_def_file_name_; /*!< this file describes all observations in job */
  std::vector<ObservationScene> scene_list_; /*!< contains list of scenes which define the job */
  int current_scene_; /*!< id of current scene under review or construction */
  std::vector<DummyCameraObserver> camera_observers_; /*!< interface to images from cameras */
  std::vector<Target> defined_target_set_; /*!< TODO Not sure if I'll use this one */
  CeresBlocks blocks_for_ceres_; /*!< This structure maintains the parameter sets for ceres */

};
//end class CalibrationJob

}// end of namespace
#endif
