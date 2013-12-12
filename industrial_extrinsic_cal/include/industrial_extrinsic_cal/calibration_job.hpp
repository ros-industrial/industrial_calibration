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
#include <industrial_extrinsic_cal/basic_types.h> /** needed for Roi */
#include <industrial_extrinsic_cal/camera_observer.hpp> /** needed for CameraObserver */
#include <industrial_extrinsic_cal/ceres_excal.hpp> /** needed */

#using std::string;
#using boost::shared_ptr;

namespace industrial_extrinsic_cal {

  class Camera{
  public:
    Camera(bool ismoving): is_moving_(ismoving);
    CameraObserver o;
    CameraParameters p;
    bool is_moving() return(is_moving_);
    ::std::ostream& operator<<(::std::ostream& os, const Camera& C){ return os<< "TODO";}
  private:
    bool is_moving_;
    std::string name;
  };


  /** these structures define the calibration script or set of observations being made */
  typedef struct{ /** Trigger */
    int trigger_type;
    string trigger_popup_msg;
  } Trigger;

  typedef struct{ /** unique observation command */
    string camera_name;
    string target_name;
    Roi r;
  }ObservationCmd;

  /*! \brief a command to take a set of observations from a group of cameras upon a trigger event */
  class SyncObsCmd{ /** Synchronous Observation command */
  public:
    SynchronousObservationCmd(Trigger Trig){Trig_ = Trig};
    void clear();
    void add_observation(shared_ptr<Camera> c, shared_pter<Target> T, Roi R);
    std::vector<ObservationCmd> oc; /** list of cameras and targets to look for  */
    Trigger trig;
  private:
  };
 

  /*! \brief an observation set from one camera of a static scene */
  typedef struct{
    share_ptr c;
    Observation o;
  }CameraObservation;
  /*! \brief a collection of observations all taken from a single static scene */
  Class ObservationScene{
  public:
    clear();
    add_camera_observation(shared_ptr<Camera> c ,Observation o);
    std::vector< CameraObservation> co;
    ::std::ostream& operator<<(::std::ostream& os, const ObservationScene& C){ return os<< "TODO";}
  };

  /*! \brief a collection of observations and functions on that collection */
  class Observations{
  public:
    void clear();
    bool addObservation(Camera_Observations CO, int scene_index);
    int numbCameraFrames(); /** each camera frame has 6dof */
    int numbTargetFrames(); /** each target frame has 6dof */
    int numbUniquePointLocs(); /** each unique point has 6dof, (some targets are unique points) */
    int numbUvObservations(); 
    int totalParameters(){
      ::std::ostream& operator<<(::std::ostream& os, const Observations& C){ return os<< "TODO";}
      return(numCameraFrames()*6 + numTargetFrames()*6 + numUniquePointLocs()*3);
    }
    ;/** @brief finds the number of free parameters represented  */
  private:
  
  };
  typedef struct{
    Camera cam;
    int    scene_id;
  }MovingCamera;

  typedef struct{
    Target targ;
    int    scene_id;
  }MovingTarget;
  
  class CeresBlocks{
  public:
    bool add_static_camera(Camera camera_to_add, int scene_id);
    bool add_static_target(Target target_to_add, int scene_id);
    bool add_moving_camera(MovingCamera camera_to_add, int scene_id);
    bool add_moving_target(MovingTarget target_to_add, int scene_id);
    double * get_static_parameter_block(
  private:
    std::vector<Camera> static_cameras_;
    std::vector<MovingCamera> moving_cameras_;
    std::vector<Target> static_targets_;
    std::vector<MovingTarget> moving_targets_;
  };
    

  \*! \brief defines and exececutes the calibration script */
  class CalibrationJob{
  public:
    CalibrationJob CJ(string camerafn, string targetfn, string obsfn, string jobfn); 
    bool load();			/** @brief reads all input files to create a calibration job */
    /** @return true if successful */
    bool store();			/** @brief stores calibration job as 4 files */
    /** @return true if successful */
    bool run();			/** @brief runs both data collection and optimization */
    /** @return true if successful */
    bool runObservations();	/** @brief runs the data collection portion of the job */
                                /** @return true if successful */
    bool runOptimization();	/** @brief runs the optimization portion of the job */
                                /** @return true if successful */
    bool addCamera(Camera C);	/** @brief Adds a new camera with a subscriber */
    /** @parameter C: A camera */
    /** @return true if successful */
    bool addtarget(target T);	/** @brief Adds a new target to target list */
    /** @parameter T: A target */
    bool addCommand(SynObsCmd S); /** @brief Adds a new Observation  */
    /** @parameter  */
    /** @return true if successful */
    bool clearCameras();		/** @brief removes all cameras from job */
    /** @return true if successful */
    bool clearTargets();		/** @brief */
    /** @return true if successful */
    bool clearCommands();	/** @brief clear all commands */
                                /** @return true if successful */
    bool clearObservation_data(); /** @brief clears all previously collected data */
    /** @return true if successful */
    bool addObservationScene();
    bool addCameraObservation(int Scene_id,Observation O, Camera *C);
    ::std::ostream& operator<<(::std::ostream& os, const CalibrationJob& C){ return os<< "TODO";}
  private:
    string camera_def_file_name_;
    string target_def_file_name_;
    string observations_def_file_name_;
    std::vector<Camera> cams_;
    std::vector<Targets> targs_;
    std::vector<SynchronousObservationCmd> oc_;
    std::vector<ObservationScene> os_;
  };


} // end of namespace
#endif
