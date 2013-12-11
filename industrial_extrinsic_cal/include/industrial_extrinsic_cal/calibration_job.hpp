#include <industrial_extrinsic_cal/Swri_IRD_license.h>

#ifndef CALIBRATION_JOB_HPP_
#define CALIBRATION_JOB_HPP_

#include <industrial_extrinsic_cal/basic_types.h> /** needed for Roi */
#include <industrial_extrinsic_cal/camera_observer.hpp> /** needed for CameraObserver */
#include <industrial_extrinsic_cal/ceres_excal.hpp> /** needed */

#using std::string;

namespace industrial_extrinsic_cal {

class Camera{
public:
  Camera(bool ismoving): is_moving_(ismoving);
  CameraObserver O_;
  CameraParameters P_;
  bool is_moving() return(is_moving_);
private:
  bool is_moving_;
};


/** these structures define the calibration script or set of observations being made */
typedef struct{ /** Trigger */
  int trigger_type;
  string trigger_popup_msg;
} Trigger;

typedef struct{ /** unique observation command */
  string camera_name;
  string target_name;
  Roi R;
}ObservationCmd;

class SyncObsCmd{ /** Synchronous Observation command */
public:
  SynchronousObservationCmd(Trigger Trig){Trig_ = Trig};
  void clear();
  void add_observation(Camera* C, Target* T, Roi R);
  std::vector<ObservationCmd> OC_; /** list of cameras and targets to look for  */
  Trigger Trig_;
private:
};
 

Class ObservationFrame{
 public:
  clear();
  add_camera_observation(Camera * C ,Observation O);
  std::vector<Camera*,Observation> CO;
};

class Observations{
public:
  void clear();
  bool add_observation(Camera_Observations CO, int frame_index);
  int numb_camera_frames(); /** each camera frame has 6dof */
  int numb_target_frames(); /** each target frame has 6dof */
  int numb_unique_point_locs(); /** each unique point has 6dof, (some targets are unique points) */
  int numb_uv_observations(); 
  int total_parameters(){
    return(num_camera_frames()*6 + num_target_frames()*6 + num_unique_point_locs()*3);
  }
  ;/** @brief finds the number of free parameters represented  */
private:
  
}

class CalibrationJob{
public:
  CalibrationJob CJ(string camerafn, string targetfn, string obsfn, string jobfn); 
  bool load();			/** @brief reads all input files to create a calibration job */
                                /** @return true if successful */
  bool store();			/** @brief stores calibration job as 4 files */
                                /** @return true if successful */
  bool run();			/** @brief runs both data collection and optimization */
                                /** @return true if successful */
  bool run_observations();	/** @brief runs the data collection portion of the job */
                                /** @return true if successful */
  bool run_optimization();	/** @brief runs the optimization portion of the job */
                                /** @return true if successful */
  bool add_camera(Camera C);	/** @brief Adds a new camera with a subscriber */
				/** @parameter C: A camera */
                                /** @return true if successful */
  bool add_target(target T);	/** @brief Adds a new target to target list */
                                /** @parameter T: A target */
  bool add_command(SynObsCmd S); /** @brief Adds a new Observation  */
  /** @parameter  */
  /** @return true if successful */
  bool clear_cameras();		/** @brief removes all cameras from job */
                                /** @return true if successful */
  bool clear_targets();		/** @brief */
                                /** @return true if successful */
  bool clear_commands();	/** @brief clear all commands */
                                /** @return true if successful */
  bool clear_observation_data(); /** @brief clears all previously collected data */
  /** @return true if successful */
  bool add_observation_frame();
  bool add_camera_observation(int frame,Observation O, Camera *C);
private:
  string Camera_def_file_name_;
  string Target_def_file_name_;
  string Observations_def_file_name_;
  std::vector<Camera> Cams_;
  std::vector<Targets> Targs_;
  std::vector<SynchronousObservationCmd> OC_;
  std::vector<ObservationFrame> OF_;
};

int main(int argc, char** argv) 
{
  google::InitGoogleLogging(argv[0]);
  if (argc != 4) {
    std::cerr << "usage: extrinsic_excal <cameras> <targets>  <job>\n";
    return 1;
  }
  /** this code peforms extrinsic calibration defined by the input files */
  /** ultimately, a set of Qt widgets will allow one to create these widgets */

  string cameras_file      = argv(1);
  string targets_file      = argv(2);
  string observation_file  = argv(3);

  
  Calibration_Job CJ(camera_file,targets_file,observation_file);
  
  CJ.load();
  CJ.run_observations();
  CJ.run_optimization();

  return 0;
}

} // end of namespace
#endif
