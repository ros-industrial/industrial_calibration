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

#ifndef CAMERA_OBSERVER_HPP_
#define CAMERA_OBSERVER_HPP_

#include <industrial_extrinsic_cal/basic_types.h> /* Target,Roi,Observation,CameraObservations */
#include <industrial_extrinsic_cal/target.h>      /* Roi,Observation,CameraObservations */
#include <industrial_extrinsic_cal/ceres_costs_utils.h>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>

namespace industrial_extrinsic_cal
{
/*! \brief An observation is the x,y image location of a target's point in an image*/
typedef struct
{
  boost::shared_ptr<Target> target; /**< pointer to target who's point is observed */
  int point_id;                     /**< point's id in target's point array */
  double image_loc_x;               /**< target point was found at image location x */
  double image_loc_y;               /**< target point image location y */
  Cost_function cost_type;          /**< type of cost associated with this observation */
  Pose6d intermediate_camera_frame; /**< transform from camera_mounting_frame_ to ref_frame_ */
  Pose6d intermediate_target_frame; /**< transform form ref_frame_ to target_mounting_frame_ */
} Observation;

/*! \brief A vector of observations made by a single camera of posibly multiple targets */
typedef std::vector<Observation> CameraObservations;

/** @brief A camera observer is an object which given a set of targets and regions of interest, it provides observations
 *              of those targets. A camera obsever is configured with targets, then triggered, once observations are
 * done, they
 *              may be collected with the getter function.
 */
class CameraObserver
{
public:
  /** @brief Default destructor */
  virtual ~CameraObserver(){};

  /** @brief add a target to look for */
  /** @param targ a target to look for */
  /** @param roi Region of interest for target */
  virtual bool addTarget(boost::shared_ptr<Target> targ, Roi& roi, Cost_function cost_type) = 0;

  /** @brief remove all targets */
  virtual void clearTargets() = 0;

  /** @brief clear all previous observations */
  virtual void clearObservations() = 0;

  /** @brief return observations */
  /** @param output all observations of targets defined */
  virtual int getObservations(CameraObservations& camera_observations) = 0;

  /** @brief print this object TODO */
  virtual void triggerCamera() = 0;

  /** @brief tells when camera has completed its observations */
  virtual bool observationsDone() = 0;

  /** @brief pushes the camera_info out to the camera driver */
  /** @param fx the focal length in x*/
  /** @param fy the focal length in y*/
  /** @param cx the optical center in x*/
  /** @param fx the optical center in y*/
  /** @param k1 distortion 2nd order radial*/
  /** @param k2 distortion 4th order radial */
  /** @param k3 distortion 6th order radial */
  /** @param p1 distortion decentering */
  /** @param p2 distortion decentering */
  /** @returns true if successful */
  virtual bool pushCameraInfo(double& fx, double& fy, double& cx, double& cy, double& k1, double& k2, double& k3,
                              double& p1, double& p2) = 0;

  /** @brief pulls the camera_info in from the camera driver */
  /** @param fx the focal length in x*/
  /** @param fy the focal length in y*/
  /** @param cx the optical center in x*/
  /** @param fx the optical center in y*/
  /** @param k1 distortion 2nd order radial*/
  /** @param k2 distortion 4th order radial */
  /** @param k3 distortion 6th order radial */
  /** @param p1 distortion decentering */
  /** @param p2 distortion decentering */
  /** @returns true if successful */
  virtual bool pullCameraInfo(double& fx, double& fy, double& cx, double& cy, double& k1, double& k2, double& k3,
                              double& p1, double& p2) = 0;

  /** @brief pulls the camera_info in from the camera driver */
  /** @param fx the focal length in x*/
  /** @param fy the focal length in y*/
  /** @param cx the optical center in x*/
  /** @param fx the optical center in y*/
  /** @param k1 distortion 2nd order radial*/
  /** @param k2 distortion 4th order radial */
  /** @param k3 distortion 6th order radial */
  /** @param p1 distortion decentering */
  /** @param p2 distortion decentering */
  /** @param width image width */
  /** @param height image height */
  /** @returns true if successful */
  virtual bool pullCameraInfo(double& fx, double& fy, double& cx, double& cy, double& k1, double& k2, double& k3,
                              double& p1, double& p2, int& width, int& height) = 0;


  /** @brief sets directory for saving and restoring images */
  /** @param dir_name, the directory path */
  void set_image_directory(std::string dir_name){ image_directory_ = dir_name;};

  /** @brief gets directory for saving and restoring images */
  /** @param dir_name, the directory path */
  std::string get_image_directory(){ return(image_directory_);};

  /** @brief saves the latest image to the image_directory_ with the provided filename
   *  if the filename is a null string, the name is built 
   *  from the image_directory_/camera_name_ + underscore + scene_number.jpg
  */
  /** @param scene, the scene number */
  /** @param filename, the file name */
  bool save_current_image(int scene, std::string& filename)
  {
    std::string full_file_path_name;
    char scene_chars[8];
    sprintf(scene_chars,"_%03d.jpg",scene);
    if(filename == ""){ // build file name from image_directory_, 
      full_file_path_name  = image_directory_ + std::string("/") +  camera_name_ + std::string(scene_chars);
    }
    else{
      full_file_path_name  = image_directory_ + "/" +  filename;
    }
    ROS_INFO("saving camera: %s image with filepath %s",camera_name_.c_str(),full_file_path_name.c_str());
    if(!cv::imwrite(full_file_path_name, getCurrentImage()))
      {
	ROS_ERROR("couldn't save camera %s image with filepath %s",camera_name_.c_str(),full_file_path_name.c_str());
	return(false);
      }
      return(true);

  };

  /** @brief loads the latest image from the image_directory_ with the provided filename
   *  if the filename is a null string, the filename is checked
   *  to see if it exists under image_directory_
  */
  /** @param scene, the scene number */
  /** @param filename, the file name */
  bool load_current_image(int scene, std::string& filename)
  {
    std::string full_image_file_path_name;
    char scene_chars[8];
    sprintf(scene_chars,"_%03d.jpg",scene);
    if(filename == ""){ // build file name from image_directory_,
      full_image_file_path_name  = image_directory_ + std::string("/") +  camera_name_ + std::string(scene_chars);
    }
    else{
      full_image_file_path_name  = image_directory_ + "/" +  filename;
    }
    if(exists_test(full_image_file_path_name)){
      setCurrentImage(cv::imread(full_image_file_path_name));
      return(true);
    }
    else{
      ROS_ERROR("failed to find %s", full_image_file_path_name.c_str());
      return (false);
    }
  };
  inline bool exists_test (const std::string& name) {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
  }; //end exists_test

  /** @brief load image from the image_directory_ with the provided filename
   *  if the filename is a null string, the name is built 
   *  from the image_directory_/camera_name_ + underscore + scene_number.jpg
  */
  /** @param scene, the scene number */
  /** @param filename, the file name */
  bool load_image(int scene, std::string& filename)
  {
    std::string full_file_path_name;
    char scene_chars[8];
    sprintf(scene_chars,"_%03d.jpg",scene);
    if(filename == ""){ // build file name from image_directory_, 
      full_file_path_name  = image_directory_ + "/" +  camera_name_ + std::string(scene_chars);
    }
    else{
      full_file_path_name  = image_directory_ + "/" +  filename;
    }
    setCurrentImage(cv::imread(full_file_path_name));
    return(true);
  };

  /**
   *  @brief get current image
   *  @return the most recent image
   */
  cv::Mat getCurrentImage()
  {
    return (last_raw_image_);
  }
  /**
   *  @brief set current image
   *  @param image set the current image to this one
   */
  virtual void setCurrentImage(const cv::Mat &image)
  {
    last_raw_image_ = image.clone();
    new_image_collected_ = true;
  }

  bool checkObservationProclivity(CameraObservations& CO)
  {
    /* compute proclivity from a few specific points in the observation */
    cv::Mat Alpha(8,1, CV_64F);
    cv::Mat A(18,8, CV_64F); // use twice as many as necessary
    cv::Mat B(18,1, CV_64F);
    boost::shared_ptr<Target> target = CO[0].target;

    // determine rows and cols of target
    int rows, cols;
    target->getRowsCols(rows,cols);

    std::vector<int> selected_point_index;
    selected_point_index.push_back(0); // upper left point
    selected_point_index.push_back(rows-1); // upper right point 
    selected_point_index.push_back(rows*cols - cols -1); // lower left point 
    selected_point_index.push_back(rows*cols -1); // lower right point
    selected_point_index.push_back(1);  // nearby to point1
    selected_point_index.push_back(rows-2);// nearby to point2
    selected_point_index.push_back(rows*cols - cols -2); // nearby to point3
    selected_point_index.push_back(rows*cols -2); // nearby to point4
    selected_point_index.push_back(rows*cols/2); // middle point
    // build matrix of type Ax=b where A x is the unknown elements of the proclivity matrix "alpha"
    int row = 0;
    for(int i=0; i<(int) selected_point_index.size(); i++){
      int pi = CO.at(selected_point_index[i]).point_id; // We expect that selected_point_index = pi, but not sure
      double xi = target->pts_[pi].x;
      double yi = target->pts_[pi].y;
      double Ui = CO.at(pi).image_loc_x;
      double Vi = CO.at(pi).image_loc_y;
      A.at<double>(row,0) = -xi;
      A.at<double>(row,1) = -yi;
      A.at<double>(row,2) = -1.0;
      A.at<double>(row,3) = 0.0;
      A.at<double>(row,4) = 0.0;
      A.at<double>(row,5) = 0.0;
      A.at<double>(row,6) = Ui*xi;
      A.at<double>(row,7) = Ui*yi;
      B.at<double>(row)   = -Ui;
      row++;
      A.at<double>(row,0) = 0.0;
      A.at<double>(row,1) = 0.0;
      A.at<double>(row,2) = 0.0;
      A.at<double>(row,3) = -xi;
      A.at<double>(row,4) = -yi;
      A.at<double>(row,5) = -1.0;
      A.at<double>(row,6) = Vi*xi;
      A.at<double>(row,7) = Vi*yi;
      B.at<double>(row)   = -Vi;
      row++;
    }
    cv::solve(A,B,Alpha, cv::DECOMP_SVD);

    // construct the Proclivity matrix from Alpha
    cv::Mat P(3,3,CV_64F);
    P.at<double>(0,0) = Alpha.at<double>(0);
    P.at<double>(0,1) = Alpha.at<double>(1);
    P.at<double>(0,2) = Alpha.at<double>(2);
    P.at<double>(1,0) = Alpha.at<double>(3);
    P.at<double>(1,1) = Alpha.at<double>(4);
    P.at<double>(1,2) = Alpha.at<double>(5);
    P.at<double>(2,0) = Alpha.at<double>(6);
    P.at<double>(2,1) = Alpha.at<double>(7);
    P.at<double>(2,2) = 1.0;

    /*
    ROS_ERROR("Proclivity=");
    ROS_ERROR("[ %8.3lf %8.3lf %8.3lf ",  P.at<double>(0,0), P.at<double>(0,1), P.at<double>(0,2));
    ROS_ERROR("  %8.3lf %8.3lf %8.3lf ",  P.at<double>(1,0), P.at<double>(1,1), P.at<double>(1,2));
    ROS_ERROR("  %8.3lf %8.3lf %8.3lf ]", P.at<double>(2,0), P.at<double>(2,1), P.at<double>(2,2));
    */
    // check the proclivity of every observation
    bool rtn = true;
    double ave_error = 0.0;
    for(int i=0; i<(int) CO.size(); i++){
      int pi = CO[i].point_id;
      double Ui = CO.at(pi).image_loc_x;
      double Vi = CO.at(pi).image_loc_y;
      double xi = target->pts_[pi].x;
      double yi = target->pts_[pi].y;
      double ki = 1.0/(Alpha.at<double>(6)*xi + Alpha.at<double>(7)*yi + 1.0);
      cv::Mat UV(3,1, CV_64F);
      cv::Mat X(3,1, CV_64F);
      X.at<double>(0) = xi;
      X.at<double>(1) = yi;
      X.at<double>(2) = 1.0;
      UV = ki*P*X;
      double EU = Ui - UV.at<double>(0);
      double EV = Vi - UV.at<double>(1);
      ave_error += sqrt(EU*EU + EV*EV);
      if(fabs(EU)>5.0 || fabs(EV)>5.0){
	ROS_ERROR("pi = %d %8.3lf %8.3lf Ui Vi = %8.3lf %8.3lf UV = %8.3lf %8.3lf Error: %8.3lf %8.3lf",pi, xi, yi,
		  Ui, Vi,
		  UV.at<double>(0), UV.at<double>(1),
		  EU, EV);
	rtn = false;
      }
    }// done checking proclivities
    ave_error = ave_error/(int) CO.size();
    ROS_WARN("average proclivity error = %8.3lf",ave_error);
    return(rtn);
  };

  
  cv::Mat last_raw_image_; /**< the image last received */
  std::string image_directory_; /*!< string directory for saving and loading images */
  std::string camera_name_; /*!< string camera_name_ unique name of a camera */
  bool new_image_collected_; /*!< flag indicating that there is a new image ready for processing */

  /** @brief print this object TODO */
  //    virtual ::std::ostream& operator<<(::std::ostream& os, const CameraObserver& camera);
};

}  // end of namespace
#endif
