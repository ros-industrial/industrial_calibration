/*
 * ros_camera_observer.h
 *
 *  Created on: Dec 18, 2013
 *      Author: cgomez
 */


#ifndef ROS_CAMERA_OBSERVER_H_
#define ROS_CAMERA_OBSERVER_H_

#include <industrial_extrinsic_cal/camera_observer.hpp> /* CameraObserver class */
#include <industrial_extrinsic_cal/basic_types.h> /* Target,Roi,Observation,CameraObservations */

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>

namespace industrial_extrinsic_cal {

	/**
	 * @brief enumerator containing three options for the type of pattern to detect
	 */
    enum pattern_options_
    {
    	Chessboard, CircleGrid, AsymmetricCircleGrid
    };
 class ROSCameraObserver: public CameraObserver
 {
  public:

    /** @brief constructor */
    /** @param source_name name of image topic */
    ROSCameraObserver(std::string image_topic="");

    /** @brief Default destructor */
    ~ROSCameraObserver(){};

    /** @brief add a target to look for */
    /** @param targ a target to look for */
    /** @param roi Region of interest for target */
    void addTarget(boost::shared_ptr<Target> targ, Roi roi);

    /** @brief remove all targets */
    void clearTargets();

    /** @brief clear all previous observations */
    void clearObservations();

    /** @brief return observations */
    /** @param output all observations of targets defined */
    int getObservations(CameraObservations &camera_observations);


  private:

    //ROS specific functions
    /**
     * @brief
     * @param image_msg
     */
    void infoCallback(const sensor_msgs::ImageConstPtr& image_msg);

    /**
     *  @brief pattern to be used in detection
     */
    pattern_options_ pattern_;
    /**
     *  @brief topic name for image which is input at constructor
     */
    std::string image_topic_;
    /**
     *  @brief cropped image based on original image and region of interest
     */
    cv::Mat image_roi_;
    std::vector<cv::Point2f> observation_pts_;
    boost::shared_ptr<Target> instance_target_;

    //ROS specific params
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher results_pub_;
	tf::TransformListener tf_listener_;
	tf::TransformBroadcaster tf_broadcaster_;

    // Structures for interacting with ROS/CV messages
    cv_bridge::CvImagePtr input_bridge_;
	cv_bridge::CvImagePtr output_bridge_;
	cv_bridge::CvImagePtr out_bridge_;

  };

}//end industrial_extrinsic_cal namespace

#endif /* ROS_CAMERA_OBSERVER_H_ */
