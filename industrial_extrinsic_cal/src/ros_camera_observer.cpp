/*
 * ros_camera_observer.cpp
 *
 *  Created on: Dec 18, 2013
 *      Author: cgomez
 */


#include <industrial_extrinsic_cal/ros_camera_observer.h>

namespace industrial_extrinsic_cal {


ROSCameraObserver::ROSCameraObserver(std::string camera_topic)
{
	image_topic_=camera_topic;
	image_sub_ = nh_.subscribe(camera_topic, 1, &ROSCameraObserver::infoCallback, this);
	ROS_INFO_STREAM("ROSCameraObserver subscribing to image on "<<camera_topic);

	results_pub_ = nh_.advertise<sensor_msgs::Image>("observer_results_image", 100);
	pattern_ = Chessboard;
}

void ROSCameraObserver::infoCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
	sensor_msgs::ImageConstPtr recent_image =  ros::topic::waitForMessage<sensor_msgs::Image>(image_topic_);
	if (!recent_image)
	{
		ROS_ERROR("Failed to input image from topic");
		return;
	}
}

void ROSCameraObserver::addTarget(boost::shared_ptr<Target> targ, Roi roi)
{
	ROS_INFO_STREAM("Target type: "<<targ->target_type);
	instance_target_=targ;
	ROSCameraObserver::pattern_ == targ->target_type;

	if (targ->target_type !=0 && targ->target_type !=1 && targ->target_type!= 2)
	{
		ROS_WARN_STREAM("Unknown pattern, based on target_type");
		ROSCameraObserver::pattern_ = Chessboard;
	}

	sensor_msgs::ImageConstPtr recent_image =  ros::topic::waitForMessage<sensor_msgs::Image>(image_topic_);
	try
	{
		input_bridge_ = cv_bridge::toCvCopy(recent_image, "mono8");
		output_bridge_ = cv_bridge::toCvCopy(recent_image, "bgr8");
		out_bridge_ = cv_bridge::toCvCopy(recent_image, "mono8");
		ROS_INFO_STREAM("cv image created based on ros image");
	}
	catch (cv_bridge::Exception& ex)
	{
		ROS_ERROR("Failed to convert image");
		return;
	}

	cv::Rect input_ROI(roi.x_min, roi.y_min, roi.x_max-roi.x_min, roi.y_max-roi.y_min);//Rect takes in x,y,width,height
	//ROS_INFO_STREAM("image ROI region created");
	if (input_bridge_->image.cols < input_ROI.width || input_bridge_->image.rows < input_ROI.height)
	{
		ROS_ERROR_STREAM("ROI too big for image size");
	}

	image_roi_ = input_bridge_->image(input_ROI);


	output_bridge_->image=output_bridge_->image(input_ROI);
	out_bridge_->image=image_roi_;
	ROS_INFO_STREAM("output image size: " <<output_bridge_->image.rows<<" x "<<output_bridge_->image.cols);
	results_pub_.publish(out_bridge_->toImageMsg());
}

void ROSCameraObserver::clearTargets()
{

}

void ROSCameraObserver::clearObservations()
{

}

int ROSCameraObserver::getObservations(CameraObservations &cam_obs)
{
	bool successful_find =false;

	switch (pattern_)
	{
	case Chessboard:
		successful_find = cv::findChessboardCorners(image_roi_, cv::Size(11,11), observation_pts_, cv::CALIB_CB_ADAPTIVE_THRESH);
		break;
	case CircleGrid:
		successful_find = cv::findCirclesGrid(image_roi_, cv::Size(11,11), observation_pts_, cv::CALIB_CB_SYMMETRIC_GRID);
		break;
	case AsymmetricCircleGrid:
		successful_find= cv::findCirclesGrid(image_roi_, cv::Size(11,11), observation_pts_,
				cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
		break;
	}


	ROS_INFO_STREAM("Number of points found on board: "<<observation_pts_.size());
	cam_obs.observation.resize(observation_pts_.size());
	for (int i = 0; i<observation_pts_.size() ; i++)
	{
		ROS_INFO_STREAM("i="<<i);
		cam_obs.observation.at(i).target=instance_target_;
		cam_obs.observation.at(i).point_id=i;
		cam_obs.observation.at(i).image_loc_x=observation_pts_.at(i).x;
		cam_obs.observation.at(i).image_loc_y=observation_pts_.at(i).y;
	}

	if (successful_find)
	{
		return 1;
	}
}

}//industrial_extrinsic_cal
