/*
 * test_ros_cam_obs.cpp
 *
 *  Created on: Dec 19, 2013
 *      Author: cgomez
 */



#include <ros/ros.h>
#include <industrial_extrinsic_cal/ros_camera_observer.h>

using industrial_extrinsic_cal::ROSCameraObserver;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "my_node_name");
	ros::NodeHandle nh;
	std::string topic="/camera/rgb/image_rect_color";

	 ROSCameraObserver cam_observer(topic);

	 boost::shared_ptr<industrial_extrinsic_cal::Target> target (new industrial_extrinsic_cal::Target);
	 industrial_extrinsic_cal::Roi roi;
	 target->target_type=0;
	 target->is_moving=false;
	 target->target_name="checkerboard";
	 target->checker_board_parameters.pattern_rows=11;
	 target->checker_board_parameters.pattern_cols=11;
	 //target->target_name="circlegrid";
	 //target->circle_grid_parameters.pattern_rows=31;
	 //target->circle_grid_parameters.pattern_cols=30;
	 //target->circle_grid_parameters.is_symmetric=true;
	 //target->pts[0]=0;
	 //target->pose=NULL;
	 roi.x_min=100;
	 roi.y_min=70;
	 roi.x_max=460;
	 roi.y_max=430;

	 if (cam_observer.addTarget(target , roi))
	 {
		 ROS_INFO_STREAM("Added target successfully");
	 }

	 industrial_extrinsic_cal::CameraObservations camera_obs;
	 //cam_observer.getObservations(camera_obs);
	 if (cam_observer.getObservations(camera_obs))
	 {
		 ROS_INFO_STREAM("Success!");
	 }

}
