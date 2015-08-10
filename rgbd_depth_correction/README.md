# rgbd_depth_correction

This package was designed with RGBD cameras in mind, specifically the ASUS Prime sensor,
where the point cloud often has some inherent depth error in the point cloud due to slight
errors in the depth registration.  This package attempts to correct the depth error by 
creating a nodelet which listens to the point cloud, applies a depth correction factor,
and republishes the corrected point cloud on a new topic.  This requires calibration of the
camera intrinsics as well as calculating the depth correction factor for each camera individually
since each camera is unique.  This package depends on the industrial_extrinsic_cal package
(http://wiki.ros.org/industrial_extrinsic_cal).  Below are the details for running the 
calibration and correction node/nodelet.

## Camera Intrinsic Calibration

Use the ROS camera calibration package (http://wiki.ros.org/camera_calibration) to find the RGB
and IR camera intrinsic parameters.  Make sure to over-sample the calibration target, especially
for locations where the calibration target is at large skew angles and/or at the edge of the field
of view of the camera.  If intrinsic calibration is performed with too few images, then the results
of the intrinsic calibration process may not be accurate.  Inaccurate intrinsic calibration will 
show up later on when performing extrinsic calibration and will result in poor point cloud alignment
especially in the x and y axes (with respect to the camera rgb frame).

 1. $ roslaunch openni2_launch openni2.launch device_id:=#@#
 2. $ rosrun camera_calibration cameracalibrator.py image:=/camera/rgb/image_raw camera:=/camera/rgb --size <MxN> --pattern circles -q <spacing>
 3. Move camera or target to collect images at a variety of poses.  Oversample at large skew angles and at edges of field of view
 4. Save calibration results
 5. Copy projection matrix and distortion matrix to extrinsic calibration cameras.yaml file as well as to the calibration.launch file (target
   locator parameters)

When calibrating the IR camera, make sure to cover up the IR projector on the camera and to use an
IR source (incandescent light bulb, heat lamp, etc.) to illuminate the calibration target.

 1. $ roslaunch openni2_launch openni2.launch device_id:=#@#
 2. $ rosrun camera_calibration cameracalibrator.py image:=/camera/ir/image camera:=/camera/ir --size <MxN> --pattern circles -q <spacing>
 3. Move camera or target to collect images at a variety of poses.  Oversample at large skew angles and at edges of field of view
 4. Save calibration results

## Depth Calibration

 1. Copy RGB projection matrix and distortion matrix parameters from camera calibration file to correction.launch and camera_scene_cameras.yaml file
 2. Place calibration target on a large, flat surface.  The surface should be large enough that it fills the field of view of the camera at all
    distances from 2-8ft.
 3. Place camera at a distance of 5ft from the target and as close to normal to the surface as is possible.  Record the first set of images for depth
    correction using the /pixel_depth_correction service.  If target cannot be found, adjust camera location and/or lighting until target is found.
 4. Place camera such that the target fills the field of view of the camera.  Record one image using the /store_cloud service.  Move camera away from
    target at approximately 1ft increments, recording a new image at each location.
 5. Execute final depth calibration optimization after all images are taken using the /depth_calibration service.

## Depth Correction Nodelet

 1. $ roslaunch rgbd_depth_correction correction.launch
 2. Perform extrinsic calibration
   - $ rosservice call /calibration_service "allowable_cost_per_observation: 1.0"

