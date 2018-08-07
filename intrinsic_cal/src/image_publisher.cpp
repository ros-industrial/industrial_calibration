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
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  std::string node_name = ros::this_node::getName();

  image_transport::ImageTransport it(nh);

  image_transport::Publisher pub1 = it.advertise("camera1/rgb/image_rect_color", 1);
  image_transport::Publisher pub2 = it.advertise("camera2/rgb/image_rect_color", 1);
  image_transport::Publisher pub3 = it.advertise("camera3/rgb/image_rect_color", 1);
  image_transport::Publisher pub4 = it.advertise("camera4/rgb/image_rect_color", 1);
  image_transport::Publisher pub5 = it.advertise("camera5/rgb/image_rect_color", 1);
  image_transport::Publisher pub6 = it.advertise("camera6/rgb/image_rect_color", 1);
  image_transport::Publisher pub7 = it.advertise("camera7/rgb/image_rect_color", 1);
  image_transport::Publisher pub8 = it.advertise("camera8/rgb/image_rect_color", 1);


  std::string image_directory;
  if(!nh.getParam(node_name + "/image_directory", image_directory)){
    ROS_ERROR("MUST SET ros node parameter image_directory");
    exit(1);
  }
  else{
    ROS_INFO("Using Image Directory: %s",image_directory.c_str());
  }

  std::string image_1 = image_directory + "/camera1_cal.jpg";
  std::string image_2 = image_directory + "/camera2_cal.jpg";
  std::string image_3 = image_directory + "/camera3_cal.jpg";
  std::string image_4 = image_directory + "/camera4_cal.jpg";
  std::string image_5 = image_directory + "/camera5_cal.jpg";
  std::string image_6 = image_directory + "/camera6_cal.jpg";
  std::string image_7 = image_directory + "/camera7_cal.jpg";
  std::string image_8 = image_directory + "/camera8_cal.jpg";

  cv::Mat image1= cv::imread(image_1, CV_LOAD_IMAGE_COLOR);
  cv::Mat image2= cv::imread(image_2, CV_LOAD_IMAGE_COLOR);
  cv::Mat image3= cv::imread(image_3, CV_LOAD_IMAGE_COLOR);
  cv::Mat image4= cv::imread(image_4, CV_LOAD_IMAGE_COLOR);
  cv::Mat image5= cv::imread(image_5, CV_LOAD_IMAGE_COLOR);
  cv::Mat image6= cv::imread(image_6, CV_LOAD_IMAGE_COLOR);
  cv::Mat image7= cv::imread(image_7, CV_LOAD_IMAGE_COLOR);
  cv::Mat image8= cv::imread(image_8, CV_LOAD_IMAGE_COLOR);

  //  cv::namedWindow("test");
  //  cv::imshow("test",image1);
  //  cv::waitKey(10000);

  std_msgs::Header header;
  int seq = 1;
  header.seq = seq;
  header.frame_id = "world_frame";
  header.stamp = ros::Time::now();
  std::string encoding("bgr8");
  cv_bridge::CvImage  cv1(header, encoding, image1);
  cv_bridge::CvImage  cv2(header, encoding, image2);
  cv_bridge::CvImage  cv3(header, encoding, image3);
  cv_bridge::CvImage  cv4(header, encoding, image4);
  cv_bridge::CvImage  cv5(header, encoding, image5);
  cv_bridge::CvImage  cv6(header, encoding, image6);
  cv_bridge::CvImage  cv7(header, encoding, image7);
  cv_bridge::CvImage  cv8(header, encoding, image8);
  
  ros::Rate loop_rate(5);
  while (nh.ok()) {
    header.seq = seq++;
    header.frame_id = "world_frame";
    header.stamp = ros::Time::now();
    header.frame_id = "/camera1_rgb_optical_frame";
    cv1.header = header;
    header.frame_id = "/camera2_rgb_optical_frame";
    cv2.header = header;
    header.frame_id = "/camera3_rgb_optical_frame";
    cv3.header = header;
    header.frame_id = "/camera4_rgb_optical_frame";
    cv4.header = header;
    header.frame_id = "/camera5_rgb_optical_frame";
    cv5.header = header;
    header.frame_id = "/camera6_rgb_optical_frame";
    cv6.header = header;
    header.frame_id = "/camera7_rgb_optical_frame";
    cv7.header = header;
    header.frame_id = "/camera8_rgb_optical_frame";
    cv8.header = header;
    pub1.publish(cv1.toImageMsg());
    pub2.publish(cv2.toImageMsg());
    pub3.publish(cv3.toImageMsg());
    pub4.publish(cv4.toImageMsg());
    pub5.publish(cv5.toImageMsg());
    pub6.publish(cv6.toImageMsg());
    pub7.publish(cv7.toImageMsg());
    pub8.publish(cv8.toImageMsg());
    ros::spinOnce();
    loop_rate.sleep();
  }
}
