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

#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::string;

class InkTest
{
public:
   InkTest(ros::NodeHandle nh): it_(nh)
   {
     nh_ = nh;
     ros::NodeHandle pnh("~");

     if(!pnh.getParam( "image_topic", image_topic_)){
     image_topic_ = "Basler1/image_rect";
     ROS_ERROR("Must set param:  image_topic");
   }
   if(!pnh.getParam( "image_height", image_height_)){
     image_height_ = 1080;
     ROS_ERROR("Must set param: image_height");
   }
   if(!pnh.getParam( "image_width", image_width_)){
     image_width_ = 1920;
     ROS_ERROR("Must set param: image_width");
   }
   has_roi_=true;
   int roi_minx, roi_miny, roi_maxx, roi_maxy;

   if(!pnh.getParam( "image_roi_minx", roi_minx) ||
      !pnh.getParam( "image_roi_miny", roi_miny) ||
      !pnh.getParam( "image_roi_maxx", roi_maxx) ||
      !pnh.getParam( "image_roi_maxy", roi_maxy) ){
     has_roi_=false;
   }
   if(has_roi_){
     my_roi_ = cv::Rect(roi_minx,roi_miny,roi_maxx-roi_minx,roi_maxy-roi_miny);
   }
   std::string t_name;
   if(pnh.getParam( "template1", t_name)){
      t1_ = cv::imread(t_name.c_str(),0);
   }
   else{
     ROS_ERROR("Must set param: template1");
   }
   if(pnh.getParam( "template2", t_name)){
     t2_ = cv::imread(t_name.c_str(),0);
   }
   else{
     ROS_ERROR("Must set param: template2");
   }
   if(pnh.getParam( "template3", t_name)){
     t3_ = cv::imread(t_name.c_str(),0);
   }
   else{
     ROS_ERROR("Must set param: template3");
   }
   if(pnh.getParam( "template4", t_name)){
     t4_ = cv::imread(t_name.c_str(), 0);
   }
   else{
     ROS_ERROR("Must set param: template4");
   }

   // initialize rois for tracking 4 targets
   roi1_.x = 0;
   roi1_.y = my_roi_.height/4+20;
   roi2_.x = my_roi_.width/4;
   roi2_.y = my_roi_.height/4+20;
   roi3_.x = my_roi_.width/2;
   roi3_.y = my_roi_.height/4+20;
   roi4_.x = my_roi_.width*3./4.;
   roi4_.y = my_roi_.height/4+20;

   roi1_.height = 80;
   roi1_.width = 80;
   roi2_.height = 80;
   roi2_.width = 80;
   roi3_.height = 80;
   roi3_.width = 80;
   roi4_.height = 80;
   roi4_.width = 80;

 
   // Subscrive to input video feed and publish output video feed
   image_sub_ = it_.subscribe(image_topic_.c_str(), 1, &InkTest::imageCb, this);
   image_pub_ = it_.advertise("/test/image_out", 1);
   laplace_image_pub_ = it_.advertise("/test/laplace_image", 1);

   //   cv::namedWindow("t1_match");
   //   cv::namedWindow("t2_match");
   //   cv::namedWindow("t3_match");
   //   cv::namedWindow("t4_match");
   cv::namedWindow("laplacian");
   //   cv::namedWindow("harris");
  };

  ~InkTest()  {  } ;
  enum shape
  {
    SHAPE_NONE,
    SHAPE_ONE,
    SHAPE_TWO,
    SHAPE_THREE,
    SHAPE_FOUR
  };
  struct detection{
    bool status;
    cv::Point pnt;
    shape s;
  };
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  detection find_spot(cv::Mat &image, cv::Rect &roi, uint16_t thresh)
  {
    detection D;
    int num=0;
    int totalx=0;
    int totaly=0;

    int mini = roi.x+roi.width;
    int maxi = roi.x;
    int minj = roi.y+roi.height;
    int maxj = roi.y;
    for( int j = roi.y; j < (roi.y+roi.height); j++ )
      { 
	for( int i = roi.x ; i < (roi.x+roi.width); i++ )
          {
            if( image.at<uint16_t>(j,i) >= thresh )
              {
                num++;
		totalx += i;
		totaly += j;
		if(i<mini) mini=i;
		if(i>maxi) maxi=i;
		if(j<minj) minj=j;
		if(j>maxj) maxj=j;
              }
          }
      }
    int centx = (mini+maxi)/2;
    int centy = (minj+maxj)/2;
    if(num==0){
      D.status = false;
      D.s = SHAPE_NONE;
    }
    else{
      D.status = true;
      D.pnt = cv::Point(totalx/num, totaly/num);
      float angle = atan2(D.pnt.x-centx, D.pnt.y-centy)*180./3.14;
      //      ROS_ERROR("shape found at %d %d, angle = %4.1f", D.pnt.x,D.pnt.y, angle);   
      if(angle>0.0 && angle < 90.0)    D.s = SHAPE_ONE;
      if(angle>=90.0)                  D.s = SHAPE_TWO;
      if(angle<0.0 && angle > -90.0)   D.s = SHAPE_THREE;
      if(angle<=-90.0)                 D.s = SHAPE_FOUR;
    }
    return(D);
  }
  void show_shape(detection &D, const char msg[])
  {
    if(D.s == SHAPE_NONE) ROS_ERROR("%s:Shape NONE",msg);
    if(D.s == SHAPE_ONE) ROS_ERROR("%s:Shape1",msg);
    if(D.s == SHAPE_TWO) ROS_ERROR("%s:Shape2",msg);
    if(D.s == SHAPE_THREE) ROS_ERROR("%s:Shape3",msg);
    if(D.s == SHAPE_FOUR) ROS_ERROR("%s:Shape4",msg);
  }
private:
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher laplace_image_pub_;
  ros::NodeHandle nh_;
  string camera_name_;
  string image_topic_;
  int image_width_;
  int image_height_;
  cv::Rect my_roi_;
  cv::Rect roi1_,roi2_,roi3_,roi4_;
  bool has_roi_;
  cv::Mat t1_,t2_,t3_,t4_;
};


void InkTest::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr; // original image

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

/*
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows == image_height_ && cv_ptr->image.cols == image_width_)
      cv::circle(cv_ptr->image, cv::Point(image_width_/2, image_height_/2), 30, 128, 10);
*/
  
    // use laplacian 
    cv::Mat gray_image;

    if(has_roi_){
      cv::Mat temp_image = cv_ptr->image.clone();
      gray_image = temp_image(my_roi_);
    }
    else{
      gray_image = cv_ptr->image.clone();
    }
    cv::Mat raw_laplace_image, laplace_image, harris_image, dst, dst_norm;
    int kernel_size = 5;
    int scale = 2;
    int delta = 0;
    int ddepth = CV_16UC1;
    ros::NodeHandle pnh("~");
    pnh.getParam( "kernel_size", kernel_size);
    if(kernel_size%2==0) kernel_size++;
    pnh.getParam( "scale", scale);
    //    ROS_INFO("ks = %d s = %d",kernel_size, scale);

    // compute laplacian image
    cv::Laplacian( gray_image, laplace_image, ddepth, kernel_size, scale, delta, cv::BORDER_DEFAULT );
    //    convertScaleAbs( raw_laplace_image, laplace_image );
    unsigned int lmax = 0;
    int maxi,maxj;
    for( int j = 0; j < laplace_image.rows ; j++ )
      { 
	for( int i = 0; i < laplace_image.cols; i++ )
          {
            if( laplace_image.at<uint16_t>(j,i) > lmax ){
	      lmax = laplace_image.at<uint16_t>(j,i);
	      maxi = i;
	      maxj = j;
	    }
          }
      }

    // initialize rois for each quarter
    uint16_t thresh = lmax*.9;
    detection D = find_spot(laplace_image, roi1_, thresh);
    rectangle(laplace_image,roi1_, cv::Scalar(255*254), 3);
    rectangle(laplace_image,roi2_, cv::Scalar(255*254), 3);
    rectangle(laplace_image,roi3_, cv::Scalar(255*254), 3);
    rectangle(laplace_image,roi4_, cv::Scalar(255*254), 3);
    if(D.pnt.x>0 && D.pnt.y>0){
      show_shape(D,"roi1");
      circle( laplace_image, D.pnt, 20, cv::Scalar(255*254),1);
      roi1_.x = D.pnt.x-roi1_.width/2;
      roi1_.y = D.pnt.y-roi1_.height/2;
    }
    else{
      roi1_.x = 0;
      roi1_.y = my_roi_.height/4;
    }
    D = find_spot(laplace_image, roi2_, thresh);
    if(D.pnt.x>0&&D.pnt.y>0){
      show_shape(D,"roi2");
      circle( laplace_image, D.pnt, 20, cv::Scalar(255*254),1);
      roi2_.x = D.pnt.x - roi2_.width/2;
      roi2_.y = D.pnt.y - roi2_.height/2;
    }
    else{
      roi2_.x = my_roi_.width/4;
      roi2_.y = my_roi_.height/4;
    }
    D = find_spot(laplace_image, roi3_, thresh);
    if(D.pnt.x>0&&D.pnt.y>0){
      show_shape(D,"roi3");
      circle( laplace_image, D.pnt, 20, cv::Scalar(255*254),1);
      roi3_.x = D.pnt.x - roi3_.width/2;
      roi3_.y = D.pnt.y - roi3_.height/2;
    }
    else{
      roi3_.x = my_roi_.width/2;
      roi3_.y = my_roi_.height/4;
    }
    D = find_spot(laplace_image, roi4_, thresh);
    if(D.pnt.x>0&&D.pnt.y>0){
      show_shape(D,"roi4");
      circle( laplace_image, D.pnt, 20, cv::Scalar(255*254),1);
      roi4_.x = D.pnt.x - roi4_.width/2;
      roi4_.y = D.pnt.y - roi4_.height/2;
    }
    else{ 
      roi4_.x = my_roi_.width*3./4.;
      roi4_.y = my_roi_.height/4;
    }
    
    // match using correlation on templates
    /*
    cv::Mat t1_match,t2_match,t3_match,t4_match;
    cv::matchTemplate( gray_image, t1_, t1_match, 4 );
    cv::matchTemplate( gray_image, t2_, t2_match, 1 );
    cv::matchTemplate( gray_image, t3_, t3_match, 5 );
    cv::matchTemplate( gray_image, t4_, t4_match, 3 );
    cv::normalize( t1_match, t1_match, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
    cv::normalize( t2_match, t2_match, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
    cv::normalize( t3_match, t3_match, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
    cv::normalize( t4_match, t4_match, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
    */

    /// Detector parameters
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;
    
    /// Detecting Harris corners
    /*
    cv::cornerHarris( gray_image, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT );
    
    /// Normalizing
    normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
    convertScaleAbs( dst_norm, harris_image );


    int thresh = 100;
    /// Drawing a circle around corners
    for( int j = 0; j < dst_norm.rows ; j++ )
      { for( int i = 0; i < dst_norm.cols; i++ )
          {
            if( (int) dst_norm.at<float>(j,i) > thresh )
              {
		circle( harris_image, cv::Point( i, j ), 5,  cv::Scalar(0), 2, 8, 0 );
              }
          }
      }

    */

    // Update GUI Window
    //    cv::imshow("t1_match", t1_match);
    //    cv::imshow("t2_match", t2_match);
    //    cv::imshow("t3_match", t3_match);
    //    cv::imshow("t4_match", t4_match);
    cv::imshow("laplacian", laplace_image);
    //    cv::imshow("harris", harris_image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    
    std_msgs::Header header; // empty header
    header.stamp = ros::Time::now(); // time
    cv_bridge::CvImage li_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, laplace_image);
    laplace_image_pub_.publish(li_img.toImageMsg());

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "inktest");
  ros::NodeHandle node_handle;
  InkTest IT(node_handle);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
