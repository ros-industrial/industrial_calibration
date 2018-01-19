/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Copyright (C) 2014, Southwest Research Institute, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

/********************************************************************************************
**   Slight Modification of OpenCV function to use ellipse fitting rather than center of mass of contour ****
**  to provide the location of the circle ****
********************************************************************************************/

#include "opencv2/features2d.hpp"
#include "opencv2/imgproc.hpp"

#include "opencv2/core/utility.hpp"
//#include "opencv2/core/private.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/core/hal/hal.hpp"

#include <algorithm>

#ifdef HAVE_TEGRA_OPTIMIZATION
#include "opencv2/features2d/features2d_tegra.hpp"
#endif

#include "opencv2/opencv.hpp"

#include <industrial_extrinsic_cal/circle_detector.hpp>
#include <iterator>

//#define DEBUG_CIRCLE_DETECTOR

#ifdef DEBUG_CIRCLE_DETECTOR
#include "opencv2/opencv_modules.hpp"
#ifdef HAVE_OPENCV_HIGHGUI
#include "opencv2/highgui/highgui.hpp"
#else
#undef DEBUG_CIRCLE_DETECTOR
#endif
#endif

namespace cv
{
class CV_EXPORTS_W CircleDetectorImpl : public CircleDetector
{
public:
  explicit CircleDetectorImpl(const CircleDetector::Params& parameters = CircleDetector::Params());

  virtual void read(const FileNode& fn);
  virtual void write(FileStorage& fs) const;

protected:
  struct CV_EXPORTS Center
  {
    Point2d location;
    double radius;
    double confidence;
  };

  virtual void detect(InputArray image, std::vector<KeyPoint>& keypoints, InputArray mask = noArray());
  virtual void findCircles(InputArray image, InputArray binaryImage, std::vector<Center>& centers) const;

  Params params;
};

/*
*  CircleDetector
*/
CircleDetector::Params::Params()
{
  thresholdStep = 10;
  minThreshold = 50;
  maxThreshold = 220;
  minRepeatability = 2;
  minDistBetweenCircles = 10;
  minRadiusDiff = 10;

  filterByColor = true;
  circleColor = 0;

  filterByArea = true;
  minArea = 25;
  maxArea = 5000;

  filterByCircularity = false;
  minCircularity = 0.8f;
  maxCircularity = std::numeric_limits<float>::max();

  filterByInertia = true;
  minInertiaRatio = 0.1f;
  maxInertiaRatio = std::numeric_limits<float>::max();

  filterByConvexity = true;
  minConvexity = 0.95f;
  maxConvexity = std::numeric_limits<float>::max();
}

void CircleDetector::Params::read(const cv::FileNode& fn)
{
  thresholdStep = fn["thresholdStep"];
  minThreshold = fn["minThreshold"];
  maxThreshold = fn["maxThreshold"];

  minRepeatability = (size_t)(int)fn["minRepeatability"];
  minDistBetweenCircles = fn["minDistBetweenCircles"];

  filterByColor = (int)fn["filterByColor"] != 0 ? true : false;
  circleColor = (uchar)(int)fn["circleColor"];

  filterByArea = (int)fn["filterByArea"] != 0 ? true : false;
  minArea = fn["minArea"];
  maxArea = fn["maxArea"];

  filterByCircularity = (int)fn["filterByCircularity"] != 0 ? true : false;
  minCircularity = fn["minCircularity"];
  maxCircularity = fn["maxCircularity"];

  filterByInertia = (int)fn["filterByInertia"] != 0 ? true : false;
  minInertiaRatio = fn["minInertiaRatio"];
  maxInertiaRatio = fn["maxInertiaRatio"];

  filterByConvexity = (int)fn["filterByConvexity"] != 0 ? true : false;
  minConvexity = fn["minConvexity"];
  maxConvexity = fn["maxConvexity"];
}

void CircleDetector::Params::write(cv::FileStorage& fs) const
{
  fs << "thresholdStep" << thresholdStep;
  fs << "minThreshold" << minThreshold;
  fs << "maxThreshold" << maxThreshold;

  fs << "minRepeatability" << (int)minRepeatability;
  fs << "minDistBetweenCircles" << minDistBetweenCircles;

  fs << "filterByColor" << (int)filterByColor;
  fs << "circleColor" << (int)circleColor;

  fs << "filterByArea" << (int)filterByArea;
  fs << "minArea" << minArea;
  fs << "maxArea" << maxArea;

  fs << "filterByCircularity" << (int)filterByCircularity;
  fs << "minCircularity" << minCircularity;
  fs << "maxCircularity" << maxCircularity;

  fs << "filterByInertia" << (int)filterByInertia;
  fs << "minInertiaRatio" << minInertiaRatio;
  fs << "maxInertiaRatio" << maxInertiaRatio;

  fs << "filterByConvexity" << (int)filterByConvexity;
  fs << "minConvexity" << minConvexity;
  fs << "maxConvexity" << maxConvexity;
}

CircleDetectorImpl::CircleDetectorImpl(const CircleDetector::Params& parameters) : params(parameters)
{
}

void CircleDetectorImpl::read(const cv::FileNode& fn)
{
  params.read(fn);
}

void CircleDetectorImpl::write(cv::FileStorage& fs) const
{
  writeFormat(fs);
  params.write(fs);
}

void CircleDetectorImpl::findCircles(InputArray _image, InputArray _binaryImage, std::vector<Center>& centers) const
{
  //  CV_INSTRUMENT_REGION()

  Mat image = _image.getMat();  // Oh so much  cleaner this way :(
  Mat binaryImage = _binaryImage.getMat();

  (void)image;
  centers.clear();

  vector<vector<Point> > contours;
  Mat tmpBinaryImage = binaryImage.clone();
  findContours(tmpBinaryImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

  // loop on all contours
  for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
  {
    // each if statement may eliminate a contour through the continue function
    // some if statements may also set the confidence whose default is 1.0
    Center center;
    center.confidence = 1;
    Moments moms = moments(Mat(contours[contourIdx]));
    if (params.filterByArea)
    {
      double area = moms.m00;
      if (area < params.minArea || area >= params.maxArea) continue;
    }

    if (params.filterByCircularity)
    {
      double area = moms.m00;
      double perimeter = arcLength(Mat(contours[contourIdx]), true);
      double ratio = 4 * CV_PI * area / (perimeter * perimeter);
      if (ratio < params.minCircularity || ratio >= params.maxCircularity) continue;
    }

    if (params.filterByInertia)
    {
      double denominator = sqrt(pow(2 * moms.mu11, 2) + pow(moms.mu20 - moms.mu02, 2));
      const double eps = 1e-2;
      double ratio;
      if (denominator > eps)
      {
        double cosmin = (moms.mu20 - moms.mu02) / denominator;
        double sinmin = 2 * moms.mu11 / denominator;
        double cosmax = -cosmin;
        double sinmax = -sinmin;

        double imin = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmin - moms.mu11 * sinmin;
        double imax = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmax - moms.mu11 * sinmax;
        ratio = imin / imax;
      }
      else
      {
        ratio = 1;
      }

      if (ratio < params.minInertiaRatio || ratio >= params.maxInertiaRatio) continue;

      center.confidence = ratio * ratio;
    }

    if (params.filterByConvexity)
    {
      vector<Point> hull;
      convexHull(Mat(contours[contourIdx]), hull);
      double area = contourArea(Mat(contours[contourIdx]));
      double hullArea = contourArea(Mat(hull));
      double ratio = area / hullArea;
      if (ratio < params.minConvexity || ratio >= params.maxConvexity) continue;
    }
    Mat pointsf;
    Mat(contours[contourIdx]).convertTo(pointsf, CV_32F);
    if (pointsf.rows < 5) continue;
    RotatedRect box = fitEllipse(pointsf);

    // find center
    // center.location = Point2d(moms.m10 / moms.m00, moms.m01 / moms.m00);
    center.location = box.center;

    // one more filter by color of central pixel
    if (params.filterByColor)
    {
      if (binaryImage.at<uchar>(cvRound(center.location.y), cvRound(center.location.x)) != params.circleColor) continue;
    }

    // compute circle radius
    //	{
    //	vector<double> dists;
    //	for (size_t pointIdx = 0; pointIdx < contours[contourIdx].size(); pointIdx++)
    //	{
    //		Point2d pt = contours[contourIdx][pointIdx];
    //		dists.push_back(norm(center.location - pt));
    //	}
    //	std::sort(dists.begin(), dists.end());
    //	center.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;
    //}
    center.radius = (box.size.height + box.size.width) / 4.0;
    centers.push_back(center);

#ifdef DEBUG_CIRCLE_DETECTOR
//    circle( keypointsImage, center.location, 1, Scalar(0,0,255), 1 );
#endif
  }
#ifdef DEBUG_CIRCLE_DETECTOR
//  imshow("bk", keypointsImage );
//  waitKey();
#endif
}

void CircleDetectorImpl::detect(InputArray _image, std::vector<KeyPoint>& keypoints, InputArray mask)
{
  Mat image = _image.getMat();

  //  CV_INSTRUMENT_REGION()

  // TODO: support mask
  keypoints.clear();
  Mat grayscaleImage;
  if (image.channels() == 3)
    cvtColor(image, grayscaleImage, CV_BGR2GRAY);
  else
    grayscaleImage = image;

  vector<vector<Center> > centers;
  for (double thresh = params.minThreshold; thresh < params.maxThreshold; thresh += params.thresholdStep)
  {
    Mat binarizedImage;
    threshold(grayscaleImage, binarizedImage, thresh, 255, THRESH_BINARY);

#ifdef DEBUG_CIRCLE_DETECTOR
//    Mat keypointsImage;
//    cvtColor( binarizedImage, keypointsImage, CV_GRAY2RGB );
#endif

    vector<Center> curCenters;
    findCircles(grayscaleImage, binarizedImage, curCenters);
    vector<vector<Center> > newCenters;
    for (size_t i = 0; i < curCenters.size(); i++)
    {
#ifdef DEBUG_CIRCLE_DETECTOR
//      circle(keypointsImage, curCenters[i].location, curCenters[i].radius, Scalar(0,0,255),-1);
#endif

      bool isNew = true;
      for (size_t j = 0; j < centers.size(); j++)
      {
        double dist = norm(centers[j][centers[j].size() / 2].location - curCenters[i].location);
        //				isNew = dist >= params.minDistBetweenCircles && dist >= centers[j][ centers[j].size() / 2 ].radius &&
        // dist >= curCenters[i].radius;
        double rad_diff = fabs(centers[j][centers[j].size() / 2].radius - curCenters[i].radius);
        isNew = dist >= params.minDistBetweenCircles || rad_diff >= params.minRadiusDiff;
        if (!isNew)
        {
          centers[j].push_back(curCenters[i]);

          size_t k = centers[j].size() - 1;
          while (k > 0 && centers[j][k].radius < centers[j][k - 1].radius)
          {
            centers[j][k] = centers[j][k - 1];
            k--;
          }
          centers[j][k] = curCenters[i];

          break;
        }
      }
      if (isNew)
      {
        newCenters.push_back(vector<Center>(1, curCenters[i]));
        // centers.push_back(vector<Center> (1, curCenters[i]));
      }
    }
    std::copy(newCenters.begin(), newCenters.end(), std::back_inserter(centers));

#ifdef DEBUG_CIRCLE_DETECTOR
//    imshow("binarized", keypointsImage );
// waitKey();
#endif
  }

  for (size_t i = 0; i < centers.size(); i++)
  {
    if (centers[i].size() < params.minRepeatability) continue;
    Point2d sumPoint(0, 0);
    double normalizer = 0;
    for (size_t j = 0; j < centers[i].size(); j++)
    {
      sumPoint += centers[i][j].confidence * centers[i][j].location;
      normalizer += centers[i][j].confidence;
    }
    sumPoint *= (1. / normalizer);
    KeyPoint kpt(sumPoint, (float)(centers[i][centers[i].size() / 2].radius * 2.0));
    keypoints.push_back(kpt);
  }

#ifdef DEBUG_CIRCLE_DETECTOR
  namedWindow("keypoints", CV_WINDOW_NORMAL);
  Mat outImg = image.clone();
  for (size_t i = 0; i < keypoints.size(); i++)
  {
    circle(outImg, keypoints[i].pt, keypoints[i].size, Scalar(255, 0, 255), -1);
  }
  // drawKeypoints(image, keypoints, outImg);
  imshow("keypoints", outImg);
  waitKey();
#endif
}

Ptr<CircleDetector> CircleDetector::create(const CircleDetector::Params& params)
{
  return makePtr<CircleDetectorImpl>(params);
}

}  // end  namespace cv
