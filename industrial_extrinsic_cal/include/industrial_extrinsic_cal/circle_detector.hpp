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
#ifndef CIRCLEDETECTOR_HPP
#define CIRCLEDETECTOR_HPP

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

using std::vector;

namespace cv
{
class CV_EXPORTS_W CircleDetector : public FeatureDetector
{
public:
  struct CV_EXPORTS_W_SIMPLE Params
  {
    CV_WRAP Params();
    CV_PROP_RW float thresholdStep;
    CV_PROP_RW float minThreshold;
    CV_PROP_RW float maxThreshold;
    CV_PROP_RW size_t minRepeatability;
    CV_PROP_RW float minDistBetweenCircles;
    CV_PROP_RW float minRadiusDiff;

    CV_PROP_RW bool filterByColor;
    CV_PROP_RW uchar circleColor;

    CV_PROP_RW bool filterByArea;
    CV_PROP_RW float minArea, maxArea;

    CV_PROP_RW bool filterByCircularity;
    CV_PROP_RW float minCircularity, maxCircularity;

    CV_PROP_RW bool filterByInertia;
    CV_PROP_RW float minInertiaRatio, maxInertiaRatio;

    CV_PROP_RW bool filterByConvexity;
    CV_PROP_RW float minConvexity, maxConvexity;

    void read(const FileNode& fn);
    void write(FileStorage& fs) const;
  };

  CV_WRAP static Ptr<CircleDetector> create(const CircleDetector::Params& parameters = CircleDetector::Params());
};
}
#endif
