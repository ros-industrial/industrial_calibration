/*
 IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.

 By downloading, copying, installing or using the software you agree
 to this license.
 If you do not agree to this license, do not download, install,
 copy or use the software.


                          License Agreement
               For Open Source Computer Vision Library

Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
Copyright (C) 2009, Willow Garage Inc., all rights reserved.
Copyright (C) 2014, Southwest Research Institute, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistribution's of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistribution's in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * The name of the copyright holders may not be used to endorse or
    promote products
    derived from this software without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed.
In no event shall the Intel Corporation or contributors be liable for any
direct,
indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

// Slight Modification of OpenCV function to use ellipse fitting rather than
// center of mass of contour to provide the location of the circle.

#include <ical_core/target_finders/utils/circle_detector.h>
#include <ical_core/exceptions.h>

#include <algorithm>
#include <iterator>
#include <yaml-cpp/yaml.h>

namespace industrial_calibration
{
/**
 * @brief The DetectionOutput struct
 */
struct DetectionResult
{
  std::vector<CircleDetector::Center> centers;
  std::vector<std::vector<cv::Point>> contours;
};

/**
 * @brief Helper function for finding circles within an image
 * This function thresholds the input image at the input intensity, applies filtering, and attempts to find circles from
 * contours in the image
 * @param image - grayscale image
 * @param threshold - threshold intensity
 * @param params - circle detection parameters
 * @return
 */
DetectionResult findCircles(const cv::Mat& image, const double threshold, const CircleDetectorParams& params)
{
  // Threshold the image
  cv::Mat binarized_image;
  cv::threshold(image, binarized_image, threshold, 255, cv::THRESH_BINARY);

  // Get the contours of the image
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binarized_image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

  // Remove all contours with less than 5 points (OpenCV requirement for ellipse fitting)
  contours.erase(std::remove_if(contours.begin(), contours.end(),
                                [](const std::vector<cv::Point>& contour) { return contour.size() < 5; }),
                 contours.end());

  // Debug
  cv::Mat debug_img;
  cv::cvtColor(binarized_image, debug_img, cv::COLOR_GRAY2RGB);
  const cv::Scalar color(255, 0, 0);

  // Loop on all contours
  DetectionResult result;
  for (std::size_t i = 0; i < contours.size(); ++i)
  {
    // Each if statement may eliminate a contour through the continue function
    const std::vector<cv::Point>& contour = contours[i];

    // Calculate the moment of inertia of the contour
    cv::Moments moms = cv::moments(cv::Mat(contour));

    // Area filter
    if (params.filterByArea)
    {
      double area = moms.m00;
      if (area < params.minArea || area >= params.maxArea) continue;
    }

    // Circularity filter
    if (params.filterByCircularity)
    {
      double area = moms.m00;
      double perimeter = cv::arcLength(cv::Mat(contour), true);
      double ratio = 4 * CV_PI * area / (perimeter * perimeter);
      if (ratio < params.minCircularity || ratio >= params.maxCircularity) continue;
    }

    // Inertia filter
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
    }

    // Convexity filter
    if (params.filterByConvexity)
    {
      std::vector<cv::Point> hull;
      cv::convexHull(cv::Mat(contour), hull);
      double area = cv::contourArea(cv::Mat(contour));
      double hullArea = cv::contourArea(cv::Mat(hull));
      double ratio = area / hullArea;
      if (ratio < params.minConvexity || ratio >= params.maxConvexity) continue;
    }

    // Fit an ellipse to the contour
    cv::RotatedRect box = cv::fitEllipse(contour);

    // Check that the color of the center pixel matches the expectation
    if (params.filterByColor)
    {
      if (binarized_image.at<uchar>(cvRound(box.center.y), cvRound(box.center.x)) != params.circleColor) continue;
    }

    // Check that the contour matches a model of an ellipse within tolerance
    double err = 0.0;
    {
      // Get the angle of the ellipse and its major (a) and minor (b) axes
      double angle = box.angle * CV_PI / 180.0;
      double a = box.size.width / 2.0;
      double b = box.size.height / 2.0;

      // Calculate the output of the ellipse equation ( x^2/a^2 + y^2/b^2 = 1) for each point in the contour
      double output = 0.0;
      for (const cv::Point& pt : contour)
      {
        // Get the vector from the center of the ellipse to the point on the contour
        cv::Point2f v = cv::Point2f(pt) - box.center;

        // Since the major and minor axes are not aligned with the coordinate system, we need to transform the x and y
        // coordinates
        double x = v.x * std::cos(angle) + v.y * std::sin(angle);
        double y = v.x * std::sin(angle) - v.y * std::cos(angle);

        // The output of the ellipse equation using this point should equal one
        double one = std::sqrt((std::pow(x, 2.0) / std::pow(a, 2.0)) + (std::pow(y, 2.0) / std::pow(b, 2.0)));

        // Accumulate the output
        output += one;
      }

      // Average the error and subtract the expected output of the ellipse equation (i.e. 1.0)
      err = std::abs((output / contour.size()) - 1.0);
    }

    // If the error is below the threshold then add it to the list of circles
    if (err < params.maxAverageEllipseError)
    {
      // Create the center
      CircleDetector::Center center;
      center.location = box.center;
      center.confidence = 1.0 / err;
      center.radius = (box.size.height + box.size.width) / 4.0;

      result.centers.push_back(center);
      result.contours.push_back(contours[i]);
    }
  }

  return result;
}

/**
 * @brief Applies various thresholds to the input image and attempts to find all circles within the images
 * @param image - grayscale image
 * @param params - circle detection parameters
 * @return
 */
std::vector<DetectionResult> applyThresholds(const cv::Mat& image, const CircleDetectorParams& params)
{
  // Check that the detection parameters make sense
  if (params.minRepeatability > params.nThresholds)
  {
    std::stringstream ss;
    ss << "Minimum repeatability (" << params.minRepeatability << ") cannot exceed the number of thresholds ("
       << params.nThresholds << ")";
    throw ICalException(ss.str());
  }

  // Create a container for all of the circle centers and contours detected at the different threshold levels
  std::vector<DetectionResult> results;
  results.reserve(params.nThresholds);

  // Threshold the image per the input parameters and attempt to find all circles
  const double threshold_range = params.maxThreshold - params.minThreshold;
  for (std::size_t i = 0; i < params.nThresholds; ++i)
  {
    double threshold = 0.0;
    if (params.nThresholds < 2)
    {
      threshold = params.minThreshold + (threshold_range / 2.0);
    }
    else
    {
      threshold = params.minThreshold + (static_cast<double>(i) / (params.nThresholds - 1)) * threshold_range;
    }

    // Find all the circles in the image
    results.push_back(findCircles(image, threshold, params));
  }

  return results;
}

/**
 * @brief Given a list of potential circle centers acquired from differently threshold-ed images, this function matches
 * circle centers that are close in position and radius, and performs a weighted average of their position and radius to
 * create keypoints
 * @param all_centers - a vector of vector of circle centers acquired from images with different threshold values
 * @param params - circle detection parameters
 * @return
 */
std::vector<cv::KeyPoint> detectKeyPoints(const std::vector<std::vector<CircleDetector::Center>>& all_centers,
                                          const CircleDetectorParams& params)
{
  // Lambda for determining if the input circle center already exists in the list of known circle centers
  auto exists = [&params](const CircleDetector::Center& c,
                          const std::vector<std::vector<CircleDetector::Center>>& known_centers) -> int {
    for (std::size_t i = 0; i < known_centers.size(); ++i)
    {
      const std::vector<CircleDetector::Center>& existing_centers = known_centers.at(i);
      if (!existing_centers.empty())
      {
        // Compare the identified center with the first existing center
        const CircleDetector::Center& existing = existing_centers.front();
        double pos_diff = cv::norm(existing.location - c.location);
        double radius_diff = std::abs(existing.radius - c.radius);

        // If the position and radius differences are within tolerance, add this point to this list of same centers
        // (to be weight-averaged later)
        if (pos_diff <= params.circleInclusionRadius && radius_diff <= params.maxRadiusDiff)
        {
          return i;
        }
      }
    }

    return -1;
  };

  // Create a container for the grouped circle centers
  std::vector<std::vector<CircleDetector::Center>> grouped_centers;

  // Loop over all of the circle centers detected at the various threshold levels and try to group circle centers that
  // are close together
  for (const std::vector<CircleDetector::Center>& centers : all_centers)
  {
    // Loop over all of the circle centers for the current threshold level
    for (const CircleDetector::Center& c : centers)
    {
      // Check if this circle center has already been identified
      int match_idx = exists(c, grouped_centers);

      // If so, then add this center to the vector of other circle centers at approximately the same location
      if (match_idx >= 0)
      {
        grouped_centers.at(match_idx).push_back(c);
      }
      else
      {
        // Otherwise, add it in a new vector to the circle centers list
        grouped_centers.push_back({ c });
      }
    }
  }

  // Once all circle centers have been found in the variously thresholded image, create a keypoint based on the weighted
  // average of each center
  std::vector<cv::KeyPoint> keypoints;
  keypoints.reserve(grouped_centers.size());

  for (std::size_t i = 0; i < grouped_centers.size(); i++)
  {
    // Check that the circle center meets the minimum repeatability criteria
    if (grouped_centers[i].size() >= params.minRepeatability)
    {
      // Get the weighted average of the circle center's location
      cv::Point2d sum_point(0, 0);
      double sum_radius = 0.0;
      double normalizer = 0;
      for (std::size_t j = 0; j < grouped_centers[i].size(); j++)
      {
        sum_point += grouped_centers[i][j].confidence * grouped_centers[i][j].location;
        sum_radius += grouped_centers[i][j].confidence * grouped_centers[i][j].radius;
        normalizer += grouped_centers[i][j].confidence;
      }
      sum_point /= normalizer;
      sum_radius /= normalizer;

      // Create a key point and add it to the output parameter
      keypoints.emplace_back(sum_point, sum_radius * 2.0);
    }
  }

  return keypoints;
}

CircleDetector::CircleDetector(const CircleDetectorParams& parameters) : params(parameters)
{
}

void CircleDetector::detect(cv::InputArray input, std::vector<cv::KeyPoint>& keypoints, cv::InputArray)
{
  cv::Mat image = input.getMat();
  cv::Mat grayscale_image;

  if (image.channels() == 3)
    cv::cvtColor(image, grayscale_image, cv::COLOR_BGR2GRAY);
  else
    grayscale_image = image;

  // Get the circle detection results (circle centers and contours) at all of the threshold levels
  const std::vector<DetectionResult> results = applyThresholds(grayscale_image, params);

  // Create a container of all the circle centers identified in each threshold step
  std::vector<std::vector<Center>> centers;
  centers.reserve(results.size());
  for (const auto& result : results)
  {
    centers.push_back(result.centers);
  }

  // Use the circle centers to detect the key points
  keypoints = detectKeyPoints(centers, params);
}

cv::Mat CircleDetector::drawDetectedCircles(const cv::Mat& image)
{
  cv::Mat grayscale_image;
  if (image.channels() == 3)
    cv::cvtColor(image, grayscale_image, cv::COLOR_BGR2GRAY);
  else
    grayscale_image = image;

  // Detect the keypoints
  const std::vector<DetectionResult> results = applyThresholds(grayscale_image, params);

  // Output
  cv::Mat output = image.clone();

  // Draw the contours on the image
  const cv::Scalar color(255, 0, 0);
  for (const auto& result : results)
  {
    for (std::size_t i = 0; i < result.contours.size(); ++i)
    {
      cv::drawContours(output, result.contours, i, color, 2);
    }
  }

  // Create a container of all the circle centers identified in each threshold step
  std::vector<std::vector<Center>> centers;
  centers.reserve(results.size());
  for (const auto& result : results)
  {
    centers.push_back(result.centers);
  }

  // Draw the keypoints on the image
  const std::vector<cv::KeyPoint> keypoints = detectKeyPoints(centers, params);
  cv::drawKeypoints(output, keypoints, output);

  return output;
}

cv::Ptr<CircleDetector> CircleDetector::create(const CircleDetectorParams& params)
{
  return cv::makePtr<CircleDetector>(params);
}

}  // namespace industrial_calibration
