#ifndef CIRCLEDETECTOR_HPP
#define CIRCLEDETECTOR_HPP

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"


namespace cv {
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

      void read( const FileNode& fn );
      void write( FileStorage& fs ) const;
  };

  CV_WRAP CircleDetector(const CircleDetector::Params &parameters = CircleDetector::Params());

  virtual void read( const FileNode& fn );
  virtual void write( FileStorage& fs ) const;

protected:
  struct CV_EXPORTS Center
  {
      Point2d location;
      double radius;
      double confidence;
  };

  virtual void detectImpl( const Mat& image, vector<KeyPoint>& keypoints, const Mat& mask=Mat() ) const;
  virtual void findCircles(const Mat &image, const Mat &binaryImage, vector<Center> &centers) const;

  Params params;
};
}
#endif
