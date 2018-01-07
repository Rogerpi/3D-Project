#ifndef COMMONFUNC_H
#define COMMONFUNC_H

#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>


class commonFunc
{
public:
  commonFunc();

  /************ THIS Functions have been taken from Cansen OpenCV Tutorial *********************/
  void dispDepthColor(cv::Mat &depthMap, cv::Mat &depthColor);
  void dispDepthGray(cv::Mat &depthMap, cv::Mat &depthGray);

  // defining a static function
  static void static_dispDepthGray(cv::Mat &depthMap, cv::Mat &depthGray)
  {
    cv::normalize(depthMap, depthGray, 0, 255, CV_MINMAX);
    depthGray.convertTo(depthGray, CV_8UC3);
  }

  ////////////////////////////////////////////////////////////////
  //////////////// Opencv Image Processing Functions /////////////
  ////////////////////////////////////////////////////////////////
  // Convert color image to gray scale image
  void rgb2gray(cv::Mat &rgbImg, cv::Mat &grayImg);

  // Flip image left and right
  void flipImage(cv::Mat &rgbImg);


  // Detect feature points on image
  void featureExtraction(cv::Mat &rgbImg, std::vector<cv::KeyPoint> &keyPts);

  // Show detected keypoints on the color image
  void showKeyPoints(cv::Mat &rgbImg, std::vector<cv::KeyPoint> &keyPts);



  // Detect and match features between two images
  void featureMatching(cv::Mat &rgbImg01, std::vector<cv::KeyPoint> &keyPts01,
                       cv::Mat &rgbImg02, std::vector<cv::KeyPoint> &keyPts02,
                       std::vector< cv::DMatch > *matches, bool robustMatch = false);
   /********************************************************************************************/
  /************ THIS Functions have been taken from Cansen OpenCV Tutorial *********************/

  // Draw the feature matches
  //This function has been modified to draw manually the matches to make sure we were taking the correct points (cx,cy)
  cv::Mat showFeatureMatches(cv::Mat &rgb_1, std::vector<cv::KeyPoint> &keyPts_1,
                          cv::Mat &rgb_2, std::vector<cv::KeyPoint> &keyPts_2,
                          std::vector< cv::DMatch > *matches);

  //scalarFactor has been modified from 5000 to 1000 because kinect depth resolution is in mm
  void myDepth2meter(const float feat_x, const float feat_y, const float rawDisparity,
                   float &x, float &y, float &z);

//This function has been modified to return a pcl pointcloud instead of saving points into a text file
pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbd2pcl(const cv::Mat &rgbImg, const cv::Mat &depthImg, const double min_dist_thr = 0 , const double max_dist_thr = 4.5);
 /***********************************************************************************************************************/
/************************************ Image Processing Technics *********************************************************/
cv::Mat threshold_mapped_depth_to_color(cv::Mat& depth_im,cv::Mat& color_im, const double min_m_thr, const double max_m_thr);
void threshold_depth(cv::Mat& depth_im, cv::Mat& depth_thr, const double min_m_thr, const double max_m_thr);

 void border_outlier_removal(cv::Mat & depth_thr_in,cv::Mat &depth_thr_out, int w_size );
 /***********************************************************************************************************************/
};

#endif // COMMONFUNC_H
