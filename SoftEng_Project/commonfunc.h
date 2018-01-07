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

  //Convrt depth matrix(depthMap) to color image(depthColor)
  void dispDepthColor(cv::Mat &depthMap, cv::Mat &depthColor);

  //Convert depth matrix(depthMap) to gray image(depthGray)
  void dispDepthGray(cv::Mat &depthMap, cv::Mat &depthGray);

  //Convert depth matrix(depthMap) to gray image(depthGray)
  static void static_dispDepthGray(cv::Mat &depthMap, cv::Mat &depthGray)
  {
    cv::normalize(depthMap, depthGray, 0, 255, CV_MINMAX);
    depthGray.convertTo(depthGray, CV_8UC3);
  }

  ////////////////////////////////////////////////////////////////
  //////////////// Opencv Image Processing Functions /////////////
  ////////////////////////////////////////////////////////////////

  // Convert color image(rgbImg) to gray scale image(grayImg)
  void rgb2gray(cv::Mat &rgbImg, cv::Mat &grayImg);

  // Flip image left and right
  void flipImage(cv::Mat &rgbImg);


  // Detect feature points on image(rgbImg). Keypoints are stored in (keyPts) vector
  void featureExtraction(cv::Mat &rgbImg, std::vector<cv::KeyPoint> &keyPts);

  // Show detected keypoints(keyPts) on the color image(rgbImg)
  void showKeyPoints(cv::Mat &rgbImg, std::vector<cv::KeyPoint> &keyPts);

  // Detect and match features between two images(rgbImg01, rgbImg02). Keypoints must be previously found(keyPts01,keyPts02). Matches are stored in (matches) vector
  void featureMatching(cv::Mat &rgbImg01, std::vector<cv::KeyPoint> &keyPts01,
                       cv::Mat &rgbImg02, std::vector<cv::KeyPoint> &keyPts02,
                       std::vector< cv::DMatch > *matches, bool robustMatch = false);

  /********************************************************************************************/

  /************ THIS Functions have been MODIFIED from Cansen OpenCV Tutorial *********************/

  // Draw the feature matches
  //This function has been modified to draw manually the matches to make sure we were taking the correct points (cx,cy)
  //Matches will be shown between images (rgb_1,rgb_2). Matches must be previously found and stores in (matches) array.
  cv::Mat showFeatureMatches(cv::Mat &rgb_1, std::vector<cv::KeyPoint> &keyPts_1,
                          cv::Mat &rgb_2, std::vector<cv::KeyPoint> &keyPts_2,
                          std::vector< cv::DMatch > *matches);

  //scalarFactor has been modified from 5000 to 1000 because kinect depth resolution is in mm
  //This function converts a image point (feat_x,feat_y) and depth value on 3D space (x,y,z)
  void myDepth2meter(const float feat_x, const float feat_y, const float rawDisparity,
                   float &x, float &y, float &z);

//This function has been modified to return a pcl pointcloud instead of saving points into a text file
//Convert a RGB and Depth image (rgbImg, depthImg) to PointCloud. min and max distance threshold can be defined (in meters)
pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbd2pcl(const cv::Mat &rgbImg, const cv::Mat &depthImg, const double min_dist_thr = 0 , const double max_dist_thr = 4.5);

/***********************************************************************************************************************/

/************************************ Image Processing Technics (Our Implementation) **************************************/

// Threshold depth matrix (depth_im) using min and max threshold (in meters). Result will be stored at (depth_thr) image. Notice that depths outside
// threshold range will be set at 0 value.
void threshold_depth(cv::Mat& depth_im, cv::Mat& depth_thr, const double min_m_thr, const double max_m_thr);

// Threshold color image (color_im) using min and max threshold (in meters). Result will be stored at the same(color_im) image. Notice that depths outside
// threshold range will be set at (0,0,0) value, gray.
cv::Mat threshold_mapped_depth_to_color(cv::Mat& depth_im,cv::Mat& color_im, const double min_m_thr, const double max_m_thr);

//Remove borders of depth thresholded image (depth_thr_in) and store it at (depth_thr_out). The threholded image is binarized, eroded and the result is
//used as a mask to remove borders (set it to 0). w_size argument correspond to the size of the erode window.
 void border_outlier_removal(cv::Mat & depth_thr_in,cv::Mat &depth_thr_out, int w_size );
 /***********************************************************************************************************************/
};

#endif // COMMONFUNC_H
