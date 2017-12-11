#include "pclviewer.h"
#include <QApplication>
#include <QMainWindow>
#include <iostream>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <C:\Program Files\OpenNI2\Include\OpenNI.h>

#include <opencv2/video/video.hpp>
#include <opencv/highgui.h>

#include "commonfunc.h"

// 1. Understanding OpenCV Documentation (corresponding OpenCV version with C++)
// 2. Data accessing of OpenCV matrix (data type conversion is very important!!!)
// 3. Remember to link and include the corresponding header and libs.
// 4. Namespace confliction, e.g. std::vector or cv::vector
// 5. Comment your codes.

// Some tricks
// Qt short cut:
// vertical selection: Shift + alt + arrow
// indentation: Ctrl A + i
// auto-completion: Tab or Ctrl + Space
// Shifting between functions: F2
//
// OpenCV cv::waitkey(0): if you call this function outside the main function,
//                        you need to kill the window before next step.
//
// *** Understand the parameter before tuning it ***
//
// C++ skills
// *** Pass reference address of parameters ***
// Defining static function and default arguments
// Dynamic allocation and memory release
//
// Passing a pointer vs. passing reference
// A pointer can be null, but a reference NOT.
//
// Static member function:
// 1. because static member functions are not attached to an object,
//    they have no this pointer!
// 2. static member functions can only access static member variables.
// They can not access non-static member variables.

int main (int argc, char *argv[])
{
  QApplication a (argc, argv);
  std::cout<<"OpenCV Toy Tutorials"<<std::endl;

  //////////////////////////////////////////////////////////
  ///////////// Video Capturing using Kinect ///////////////
  //////////////////////////////////////////////////////////
  //  https://docs.opencv.org/2.4.13.2/doc/user_guide/ug_kinect.html

  //////////////////////////////////////////////////////////
  ///////////// Load Kinect Data from File   ///////////////
  //////////////////////////////////////////////////////////
  cv::Mat depthMap = cv::imread("../QtOpenCVExample/kinect_data/depth/frame_01.png", CV_LOAD_IMAGE_UNCHANGED);
  cv::Mat colorImg = cv::imread("../QtOpenCVExample/kinect_data/rgb/frame_01.png", CV_LOAD_IMAGE_UNCHANGED);
  cv::Mat depthGray;
  cv::Mat depthColor;

  // Static function requires no object creation
  commonFunc::static_dispDepthGray(depthMap, depthGray);
  // Display depth image using Gray scale
  cv::imshow("depthMap 1", depthGray);

  // Non-static function requires object creation
  commonFunc imgProcessing;

  imgProcessing.dispDepthColor(depthMap, depthColor);
  // Display depth image using Color scale
  //cv::imshow("depthColor", depthColor);

  // Display color image
  cv::imshow("imgColor 1", colorImg);


  // Display color image
  cv::Mat grayImg;
  imgProcessing.rgb2gray(colorImg, grayImg);
  //cv::imshow("imgGray", grayImg);


  //////////////////////////////////////////////////////////
  ///////////// OpenCV Image Processing      ///////////////
  //////////////////////////////////////////////////////////
  // Feature point detection and display on the image
  std::vector<cv::KeyPoint> keyPts; keyPts.clear();
  imgProcessing.featureExtraction(colorImg, keyPts);
  imgProcessing.showKeyPoints(colorImg, keyPts);

  // Load second image for feature matching examples
  cv::Mat depthMap_2 = cv::imread("../QtOpenCVExample/kinect_data/depth/frame_02.png", CV_LOAD_IMAGE_UNCHANGED);
  cv::Mat colorImg_2 = cv::imread("../QtOpenCVExample/kinect_data/rgb/frame_02.png", CV_LOAD_IMAGE_UNCHANGED);
  std::cout<<"reading 2nd images"<<std::endl;
  commonFunc::static_dispDepthGray(depthMap_2, depthGray);
  cv::imshow("depthMap 2", depthGray);
  cv::imshow("color image 2", colorImg_2);
  //cv::waitKey(0);

  std::vector<cv::KeyPoint> keyPts_1, keyPts_2; keyPts_1.clear(); keyPts_2.clear();
  std::vector< cv::DMatch > *matches = new std::vector< cv::DMatch >;
  imgProcessing.featureMatching(colorImg, keyPts_1, colorImg_2, keyPts_2, matches);

  if(matches->size()>0)
    {
      std::cout<<"matches number "<<matches->size()<<std::endl;
      // Show feature matches
      imgProcessing.showFeatureMatches(colorImg, keyPts_1, colorImg_2, keyPts_2, matches);
    }

  bool robustMatch = true;

  imgProcessing.featureMatching(colorImg, keyPts_1, colorImg_2, keyPts_2, matches, robustMatch);

  if(matches->size()>0)
    {
      std::cout<<"matches number "<<matches->size()<<std::endl;
      // Show feature matches
      imgProcessing.showFeatureMatches(colorImg, keyPts_1, colorImg_2, keyPts_2, matches);
    }


  // Save feature points
  imgProcessing.saveFeatures(colorImg, depthMap, keyPts_1,
                             colorImg_2, depthMap_2, keyPts_2, matches);

  // Convert RGB-D data to colored-cloud
  std::string colorCloud_1 = "../QtOpenCVExample/kinect_data/rgbCloud_1.txt";
  imgProcessing.rgbd2pointcloud(colorImg, depthMap, colorCloud_1);
  std::string colorCloud_2 = "../QtOpenCVExample/kinect_data/rgbCloud_2.txt";
  imgProcessing.rgbd2pointcloud(colorImg_2, depthMap_2, colorCloud_2);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = imgProcessing.rgbd2pcl(colorImg, depthMap);
  PCLViewer w;
  w.setPointCloud(cloud);
  w.show ();



std::cout<<"PointCloud size = "<<cloud->size()<<endl;
 cv::waitKey(0);
  std::cout<<"Program killed..."<<std::endl;
  return 0;//a.exec ();
}
