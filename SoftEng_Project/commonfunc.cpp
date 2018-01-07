#include "commonfunc.h"
#include <opencv/highgui.h>

// Feature detection function headers
#include <opencv2/features2d/features2d.hpp>

#include <opencv2/core/core.hpp>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
commonFunc::commonFunc()
{

}

void commonFunc::dispDepthColor(cv::Mat &depthMap, cv::Mat &depthColor)
{
  double min;
  double max;

  // Get minimum and maximum values
  cv::minMaxIdx(depthMap, &min, &max);

  // Histogram Equalization
  cv::Mat adjMap;
  float scale = 255 / (max-min);
  depthMap.convertTo(adjMap,CV_8UC1, scale, -min*scale);


  // Convert to color map display
  cv::applyColorMap(adjMap, depthColor, cv::COLORMAP_JET);
}

void commonFunc::dispDepthGray(cv::Mat &depthMap, cv::Mat &depthGray)
{
  // normalize depth map to uint8 values
  cv::normalize(depthMap, depthGray, 0, 255, CV_MINMAX);
  depthGray.convertTo(depthGray, CV_8UC3);
}

void commonFunc::rgb2gray(cv::Mat &rgbImg, cv::Mat &grayImg)
{
  cv::cvtColor(rgbImg, grayImg, cv::COLOR_RGB2GRAY);
}

// Flip image left to right
void commonFunc::flipImage(cv::Mat &rgbImg)
{
  cv::flip(rgbImg, rgbImg, 1);
}

////////////////////////////////////////////////////////////////
/////////////        Image feature detection       /////////////
////////////////////////////////////////////////////////////////
void commonFunc::featureExtraction
(cv::Mat &rgbImg, std::vector<cv::KeyPoint> &keyPts)
{
  std::cout<<"Start feature detection...\n";
  // Extract SIFT features
  cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();
   cv::Mat grayImg; rgb2gray(rgbImg, grayImg);
  f2d->detect(grayImg,keyPts);

  std::cout<<"Number of detected key points: "<<keyPts.size()<<std::endl;
  /*
  cv::SiftFeatureDetector siftDetector(100, 5);
  cv::Mat grayImg; rgb2gray(rgbImg, grayImg);
  // Remember to convert RGB image to Grayscale image beforehand
  siftDetector.detect(grayImg, keyPts);
  std::cout<<"Number of detected key points: "<<keyPts.size()<<std::endl;
    */
}

void commonFunc::showKeyPoints
(cv::Mat &rgbImg, std::vector<cv::KeyPoint> &keyPts){
  //-- Draw keypoints
  cv::Mat img_keyPts;

  drawKeypoints( rgbImg, keyPts, img_keyPts, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

  //-- Show detected (drawn) keypoints
  cv::imshow("Keypoints", img_keyPts );
  //cv::waitKey(0);
  std::cout<<"End feature detection...\n";
}

////////////////////////////////////////////////////////////////
///////////// Image feature detection and matching /////////////
////////////////////////////////////////////////////////////////
// This is a simplified version of the following codes
// https://docs.opencv.org/2.4/doc/tutorials/features2d/feature_flann_matcher/feature_flann_matcher.html
//
// Understanding SIFT descriptor
// https://docs.opencv.org/3.1.0/da/df5/tutorial_py_sift_intro.html
//
// Distance metrics:
// https://docs.opencv.org/2.4/doc/tutorials/imgproc/histograms/histogram_comparison/histogram_comparison.html
void commonFunc::featureMatching
(cv::Mat &rgb_1, std::vector<cv::KeyPoint> &keyPts_1,
 cv::Mat &rgb_2, std::vector<cv::KeyPoint> &keyPts_2,
 std::vector< cv::DMatch > *matches, bool robustMatch)
{
  std::cout<<"Start feature matching...\n";
  //-- Extract SIFT features

  cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();
  cv::Mat gray_1; rgb2gray(rgb_1, gray_1);
  cv::Mat gray_2; rgb2gray(rgb_2, gray_2);
  f2d->detect(gray_1,keyPts_1);
  f2d->detect(gray_2,keyPts_2);

  /* OLD (Opencv 2.x)
  cv::SiftFeatureDetector siftDetector(100, 5);
  cv::Mat gray_1; rgb2gray(rgb_1, gray_1);
  cv::Mat gray_2; rgb2gray(rgb_2, gray_2);
  siftDetector.detect(gray_1, keyPts_1);
  siftDetector.detect(gray_2, keyPts_2);
*/
  //-- Draw keypoints
  cv::Mat img_keyPts_1;
  cv::Mat img_keyPts_2;
  drawKeypoints( rgb_1, keyPts_1, img_keyPts_1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
  drawKeypoints( rgb_2, keyPts_2, img_keyPts_2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );

  //-- Compute descriptor
  cv::Ptr<cv::xfeatures2d::SIFT> extractor = cv::xfeatures2d::SiftDescriptorExtractor::create();
  cv::Mat descriptors_1, descriptors_2;
  extractor->compute( gray_1, keyPts_1, descriptors_1 );
  extractor->compute( gray_2, keyPts_2, descriptors_2 );

  std::cout<<"descriptors type: "<<descriptors_1.type()<<std::endl;


  /* OLD (Opencv 2.x)
  cv::SiftDescriptorExtractor siftDesExtractor;
  cv::Mat descriptors_1, descriptors_2;
  siftDesExtractor.compute( gray_1, keyPts_1, descriptors_1 );
  siftDesExtractor.compute( gray_2, keyPts_2, descriptors_2 );
  std::cout<<"descriptors type: "<<descriptors_1.type()<<std::endl;
   */
  //-- Feature matching using descriptors
  cv::FlannBasedMatcher matcher;

  if ( descriptors_1.empty() || descriptors_2.empty() )
  {
      matches->clear(); //set to 0
      return;
  }
  matcher.match( descriptors_1, descriptors_2, *matches );

// SIFT descriptor has type 5: 32F
//  +--------+----+----+----+----+------+------+------+------+
//  |        | C1 | C2 | C3 | C4 | C(5) | C(6) | C(7) | C(8) |
//  +--------+----+----+----+----+------+------+------+------+
//  | CV_8U  |  0 |  8 | 16 | 24 |   32 |   40 |   48 |   56 |
//  | CV_8S  |  1 |  9 | 17 | 25 |   33 |   41 |   49 |   57 |
//  | CV_16U |  2 | 10 | 18 | 26 |   34 |   42 |   50 |   58 |
//  | CV_16S |  3 | 11 | 19 | 27 |   35 |   43 |   51 |   59 |
//  | CV_32S |  4 | 12 | 20 | 28 |   36 |   44 |   52 |   60 |
//  | CV_32F |  5 | 13 | 21 | 29 |   37 |   45 |   53 |   61 |
//  | CV_64F |  6 | 14 | 22 | 30 |   38 |   46 |   54 |   62 |
//  +--------+----+----+----+----+------+------+------+------+
  std::cout<<"Matched descriptors: "<<matches->size()<<std::endl;

  if(robustMatch)
    {
      std::cout<<"\n Start outlier removal...\n";
      double max_dist = 0; double min_dist = 100;
      //-- Quick calculation of max and min distances between keypoints
         for( int i = 0; i < descriptors_1.rows; i++ )
       { double dist = (*matches)[i].distance;
         if( dist < min_dist ) min_dist = dist;
         if( dist > max_dist ) max_dist = dist;
       }
       std::cout<<"max descriptor distance = "<<max_dist<<", min descriptor disance = " <<min_dist<<std::endl;
       // Only consider matches with small distances
       std::vector< cv::DMatch > good_matches;
       for( int i = 0; i < descriptors_1.rows; i++ )
       { if( (*matches)[i].distance <= cv::max(10*min_dist, 0.02) )
         { good_matches.push_back( (*matches)[i]); }
       }
       matches->clear();
       for(int i = 0; i<good_matches.size(); i++ )
         {
           matches->push_back( good_matches[i]);
         }
       std::cout<<"Good matches number: "<<matches->size()<<std::endl;
    }
  std::cout<<"End feature matching...\n";
}

cv::Mat commonFunc::showFeatureMatches
(cv::Mat &rgb_1, std::vector<cv::KeyPoint> &keyPts_query,
 cv::Mat &rgb_2, std::vector<cv::KeyPoint> &keyPts_train,
 std::vector< cv::DMatch > *matches)
{
  //-- Show detected (drawn) matches
  cv::Mat img_matches;
  hconcat(rgb_1,rgb_2,img_matches);

  int diam = 5;


  //for(int i=0; i<std::min(n_matches,int(matches->size())); i++) // x
 for (int i = 0; i <int(matches->size()) ;i++ )
    {

      cv::Point2f pt1 = keyPts_query[(*matches)[i].queryIdx].pt;
      cv::Point2f pt2 = keyPts_train[(*matches)[i].trainIdx].pt;

      int feat_query_y = (int) pt1.x;
      int feat_query_x = (int )pt1.y;
      int feat_train_y = (int) pt2.x;
      int feat_train_x = (int )pt2.y;
      // big gray circle.

      //Draw match
      cv::Scalar color(rand() % 255,rand() % 255,rand() % 255);
      cv::circle(img_matches, cv::Point(feat_query_y, feat_query_x), (diam/2),color);

      cv::circle(img_matches, cv::Point(feat_train_y+rgb_1.cols, feat_train_x), (diam/2),color);

      cv::line(img_matches,cv::Point(feat_query_y, feat_query_x),cv::Point(feat_train_y+rgb_1.cols, feat_train_x),color);


    }

  return img_matches;


  //-- Show detected matches
  //imshow( "Feature Matches", img_matches );
  //cv::waitKey(0);
}

////////////////////////////////////////////////////////////////
/////////// RGB-D data to 3D Point Cloud Rendering /////////////
////////////////////////////////////////////////////////////////

void commonFunc::myDepth2meter(const float feat_x, const float feat_y, const float rawDisparity,
                             float &X, float &Y, float &Z)
{
  // reject invalid points
  if(rawDisparity <= 0)
    {
      X = 0; Y = 0; Z = 0; return;
    }

  float fx = 525.0; // focal length x
  float fy = 525.0; // focal length y
  float cx = 319.5; // optical center x
  float cy = 239.5; // optical center y
  float sclFactor = 1000.0;

  // Recall the camera projective projection model
  Z = rawDisparity / sclFactor;
  X = (feat_x - cx) * Z / fx;
  Y = (feat_y - cy) * Z / fy;
}

////////////////////////////////////////////////////////////////
///////////// Kinect RGB-D to 3D Point Conversion  /////////////
////////////////////////////////////////////////////////////////
// Kinect's depth value is 16-bit, but only the low-12-bit store real distance

pcl::PointCloud<pcl::PointXYZRGB>::Ptr commonFunc::rgbd2pcl(const cv::Mat &rgbImg, const cv::Mat &depthImg, const double min_dist_thr, const double max_dist_thr){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->points.resize (rgbImg.rows*rgbImg.cols);
    int p_count = 0;
    for(int i=0; i<rgbImg.rows; i++) // x
      {
        for(int j=0; j<rgbImg.cols; j++) // y
          {
            float X, Y, Z;
            unsigned short depth = depthImg.at<unsigned short>(i, j);
            // Render the 3D values
            myDepth2meter(i,j,depth, X, Y, Z);
            // Remove features which are out of Kinect senser range
            if(X>5 || Y > 5 || Z <= std::max(0.0, min_dist_thr) || Z > max_dist_thr){continue; }
            // Write out the colored 3D point
            cloud->points[p_count].x = X;
            cloud->points[p_count].y = Y;
            cloud->points[p_count].z = Z;


            cloud->points[p_count].r = (float)rgbImg.at<cv::Vec3b>(i,j)[2];
            cloud->points[p_count].g = (float)rgbImg.at<cv::Vec3b>(i,j)[1];
            cloud->points[p_count].b = (float)rgbImg.at<cv::Vec3b>(i,j)[0];
            p_count++;

          }
      }
      return cloud;


}


cv::Mat commonFunc::threshold_mapped_depth_to_color(cv::Mat& depth_im,cv::Mat& color_im, const double min_m_thr, const double max_m_thr){
    float sclFactor = 1000.0;
    float depth = 0;
    cv::Mat thr_color = color_im.clone();

    for(int i=0; i<depth_im.rows; i++) // x
      {
        for(int j=0; j<depth_im.cols; j++) // y
        {
            depth = depth_im.at<unsigned short>(i, j)/sclFactor;
            if ( depth <= min_m_thr || depth > max_m_thr){
                thr_color.at<cv::Vec3b>(i,j)[0] = 0;
                thr_color.at<cv::Vec3b>(i,j)[1] = 0;
                thr_color.at<cv::Vec3b>(i,j)[2] = 0;
            }

         }
     }

     return thr_color;
}

void commonFunc::threshold_depth(cv::Mat& depth_im, cv::Mat& depth_thr, const double min_m_thr, const double max_m_thr){
    float sclFactor = 1000.0;
    float depth = 0;

    depth_thr = depth_im.clone();
    for(int i=0; i<depth_im.rows; i++) // x
      {
        for(int j=0; j<depth_im.cols; j++) // y
        {
            depth = depth_im.at<unsigned short>(i, j)/sclFactor;
            if ( depth < min_m_thr || depth > max_m_thr){
                depth_thr.at<unsigned short>(i,j) = 0;

            }

         }
     }

}

void commonFunc::border_outlier_removal(cv::Mat &depth_thr_in,cv::Mat &depth_thr_out, int w_size)
{
    depth_thr_out = depth_thr_in.clone();
    cv::Mat depthGray;
    cv::normalize(depth_thr_in, depthGray, 0, 255, CV_MINMAX);
    depthGray.convertTo(depthGray, CV_8UC3);

    cv::threshold(depthGray,depthGray,0,255,CV_THRESH_BINARY);
    //cv::imshow("depth gray binary",depthGray);

    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*w_size + 1, 2*w_size+1 ), cv::Point( w_size,w_size ) );

    /// Apply the erosion operation
    cv::erode( depthGray, depthGray, element );

   // cv::imshow("depth gray binary erode",depthGray);

    for(int i=0; i<depth_thr_in.rows; i++) // x
      {
        for(int j=0; j<depth_thr_in.cols; j++) // y
        {

            if ( depthGray.at<uchar>(i, j) == 0)
                depth_thr_out.at<unsigned short>(i,j) = 0;

         }
     }

}



