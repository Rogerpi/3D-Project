//*Author: ROGER PI ROIG u1926981@gmail.com *//
// Free to use but keep author information //

#include <Kinect.h>
#include <stdexcept>
#include <iostream>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

#ifndef KINECT_RGBD_GRABBER_H
#define KINECT_RGBD_GRABBER_H

// Safe release for Windows interfaces
template<class Interface>
inline void SafeRelease(Interface*& ptr_int)
{
    if (ptr_int)
    {
        ptr_int->Release();
        ptr_int = nullptr;
    }
}


class Kinect_RGBD_Grabber
{
public:
    Kinect_RGBD_Grabber();

    ~Kinect_RGBD_Grabber();

    cv::Mat GetDepthFrame();

    cv::Mat GetColorFrame();

    cv::Mat map_depth_to_color(cv::Mat& depth_im, cv::Mat& rgb_im, int colorBytesPerPixel = 4);


private:

    void InitDepthSource();

    void InitColorSource();

    void InitMapper();

    IDepthFrame* WaitForDepthFrame();

    IColorFrame* WaitForColorFrame();




    static const int depth_w_ = 512;  //512
    static const int depth_h_ = 424;  //424
    static const int color_w_ = 1920; //1920
    static const int color_h_ = 1080; //1080
    //Changing size will probably make it crash

    IKinectSensor* kin_sensor_;
    IDepthFrameReader* depth_frame_reader_;
    IColorFrameReader* color_frame_reader_;
    ICoordinateMapper* mapper;

    UINT16 *pBuffer;




};


#endif // KINECT_RGBD_GRABBER_H
