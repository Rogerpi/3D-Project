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
    //Constructor
    Kinect_RGBD_Grabber();

    //Destructor
    ~Kinect_RGBD_Grabber();

    // Read depth frame from Kinect and return it as a cv::Mat
    cv::Mat GetDepthFrame();

    // Read color frame from Kinect and return it as a cv::Mat
    cv::Mat GetColorFrame();

    //Convert color image to depth space (same resolution and coordinate systems).  colorBytesPerPixel is 4 because images are received as RGBA
    cv::Mat map_depth_to_color(cv::Mat& depth_im, cv::Mat& rgb_im, int colorBytesPerPixel = 4);


private:

    //Init Kinect depth extractor
    void InitDepthSource();

    //Init Kinect color extractor
    void InitColorSource();

    //Init Kinect mapper to convert coordinate systems
    void InitMapper();

    //This function will wait until depth frame is available (all data loaded in buffer)
    IDepthFrame* WaitForDepthFrame();

    //This function will wait until color frame is available (all data loaded in buffer)
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
