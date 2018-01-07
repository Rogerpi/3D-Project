
#include "kinect_rgbd_grabber.h"

// Constructor destructor
Kinect_RGBD_Grabber::Kinect_RGBD_Grabber()
{
    HRESULT hr = GetDefaultKinectSensor(&kin_sensor_);
    if (FAILED(hr))
    {
        throw std::runtime_error("Kinect sensor could not be found!");
    }

    if (kin_sensor_)
    {
        InitDepthSource();
        InitColorSource();
        InitMapper();
    }

    if (!kin_sensor_ || FAILED(hr))
    {
        throw std::runtime_error("Kinect init failed!");
    }
}

Kinect_RGBD_Grabber::~Kinect_RGBD_Grabber()
{
    SafeRelease(color_frame_reader_);
    SafeRelease(depth_frame_reader_);

    // Close the Kinect Sensor
    if (kin_sensor_)
    {
        kin_sensor_->Close();
    }

    SafeRelease(kin_sensor_);
}

//Private

void Kinect_RGBD_Grabber::InitDepthSource()
{
    IDepthFrameSource* depth_frame_source = nullptr;

    HRESULT hr = kin_sensor_->Open();

    if (SUCCEEDED(hr))
    {
        hr = kin_sensor_->get_DepthFrameSource(&depth_frame_source);
    }

    if (SUCCEEDED(hr))
    {
        hr = depth_frame_source->OpenReader(&depth_frame_reader_);
    }

    SafeRelease(depth_frame_source);
}

void Kinect_RGBD_Grabber::InitMapper(){

     int hr = kin_sensor_->get_CoordinateMapper(&mapper);
}

void Kinect_RGBD_Grabber::InitColorSource()
{
    IColorFrameSource* color_frame_source = nullptr;

    HRESULT hr = kin_sensor_->Open();

    if (SUCCEEDED(hr))
    {
        hr = kin_sensor_->get_ColorFrameSource(&color_frame_source);
    }

    if (SUCCEEDED(hr))
    {
        hr = color_frame_source->OpenReader(&color_frame_reader_);
    }

    SafeRelease(color_frame_source);
}

IDepthFrame* Kinect_RGBD_Grabber::WaitForDepthFrame()
{
    while (true)
    {
        IDepthFrame* depth_frame = nullptr;
        HRESULT hr = depth_frame_reader_->AcquireLatestFrame(&depth_frame);

        if (SUCCEEDED(hr))
        {
            return depth_frame;
        }

        SafeRelease(depth_frame);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

IColorFrame* Kinect_RGBD_Grabber::WaitForColorFrame()
{
    while (true)
    {
        IColorFrame* color_frame = nullptr;
        HRESULT hr = color_frame_reader_->AcquireLatestFrame(&color_frame);

        if (SUCCEEDED(hr))
        {
            return color_frame;
        }

        SafeRelease(color_frame);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}


//Public

cv::Mat Kinect_RGBD_Grabber::GetDepthFrame(){
    IDepthFrame* depth_frame = WaitForDepthFrame();

    IFrameDescription* frame_desc = nullptr;
    UINT nBufferSize = 0;
    pBuffer = NULL;

    HRESULT hr = depth_frame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);

    cv::Mat depth_mat(depth_h_, depth_w_, CV_16UC1);

    memcpy(depth_mat.data, pBuffer, nBufferSize * sizeof(uint16_t));

    SafeRelease(depth_frame);

    return depth_mat;
}


cv::Mat Kinect_RGBD_Grabber::GetColorFrame(){

    IColorFrame* color_frame = WaitForColorFrame();
    ColorImageFormat imageFormat = ColorImageFormat_None;
    HRESULT hr = color_frame->get_RawColorImageFormat(&imageFormat);

    cv::Mat color_mat(color_h_, color_w_, CV_8UC4);
    const int buf_size = color_h_ * color_w_ * sizeof(uint8_t) * 4;
    hr = color_frame->CopyConvertedFrameDataToArray(buf_size, color_mat.data, ColorImageFormat_Bgra);

    SafeRelease(color_frame);

    return color_mat;
}


cv::Mat Kinect_RGBD_Grabber::map_depth_to_color(cv::Mat& depth_im, cv::Mat& rgb_im, int colorBytesPerPixel ){

    // Retrieve Mapped Coordinates
    std::vector<ColorSpacePoint> colorSpacePoints(depth_w_ * depth_h_ );
    mapper->MapDepthFrameToColorSpace(depth_w_ * depth_h_, (UINT16*)depth_im.data, colorSpacePoints.size(), &colorSpacePoints[0] );

    // Mapped Color Buffer
    std::vector<BYTE> buffer( depth_w_ * depth_h_ * colorBytesPerPixel );

    // Mapping Color Data to Depth Resolution
    for( int depthY = 0; depthY < depth_h_; depthY++ ){
        for( int depthX = 0; depthX < depth_w_; depthX++ ){
            const unsigned int depthIndex = depthY * depth_w_ + depthX;
            const int colorX = static_cast<int>( colorSpacePoints[depthIndex].X + 0.5f );
            const int colorY = static_cast<int>( colorSpacePoints[depthIndex].Y + 0.5f );
            if( ( 0 <= colorX ) && ( colorX < color_w_ ) && ( 0 <= colorY ) && ( colorY < color_h_ ) ){
                const unsigned int colorIndex = colorY * color_w_ + colorX;
                buffer[depthIndex * colorBytesPerPixel + 0] = rgb_im.data[colorIndex * colorBytesPerPixel + 0];
                buffer[depthIndex * colorBytesPerPixel + 1] = rgb_im.data[colorIndex * colorBytesPerPixel + 1];
                buffer[depthIndex * colorBytesPerPixel + 2] = rgb_im.data[colorIndex * colorBytesPerPixel + 2];
                buffer[depthIndex * colorBytesPerPixel + 3] = rgb_im.data[colorIndex * colorBytesPerPixel + 3];
            }
        }
    }


    // e.g. Mapped Color Buffer to cv::Mat
    cv::Mat colorMat = cv::Mat( depth_h_, depth_w_, CV_8UC4, &buffer[0] ).clone();
    return colorMat;

}




