
#include <Kinect.h>
#include <opencv2\opencv.hpp>


template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
    if (pInterfaceToRelease != NULL)
    {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}

class CBodyBasics
{

    static const int        cDepthWidth = 512;
    static const int        cDepthHeight = 424;

public:
    CBodyBasics();
    ~CBodyBasics();
    void                    Update();
    HRESULT                 InitializeDefaultSensor();

private:
    IKinectSensor*          m_pKinectSensor;
    ICoordinateMapper*      m_pCoordinateMapper;
    IBodyFrameReader*       m_pBodyFrameReader;
    IDepthFrameReader*      m_pDepthFrameReader;
    IBodyIndexFrameReader*  m_pBodyIndexFrameReader;


    void                    ProcessBody(int nBodyCount, IBody** ppBodies);

    void DrawBone(const Joint* pJoints, const DepthSpacePoint* depthSpacePosition, JointType joint0, JointType joint1);

    void DrawHandState(const DepthSpacePoint depthSpacePosition, HandState handState);

    cv::Mat skeletonImg;
    cv::Mat depthImg;
};
