#include "myKinect.h"
#include <iostream>

/// Initializes the default Kinect sensor
HRESULT CBodyBasics::InitializeDefaultSensor()
{

    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr)){
        return hr;
    }


    if (m_pKinectSensor)
    {
        // Initialize the Kinect and get coordinate mapper and the body reader
        IBodyFrameSource* pBodyFrameSource = NULL;
        IDepthFrameSource* pDepthFrameSource = NULL;
        IBodyIndexFrameSource* pBodyIndexFrameSource = NULL;


        hr = m_pKinectSensor->Open();

        //coordinatemapper
        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
        }

        //bodyframe
        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
        }

        //depth frame
        if (SUCCEEDED(hr)){
            hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
        }

        if (SUCCEEDED(hr)){
            hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
        }

        //body index frame
        if (SUCCEEDED(hr)){
            hr = m_pKinectSensor->get_BodyIndexFrameSource(&pBodyIndexFrameSource);
        }

        if (SUCCEEDED(hr)){
            hr = pBodyIndexFrameSource->OpenReader(&m_pBodyIndexFrameReader);
        }

        SafeRelease(pBodyFrameSource);
        SafeRelease(pDepthFrameSource);
        SafeRelease(pBodyIndexFrameSource);
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        std::cout << "Kinect initialization failed!" << std::endl;
        return E_FAIL;
    }


    skeletonImg.create(cDepthHeight, cDepthWidth, CV_8UC3);
    skeletonImg.setTo(0);


    depthImg.create(cDepthHeight, cDepthWidth, CV_8UC1);
    depthImg.setTo(0);

    return hr;
}


/// Main processing function
void CBodyBasics::Update()
{

    skeletonImg.setTo(0);


    if (!m_pBodyFrameReader)
    {
        return;
    }

    IBodyFrame* pBodyFrame = NULL;
    IDepthFrame* pDepthFrame = NULL;
    IBodyIndexFrame* pBodyIndexFrame = NULL;


    HRESULT hr = S_OK;


    if (SUCCEEDED(hr)){
        hr = m_pBodyIndexFrameReader->AcquireLatestFrame(&pBodyIndexFrame);
    }
    if (SUCCEEDED(hr)){
        BYTE *bodyIndexArray = new BYTE[cDepthHeight * cDepthWidth];
        pBodyIndexFrame->CopyFrameDataToArray(cDepthHeight * cDepthWidth, bodyIndexArray);


        uchar* skeletonData = (uchar*)skeletonImg.data;
        for (int j = 0; j < cDepthHeight * cDepthWidth; ++j){
            *skeletonData = bodyIndexArray[j]; ++skeletonData;
            *skeletonData = bodyIndexArray[j]; ++skeletonData;
            *skeletonData = bodyIndexArray[j]; ++skeletonData;
        }
        delete[] bodyIndexArray;
    }
    SafeRelease(pBodyIndexFrame);


    if (SUCCEEDED(hr)){
        hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
    }
    if (SUCCEEDED(hr)){
        UINT16 *depthArray = new UINT16[cDepthHeight * cDepthWidth];
        pDepthFrame->CopyFrameDataToArray(cDepthHeight * cDepthWidth, depthArray);


        uchar* depthData = (uchar*)depthImg.data;
        for (int j = 0; j < cDepthHeight * cDepthWidth; ++j){
            *depthData = depthArray[j];
            ++depthData;
        }
        delete[] depthArray;
    }
    SafeRelease(pDepthFrame);
    imshow("depthImg", depthImg);
    cv::waitKey(5);

    if (SUCCEEDED(hr)){
        hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
    }
    if (SUCCEEDED(hr))
    {
        IBody* ppBodies[BODY_COUNT] = { 0 };

        if (SUCCEEDED(hr))
        {

            hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
        }

        if (SUCCEEDED(hr))
        {

            ProcessBody(BODY_COUNT, ppBodies);
        }

        for (int i = 0; i < _countof(ppBodies); ++i)
        {
            SafeRelease(ppBodies[i]);
        }
    }
    SafeRelease(pBodyFrame);

}

/// Handle new body data
void CBodyBasics::ProcessBody(int nBodyCount, IBody** ppBodies)
{

    HRESULT hr;


    for (int i = 0; i < nBodyCount; ++i)
    {
        IBody* pBody = ppBodies[i];
        if (pBody)
        {
            BOOLEAN bTracked = false;
            hr = pBody->get_IsTracked(&bTracked);

            if (SUCCEEDED(hr) && bTracked)
            {
                Joint joints[JointType_Count];
                HandState leftHandState = HandState_Unknown;
                HandState rightHandState = HandState_Unknown;


                pBody->get_HandLeftState(&leftHandState);
                pBody->get_HandRightState(&rightHandState);


                DepthSpacePoint *depthSpacePosition = new DepthSpacePoint[_countof(joints)];


                hr = pBody->GetJoints(_countof(joints), joints);
                if (SUCCEEDED(hr))
                {
                    for (int j = 0; j < _countof(joints); ++j)
                    {

                        m_pCoordinateMapper->MapCameraPointToDepthSpace(joints[j].Position, &depthSpacePosition[j]);
                    }


                    DrawHandState(depthSpacePosition[JointType_HandLeft], leftHandState);
                    DrawHandState(depthSpacePosition[JointType_HandRight], rightHandState);


                    DrawBone(joints, depthSpacePosition, JointType_Head, JointType_Neck);
                    DrawBone(joints, depthSpacePosition, JointType_Neck, JointType_SpineShoulder);
                    DrawBone(joints, depthSpacePosition, JointType_SpineShoulder, JointType_SpineMid);
                    DrawBone(joints, depthSpacePosition, JointType_SpineMid, JointType_SpineBase);
                    DrawBone(joints, depthSpacePosition, JointType_SpineShoulder, JointType_ShoulderRight);
                    DrawBone(joints, depthSpacePosition, JointType_SpineShoulder, JointType_ShoulderLeft);
                    DrawBone(joints, depthSpacePosition, JointType_SpineBase, JointType_HipRight);
                    DrawBone(joints, depthSpacePosition, JointType_SpineBase, JointType_HipLeft);

                    // -----------------------Right Arm ------------------------------------
                    DrawBone(joints, depthSpacePosition, JointType_ShoulderRight, JointType_ElbowRight);
                    DrawBone(joints, depthSpacePosition, JointType_ElbowRight, JointType_WristRight);
                    DrawBone(joints, depthSpacePosition, JointType_WristRight, JointType_HandRight);
                    DrawBone(joints, depthSpacePosition, JointType_HandRight, JointType_HandTipRight);
                    DrawBone(joints, depthSpacePosition, JointType_WristRight, JointType_ThumbRight);

                    //----------------------------------- Left Arm--------------------------
                    DrawBone(joints, depthSpacePosition, JointType_ShoulderLeft, JointType_ElbowLeft);
                    DrawBone(joints, depthSpacePosition, JointType_ElbowLeft, JointType_WristLeft);
                    DrawBone(joints, depthSpacePosition, JointType_WristLeft, JointType_HandLeft);
                    DrawBone(joints, depthSpacePosition, JointType_HandLeft, JointType_HandTipLeft);
                    DrawBone(joints, depthSpacePosition, JointType_WristLeft, JointType_ThumbLeft);

                    // ----------------------------------Right Leg--------------------------------
                    DrawBone(joints, depthSpacePosition, JointType_HipRight, JointType_KneeRight);
                    DrawBone(joints, depthSpacePosition, JointType_KneeRight, JointType_AnkleRight);
                    DrawBone(joints, depthSpacePosition, JointType_AnkleRight, JointType_FootRight);

                    // -----------------------------------Left Leg---------------------------------
                    DrawBone(joints, depthSpacePosition, JointType_HipLeft, JointType_KneeLeft);
                    DrawBone(joints, depthSpacePosition, JointType_KneeLeft, JointType_AnkleLeft);
                    DrawBone(joints, depthSpacePosition, JointType_AnkleLeft, JointType_FootLeft);
                }
                delete[] depthSpacePosition;
            }
        }
    }
    cv::imshow("skeletonImg", skeletonImg);
    cv::waitKey(5);
}

void CBodyBasics::DrawHandState(const DepthSpacePoint depthSpacePosition, HandState handState)
{

    CvScalar color;
    switch (handState){
    case HandState_Open:
        color = cvScalar(255, 0, 0);
        break;
    case HandState_Closed:
        color = cvScalar(0, 255, 0);
        break;
    case HandState_Lasso:
        color = cvScalar(0, 0, 255);
        break;
    default:
        return;
    }

    circle(skeletonImg,
        cvPoint(depthSpacePosition.X, depthSpacePosition.Y),
        20, color, -1);
}


/// Draws one bone of a body (joint to joint)
void CBodyBasics::DrawBone(const Joint* pJoints, const DepthSpacePoint* depthSpacePosition, JointType joint0, JointType joint1)
{
    TrackingState joint0State = pJoints[joint0].TrackingState;
    TrackingState joint1State = pJoints[joint1].TrackingState;

    // If we can't find either of these joints, exit
    if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
    {
        return;
    }

    // Don't draw if both points are inferred
    if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
    {
        return;
    }

    CvPoint p1 = cvPoint(depthSpacePosition[joint0].X, depthSpacePosition[joint0].Y),
        p2 = cvPoint(depthSpacePosition[joint1].X, depthSpacePosition[joint1].Y);

    // We assume all drawn bones are inferred unless BOTH joints are tracked
    if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
    {

        line(skeletonImg, p1, p2, cvScalar(255, 255, 255));
    }
    else
    {

        line(skeletonImg, p1, p2, cvScalar(0, 0, 255));
    }
}


/// Constructor
CBodyBasics::CBodyBasics() :
m_pKinectSensor(NULL),
m_pCoordinateMapper(NULL),
m_pBodyFrameReader(NULL){}

/// Destructor
CBodyBasics::~CBodyBasics()
{
    SafeRelease(m_pBodyFrameReader);
    SafeRelease(m_pCoordinateMapper);

    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }
    SafeRelease(m_pKinectSensor);
}
