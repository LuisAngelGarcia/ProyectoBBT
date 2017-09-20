/*********************************************************/
//SISTEMA DE ENSAYO DE CAJA Y CUBOS AUTOMATIZADO (ABBT)//
	    //Luis Ángel García Astudillo// 
  //Grado en Ingeniería en Tecnologías Industriales//
         //Universidad Carlos III de Madrid//
/********************************************************/

#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "DepthBasics.h"
#include <opencv2/opencv.hpp>

using namespace cv;


CDepthBasics::CDepthBasics() :

    m_fFreq(0),
    m_pKinectSensor(NULL),
    m_pDepthFrameReader(NULL)
{
    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf)){
        m_fFreq = double(qpf.QuadPart);
    }

}
  

CDepthBasics::~CDepthBasics(){
    
   
    SafeRelease(m_pDepthFrameReader); // done with depth frame reader

    
    if (m_pKinectSensor){ // close the Kinect Sensor
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);
}


void CDepthBasics::update(Mat &m){
    if (!m_pDepthFrameReader){
        return;
    }

    IDepthFrame* pDepthFrame = NULL;

    HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

    if (SUCCEEDED(hr)){

		imageCaptured = 1;
        INT64 nTime = 0;
        IFrameDescription* pFrameDescription = NULL;
        int nWidth = 0;
        int nHeight = 0;
        USHORT nDepthMinReliableDistance = 0;
        USHORT nDepthMaxDistance = 0;
        const UINT nBufferSize = cDepthHeight*cDepthWidth;
		UINT16 pBuffer[nBufferSize];
        UINT16 depthB;

        hr = pDepthFrame->get_RelativeTime(&nTime);

        if (SUCCEEDED(hr)){
            hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
        }

        if (SUCCEEDED(hr)){
            hr = pFrameDescription->get_Width(&nWidth);
        }

        if (SUCCEEDED(hr)){
            hr = pFrameDescription->get_Height(&nHeight);
        }

        if (SUCCEEDED(hr)){
            hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
        }

        if (SUCCEEDED(hr)){
			// In order to see the full range of depth (including the less reliable far field depth)
			// we are setting nDepthMaxDistance to the extreme potential depth threshold
			nDepthMaxDistance = USHRT_MAX;

			// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
            hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
        }

        if (SUCCEEDED(hr)){

			hr = pDepthFrame->CopyFrameDataToArray(nBufferSize, pBuffer);

			if (SUCCEEDED(hr)) {
				for (UINT i = 0; i < nBufferSize; i++) {
					depthB = pBuffer[i];
					m.at<UINT8>(i) = LOWORD(depthB);
					//m.at<USHORT16>(i) = depthB;
				}
			
			}
			
        }

        SafeRelease(pFrameDescription);
    }
	else { imageCaptured = 0; }

    SafeRelease(pDepthFrame);	
}

HRESULT CDepthBasics::initializeDefaultSensor(){
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr)){
        return hr;
    }

    if (m_pKinectSensor){
        // Initialize the Kinect and get the depth reader
        IDepthFrameSource* pDepthFrameSource = NULL;

        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr)){
            hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
        }

        if (SUCCEEDED(hr)){
            hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
        }

        SafeRelease(pDepthFrameSource);
    }

    return hr;
}
