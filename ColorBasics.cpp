/*********************************************************/
//SISTEMA DE ENSAYO DE CAJA Y CUBOS AUTOMATIZADO (ABBT)//
	    //Luis Ángel García Astudillo// 
  //Grado en Ingeniería en Tecnologías Industriales//
         //Universidad Carlos III de Madrid//
/********************************************************/

#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "ColorBasics.h"
#include <opencv2/opencv.hpp>

using namespace cv;

CColorBasics::CColorBasics() :

    m_fFreq(0),
    m_pKinectSensor(NULL),
    m_pColorFrameReader(NULL),
    m_pColorRGBX(NULL)
{
    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf)){
        m_fFreq = double(qpf.QuadPart);
    }

    // create heap storage for color pixel data in RGBX format
    m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
}
  

CColorBasics::~CColorBasics(){
  

    SafeRelease(m_pColorFrameReader); // done with color frame reader

    if (m_pKinectSensor){ // close the Kinect Sensor

        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);
}


void CColorBasics::update(Mat &m){

    if (!m_pColorFrameReader){
        return;
    }

    IColorFrame* pColorFrame = NULL;
    HRESULT hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

    if (SUCCEEDED(hr)){

		imageCaptured = 1;
        INT64 nTime = 0;
        IFrameDescription* pFrameDescription = NULL;
        int nWidth = 0;
        int nHeight = 0;
        ColorImageFormat imageFormat = ColorImageFormat_None;
        UINT nBufferSize = 0;
        RGBQUAD *pBuffer = NULL;

        hr = pColorFrame->get_RelativeTime(&nTime);

        if (SUCCEEDED(hr)){
            hr = pColorFrame->get_FrameDescription(&pFrameDescription);
        }

        if (SUCCEEDED(hr)){
            hr = pFrameDescription->get_Width(&nWidth);
        }

        if (SUCCEEDED(hr)){
            hr = pFrameDescription->get_Height(&nHeight);
        }

        if (SUCCEEDED(hr)){
            hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
        }

        if (SUCCEEDED(hr)){
            if (imageFormat == ColorImageFormat_Bgra){

                hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&m.data));
            }
            else if (m_pColorRGBX){

                pBuffer = m_pColorRGBX;
                nBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
                hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(m.data), ColorImageFormat_Bgra);            
            }
            else{
                hr = E_FAIL;
            }
        }

        SafeRelease(pFrameDescription);
    }
	else { imageCaptured = 0; }

	SafeRelease(pColorFrame);
	
	
}


HRESULT CColorBasics::initializeDefaultSensor(){

    HRESULT hr;
    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr)){
        return hr;
    }

    if (m_pKinectSensor){
        // Initialize the Kinect and get the color reader
        IColorFrameSource* pColorFrameSource = NULL;

        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr)){
            hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
        }

        if (SUCCEEDED(hr)){
            hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
        }

        SafeRelease(pColorFrameSource);
    }
    return hr;
}
