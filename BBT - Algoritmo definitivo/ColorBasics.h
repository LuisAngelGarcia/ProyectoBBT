/*********************************************************/
//SISTEMA DE ENSAYO DE CAJA Y CUBOS AUTOMATIZADO (ABBT)//
	    //Luis Ángel García Astudillo// 
  //Grado en Ingeniería en Tecnologías Industriales//
         //Universidad Carlos III de Madrid//
/********************************************************/

#pragma once

#include "resource.h"
#include "ImageRenderer.h"
#include <opencv2/opencv.hpp>
#include <Kinect.h>

using namespace cv;

class CColorBasics{

    static const int cColorWidth  = 1920;
    static const int cColorHeight = 1080;
	bool imageCaptured = 0;

public:

	CColorBasics(); //IMPORTANTE
    ~CColorBasics(); //IMPORTANTE
	int getWidth(){ return cColorWidth; }
	int getHeight(){ return cColorHeight; }
	bool getImageCaptured() { return imageCaptured; };
    
    double m_fFreq;
    IKinectSensor*m_pKinectSensor; // Current Kinect
    IColorFrameReader*m_pColorFrameReader; // Color reader
    ImageRenderer*m_pDrawColor; // Direct2D
    ID2D1Factory*m_pD2DFactory;
    RGBQUAD* m_pColorRGBX;

    void update(Mat &m); // Hace una foto de color y la devuelve al main por referencia
    HRESULT initializeDefaultSensor(); // Inicializa el sensor de color


};

