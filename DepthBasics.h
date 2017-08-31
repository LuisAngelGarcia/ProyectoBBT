//------------------------------------------------------------------------------
// <copyright file="DepthBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "resource.h"
#include "ImageRenderer.h"
#include <opencv2/opencv.hpp>
#include <Kinect.h>

using namespace cv;
using namespace std;

class CDepthBasics{

    //Parámetros de la imagen
	const int cDepthWidth = 512;
	const int cDepthHeight = 424;
	bool imageCaptured = 0;

public:

    CDepthBasics(); //IMPORTANTE
    ~CDepthBasics(); //IMPORTANTE
	int getWidth() { return cDepthWidth; };
	int getHeight() { return cDepthHeight; };
	bool getImageCaptured() { return imageCaptured; };

    
    double m_fFreq;
    IKinectSensor *m_pKinectSensor; // Current Kinect
    IDepthFrameReader *m_pDepthFrameReader; // Depth reader

    void update(Mat &m); // Hace una foto y la devuelve al main por referencia

    HRESULT initializeDefaultSensor(); // Inicializa el sensor de profundidad

};

