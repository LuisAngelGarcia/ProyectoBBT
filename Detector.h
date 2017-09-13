/*********************************************************/
//SISTEMA DE ENSAYO DE CAJA Y CUBOS AUTOMATIZADO (ABBT)//
	    //Luis Ángel García Astudillo// 
  //Grado en Ingeniería en Tecnologías Industriales//
         //Universidad Carlos III de Madrid//
/********************************************************/

#include "ColorBasics.h"
#include "DepthBasics.h"
#include <opencv2/opencv.hpp>
#include <Windows.h>
#include <Kinect.h>
#include <vector>
#include "Block.h"

using namespace std;
using namespace cv;

class Detector {
	//Streams
	CColorBasics color;
	CDepthBasics depth;

	//Plantillas
	Mat zero, thirty, fortyFive, sixty;

	//Constantes
	float coefR, coefG, coefB; //Variables para el White-patch
	const float coef = 0.75;
	int emptyDepth;
	float handROImean;

	//Variables de estado
	bool handDetected;
	bool emptyBoxFlag=0; //0 -> Derecho, 1 -> Izquierdo (desde el punto de vista del paciente)
	bool first = 1;
	bool end = 0;
	bool ambientLight = 1;

	//ROIs
	RotatedRect divisorRect;
	Rect boxRect;
	Rect emptyBoxRect;
	Mat	handROI;
	Mat finalResult;
	Mat emptyBoxROI;

	//Contadores
	int handDetections = 0;
	int countedBlocks = 0;
	int redBlocks;
	int greenBlocks;
	int blueBlocks;
	int yellowBlocks;
	int blackBlocks;

public:
	//Constructores y destructores
	Detector() {};
	~Detector(){};

	//Getters
	int getColorHeight() { return color.getHeight(); };
	int getColorWidth() { return color.getWidth(); };
	int getDepthHeight() { return depth.getHeight(); };
	int getDepthWidth() { return depth.getWidth(); };
	bool getHandDetected() { return handDetected; };
	int getHandDetections() { return handDetections; };
	bool getColorCaptured() { return color.getImageCaptured(); };
	bool getDepthCaptured() { return depth.getImageCaptured(); };
	Rect getEmptyBox() { return emptyBoxRect; };
	Rect getDivisor() { return divisorRect.boundingRect(); };
	bool getHandTested(){return emptyBoxFlag; };
	int getCountedBlocks(){return countedBlocks; };
	Mat getFinalResult() { return finalResult; };
	int getRed(){return redBlocks;}
	int getGreen(){return greenBlocks;}
	int getBlue(){return blueBlocks;}
	int getYellow(){return yellowBlocks;}
	int getBlack(){return blackBlocks;}
	bool getLight(){ return ambientLight; }

	//Setters
	void increaseHandDetected() { handDetections++; };
	void setZero(Mat &m); //Recibe la plantilla de archivo, hace el tratamiento y la almacena
	void setThirty(Mat &m); //Recibe la plantilla de archivo, hace el tratamiento y la almacena
	void setFortyFive(Mat &m); //Recibe la plantilla de archivo, hace el tratamiento y la almacena
	void setSixty(Mat &m); //Recibe la plantilla de archivo, hace el tratamiento y la almacena
	void setEnd(){end = 1;}
	
	////Métodos////
	//Inicializaci�n
	HRESULT initializeSensors();

	//Adquisici�n de im�genes
	void update(Mat &m, Mat &n); //coge imágenes de ambos canales
	void updateColor(Mat &m); //coge una imagen en color
	void updateDepth(Mat &m); //coge una imagen de profundidad

	//Cálculos iniciales
	void locateBoxes(Mat &m, Mat &n); //Posicionamiento inicial de la caja y la pantalla
	void correctBoxesPosition(Mat &m); //Reposicionamiento de la caja y la pantalla si el paciente las mueve
	void setCoefs(Mat &m); //Fija los coeficientes para el white-patch (sin uso actual)

	//Detectar paso de la mano
	void detectHand(Mat &m, bool a); 

	//Transformaciones de la imagen
	void whitePatchTransf (Mat &m); //Transformación White-patch para disminuir la influencia de la luz ambiental (sin uso actual)
	void darkenImage(Mat &m); //Transformación para oscurecer la imagen si la luz es demasiado directa (produce fallos en la detección de cubos amarillos)

	//�lgoritmo de detecci�n
	void detection(Mat &m, Mat &n); //Método principal de la detección. Se encarga de la extracción de características y candidatos y de la clasificación en cascada
	void templateMatching(Mat &m, Mat &t, vector<Point> &a, vector<Point> &b); //Recibe la imagen, realiza el template matching y clasifica los candidatos según su probabilidad
	void colorClassifier(Mat &roi, vector<Point> &src, vector<Block> &dst);  //Analiza los candidatos y los clasifica en función de su color. Descarta los que contienen fondo.
	

};











