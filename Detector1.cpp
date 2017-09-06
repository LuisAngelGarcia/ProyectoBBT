/*********************************************************/
//SISTEMA DE ENSAYO DE CAJA Y CUBOS AUTOMATIZADO (ABBT)//
	    //Luis Ángel García Astudillo// 
  //Grado en Ingeniería en Tecnologías Industriales//
         //Universidad Carlos III de Madrid//
/********************************************************/

#include "Detector.h"
#include <opencv2/opencv.hpp>
#include <Windows.h>
#include <Kinect.h>
#include <math.h>
#include <vector>
#include <algorithm>


#define FX 2.95
#define FXINV 0.34
#define FY 2.85
#define FYINV 0.35
//Valores de Hue para la segmentación por colores http://www.workwithcolor.com/pink-red-color-hue-range-01.htm
#define REDINF 340.0
#define REDSUP 21.9
#define GREENINF 85.0
#define GREENSUP 125.0
#define BLUEINF 195.0
#define BLUESUP 235.0
#define YELLOWINF 49.0
#define YELLOWSUP 70.0
#define BACKGROUNDINF 22.0
#define BACKGROUNDSUP 48.9
//Parámetros de ajuste
const float T1 = 0.75, T2 = 0.45; //T1 -> umbral de detecciones seguras, T2 -> umbral de detecciones menos probables
const char distanceBetweenBlocks = 21; //Distancia mínima entre dos detecciones para no ser descartadas (en X y en Y)
const char blurCoeff = 8, blurCoeff1 = 8;
//const char medianCoeff = 8;
const float ratio = 2; //ratio entre los parámetros del filtro Canny
const int lowCanny = 90; //Parámetro más pequeño del filtro Canny

using namespace std;
using namespace cv;

HRESULT Detector::initializeSensors() {
	HRESULT hrC = color.initializeDefaultSensor();
	HRESULT hrD = depth.initializeDefaultSensor();

	if (SUCCEEDED(hrC) && SUCCEEDED(hrD)) return hrC;
	else return -1;
};
void Detector::update(Mat &m, Mat &n) {
	color.update(m);
	depth.update(n);
};

void Detector::updateColor(Mat &m) {
	color.update(m);
};

void Detector::updateDepth(Mat &m) {
	depth.update(m);
};

void Detector::locateBoxes(Mat &m, Mat &n) {

	Mat rangeOutput;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	//Localizar la pantalla divisoria y su ROI
	///Threshold entre dos valores para localizar la pantalla
	
	inRange(m, 4,70, rangeOutput); //Altura de la pantalla divisoria
	//inRange(m, 740,780, rangeOutput); //Altura de la pantalla divisoria /*Alternativamente 769 - 835*/

	/*namedWindow("RangeDivisor", CV_WINDOW_AUTOSIZE);
	imshow("RangeDivisor", rangeOutput);
	waitKey(100000);
	destroyAllWindows();*/
	//imwrite("umbralDivisor.jpg", rangeOutput);

	//Buscar contornos en la imagen
	findContours(rangeOutput, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	//Encontrar los contornos rectangulares rotados cuyo centro se encuentra alrededor del centro de la imagen
	RotatedRect minRect;

	for (int i = 0; i < contours.size(); i++){
		minRect = minAreaRect(Mat(contours[i]));
		if ((minRect.center.x > 210) && (minRect.center.x < 300) && (minRect.center.y > 140) && (minRect.center.y < 285)) {//El divisor tiene que estar más o menos centrado
			if ((minRect.size.height > divisorRect.size.height)/*&&(minRect.size.width - divisorRect.size.width <20)*/) {//Comprobación de tamaño
				divisorRect = minRect;
			}
		}
	}
	
	handROI = m(divisorRect.boundingRect());
	handROImean = mean(handROI)[0];
	rectangle(m, divisorRect.boundingRect(), Scalar(255), 1, 8, 0);

	//Mostrar el rectángulo divisor
	Scalar color = Scalar(255, 255, 255);
	Point2f rect_points[4]; divisorRect.points(rect_points);
	for (int j = 0; j < 4; j++)
		line(m, rect_points[j], rect_points[(j + 1) % 4], color, 1, 8);

	namedWindow("ContoursDivisor", CV_WINDOW_AUTOSIZE);
	imshow("ContoursDivisor", m);
	waitKey(100000);
	destroyAllWindows();
	//imwrite("localizacionDivisor.jpg", m);

	//Localizar la caja y su ROI
	Mat rangeOutput1;
	vector<vector<Point> > contours1;
	vector<Vec4i> hierarchy1;

	//Threshold entre dos valores para localizar el borde de la caja. 
	inRange(m, 50, 140, rangeOutput1); //Altura del borde
	//inRange(m, 890, 950, rangeOutput1); //Altura del borde /*Alternativamente 815 - 905*/
	
	/*namedWindow("RangeBox", CV_WINDOW_AUTOSIZE);
	imshow("RangeBox", rangeOutput1);
	waitKey(100000);
	destroyAllWindows();*/
	//imwrite("umbralCajas.jpg", rangeOutput1);

	//Buscar contornos en la imagen
	findContours(rangeOutput1, contours1, hierarchy1, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	//Encontrar los contornos rectangulares cuyo centro se encuentra alrededor del centro de la imagen
	RotatedRect minRect1, temp;
	vector<vector<Point> > contours_poly(contours1.size());
	
	for (int i = 0; i < contours1.size(); i++){
		minRect1 = minAreaRect(Mat(contours1[i]));
		if ((minRect1.center.x > 210) && (minRect1.center.x < 300) && (minRect1.center.y > 140) && (minRect1.center.y < 285)) {//La caja tiene que estar más o menos centrado
			if (minRect1.size.width > temp.size.width) {//Comprobación de tamaño
				temp = minRect1;
			}
		}
	}
	boxRect = temp.boundingRect();
	rectangle(m, boxRect, Scalar(255), 1, 8, 0);
	

	//Dividir la caja en dos, determinar cuál es el compartimento a vigilar y quedarnos con el cuadrado adecuado
	Point p1, p2, tl, br;
	tl = boxRect.tl();
	br = boxRect.br();
	Point centroid1, centroid2;

	centroid1.x = tl.x + boxRect.width / 4;
	centroid1.y = tl.y + boxRect.height / 2;
	centroid2.x = tl.x + boxRect.width * 0.75;
	centroid2.y = tl.y + boxRect.height / 2;

	//Seleccionar dos ROI pequeñas alrededor de cada centroide y calcular la media de sus intensidades para averiguar qué compartimento está lleno.
	Mat rangeOutput2;
	inRange(m, 80, 200, rangeOutput2); //Altura de los cubos
	//inRange(m, 890, 930, rangeOutput2); //Altura de los cubos /*Alternativamente 845 - 965*/

	/*namedWindow("m", CV_WINDOW_AUTOSIZE);
	imshow("m", rangeOutput2);
	waitKey(100000);
	destroyAllWindows();*/
	//imwrite("umbralCubos.jpg", rangeOutput2);

	Mat leftROI = rangeOutput2(Rect(centroid1, Point (centroid1.x + 20, centroid1.y + 20)));
	Mat rightROI = rangeOutput2(Rect(centroid2, Point (centroid2.x + 20, centroid2.y + 20)));
	
	if ((mean (leftROI)[0]>mean (rightROI)[0])/*&&((m.at<UINT8>(centroid1) - m.at<UINT8>(centroid2))<255)*/){
		emptyBoxFlag=0; //lado derecho del paciente

		p1.x = tl.x + boxRect.width / 2;
		p1.y = tl.y;
		p2.x = tl.x + boxRect.width / 2;
		p2.y = tl.y + boxRect.height;
		emptyBoxRect = Rect(p1, br);
		boxRect = Rect(tl, p2);
		emptyDepth = m.at<uchar>(centroid2);
	}
	else {
		emptyBoxFlag=1;//lado izquierdo del paciente
		
		p1.x = tl.x + boxRect.width / 2;
		p1.y = tl.y + boxRect.height;
		p2.x = tl.x + boxRect.width / 2;
		p2.y = tl.y;
		emptyBoxRect = Rect(tl, p1);
		boxRect = Rect(p2, br);
		emptyDepth = m.at<uchar>(centroid1);
	}
	//Determinar frontera entre amarillo y fondo
	Point center;
	Mat colorHSV(1080, 1920, CV_32FC4);
	Mat colorH;
	Mat planes[4];
	Vec3b a;
	float values[5];

	//Posicionar el centroide en la imagen de color
	if (emptyBoxFlag == 0) center = Point((centroid2.x * FX) + 255, (centroid2.y * FY) - 62);
	else center = Point((centroid1.x * FX) + 218, (centroid1.y * FY) - 62);

	//Lectura de los colores del fondo
	cvtColor(n, colorHSV, CV_BGR2HSV);
	
	a = colorHSV.at<Vec3b>(center);
	values[0] = a[0];
	a = colorHSV.at<Vec3b>(Point(center.x - 150, center.y - 150));
	values[1] = a[0];
	a = colorHSV.at<Vec3b>(Point(center.x - 150, center.y + 150));
	values[2] = a[0];
	a = colorHSV.at<Vec3b>(Point(center.x + 150, center.y - 150));
	values[3] = a[0];
	a = colorHSV.at<Vec3b>(Point(center.x + 150, center.y + 150));
	values[4] = a[0];
	//Hallar valor máximo
	float maxval;
	maxval = (*max_element(values, values + 4)) * 2;
	if ((maxval < 45) && (maxval < 55)) {
		BACKGROUNDSUP = maxval - 1;
		YELLOWINF = BACKGROUNDSUP + 0.5;
	}
	//cout << maxval;
	//Mostrar compartimento
	
	
	rectangle(m, emptyBoxRect.tl(), emptyBoxRect.br(), color, 2, 8, 0);
	namedWindow("Box", CV_WINDOW_AUTOSIZE);
	imshow("Box", m);
	waitKey(100000);
	destroyAllWindows();
	imwrite("localizacionCajas.jpg", m);

};


void Detector::correctBoxesPosition(Mat &m) {

	Mat rangeOutput;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	//Localizar la pantalla divisoria y su ROI
	///Threshold entre dos valores para localizar la pantalla
	
	inRange(m, 4,70, rangeOutput); //Altura de la pantalla divisoria
	//inRange(m, 740,780, rangeOutput); //Altura de la pantalla divisoria en el rango de 16bits /*Alternativamente 769 - 835*/

	//Buscar contornos en la imagen
	findContours(rangeOutput, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	//Encontrar los contornos rectangulares rotados cuyo centro se encuentra alrededor del centro de la imagen
	RotatedRect minRect;

	for (int i = 0; i < contours.size(); i++){
		minRect = minAreaRect(Mat(contours[i]));
		if ((minRect.center.x > 210) && (minRect.center.x < 300) && (minRect.center.y > 140) && (minRect.center.y < 285)) {//El divisor tiene que estar más o menos centrado
			if ((minRect.size.height > divisorRect.size.height)/*&&(minRect.size.width - divisorRect.size.width <20)*/) {//Comprobación de tamaño
				divisorRect = minRect;
			}
		}
	}
	
	handROI = m(divisorRect.boundingRect());
	handROImean = mean(handROI)[0];
	rectangle(m, divisorRect.boundingRect(), Scalar(255), 1, 8, 0);

	//Localizar la caja y su ROI
	Mat rangeOutput1;
	vector<vector<Point> > contours1;
	vector<Vec4i> hierarchy1;

	//Threshold entre dos valores para localizar el borde de la caja. 
	inRange(m, 50, 140, rangeOutput1); //Altura del borde
	//inRange(m, 890, 950, rangeOutput1); //Altura del borde en el rango de 16bits /*Alternativamente 815 - 905*/

	//Buscar contornos en la imagen
	findContours(rangeOutput1, contours1, hierarchy1, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	//Encontrar los contornos rectangulares cuyo centro se encuentra alrededor del centro de la imagen
	RotatedRect minRect1, temp;
	vector<vector<Point> > contours_poly(contours1.size());
	

	for (int i = 0; i < contours1.size(); i++){
		minRect1 = minAreaRect(Mat(contours1[i]));
		if ((minRect1.center.x > 210) && (minRect1.center.x < 300) && (minRect1.center.y > 140) && (minRect1.center.y < 285)) {//La caja tiene que estar más o menos centrado
			if (minRect1.size.width > temp.size.width) {//Comprobación de tamaño
				temp = minRect1;
			}
		}
	}
	boxRect = temp.boundingRect();
	rectangle(m, boxRect, Scalar(255), 1, 8, 0); 
	

	//Dividir la caja en dos, determinar cuál es el compartimento a vigilar y quedarnos con el cuadrado adecuado
	Point p1, p2, tl, br;
	tl = boxRect.tl();
	br = boxRect.br();
	Point centroid1, centroid2;

	centroid1.x = tl.x + boxRect.width / 4;
	centroid1.y = tl.y + boxRect.height / 2;
	centroid2.x = tl.x + boxRect.width * 0.75;
	centroid2.y = tl.y + boxRect.height / 2;

	//Seleccionar dos ROI pequeñas alrededor de cada centroide y calcular la media de sus intensidades para averiguar qué compartimento está lleno.
	Mat rangeOutput2;
	inRange(m, 80, 200, rangeOutput2); //Altura de los cubos
	//inRange(m, 890, 930, rangeOutput2); //Altura de los cubos en 16 bits /*Alternativamente 845 - 965*/


	Mat leftROI = rangeOutput2(Rect(Point (centroid1.x - 20, centroid1.y - 20), Point (centroid1.x + 20, centroid1.y + 20)));
	Mat rightROI = rangeOutput2(Rect(Point (centroid2.x - 20, centroid2.y - 20), Point (centroid2.x + 20, centroid2.y + 20)));
	
	if ((mean (leftROI)[0]>mean (rightROI)[0])/*&&((m.at<UINT8>(centroid1) - m.at<UINT8>(centroid2))<255)*/){ //Estaba >
		emptyBoxFlag=0; //lado derecho del paciente

		p1.x = tl.x + boxRect.width / 2;
		p1.y = tl.y;
		p2.x = tl.x + boxRect.width / 2;
		p2.y = tl.y + boxRect.height;
		emptyBoxRect = Rect(p1, br);
		boxRect = Rect(tl, p2);
		emptyDepth = m.at<uchar>(centroid2);
	}
	else {
		emptyBoxFlag=1;//lado izquierdo del paciente
		
		p1.x = tl.x + boxRect.width / 2;
		p1.y = tl.y + boxRect.height;
		p2.x = tl.x + boxRect.width / 2;
		p2.y = tl.y;
		emptyBoxRect = Rect(tl, p1);
		boxRect = Rect(p2, br);
		emptyDepth = m.at<uchar>(centroid1);
	}

};


void Detector::detectHand(Mat &m, bool a){
	Mat temp, dst;
	dst= Scalar::all(0);
	temp = m(divisorRect.boundingRect());
	float tempMean = mean(temp)[0];
	static int cont=0;
	/*
	imshow("TEMP", temp);
	waitKey(100000);
	destroyAllWindows();
	imshow("ROI", handROI);
	waitKey(100000);
	destroyAllWindows();
	*/
	
	if (a==0){
		handDetected = 0;
		if ((abs(tempMean - handROImean)) > 50) {
		cont++;
		if (cont > 5){
			
			cont = 0;
			Sleep(40);
			handDetected = 1;
			//imwrite("tempROIIn.jpg", temp);
			//imwrite("handROI.jpg", handROI);
		}
	}

	}
	else{
		handDetected = 1;
		if ((abs(tempMean - handROImean)) < 50) {
		cont++;
		if (cont > 5){
			
			cont = 0;
			Sleep(40);
			handDetected = 0;
			//imwrite("tempROIOut.jpg", temp);
			//imwrite("handROI.jpg", handROI);
		}
	}

	}
	
};


void Detector::setCoefs(Mat &m) {
	Mat bufferColor(424, 400, CV_8UC4);
	Size dsize = Size(424, 400);
	Point p = boxRect.tl();
	Rect ROIRect;

	resize(m, bufferColor, dsize, 0, 0, CV_INTER_AREA);
	Mat planes[4], ROI;
	if (emptyBoxFlag == 0) {
		ROIRect = Rect(p.x + 30, p.y + 30, 50, 50);
	}
	else{ 
		ROIRect = Rect(p.x - 5, p.y - 5, 50, 50); 
	}
	
	ROI = bufferColor(ROIRect);
/*
	imshow("B", ROI);
	waitKey(100000);
	destroyAllWindows();
	*/
	split(ROI, planes);
	double min, max;

	minMaxLoc(planes[0], &min, &max);
	coefB = (max) / 255;
	//cout << "B: "<< coefB << endl;

	minMaxLoc(planes[1], &min, &max);
	coefG = (max) / 255;
	//cout << "G: " << coefG << endl;

	minMaxLoc(planes[2], &min, &max);
	coefR = (max) / 255;
	//cout << "R: "<<  coefR << endl;
};

void Detector::whitePatchTransf (Mat &m){

Mat planes[4];
split(m, planes);
/*
imshow("B", planes[0]);
waitKey(100000);
destroyAllWindows();
imshow("G", planes[1]);
waitKey(100000);
destroyAllWindows();
imshow("R", planes[2]);
waitKey(100000);
destroyAllWindows();
*/
planes[0] = planes[0] * coefB;
planes[1] = planes[1] * coefG;
planes[2] = planes[2] * coefR;

merge(planes, 4, m);

};

void Detector::templateMatching(Mat &m, Mat &t, vector<Point> &a, vector<Point> &b) {
	/*imshow("COLOR", m);
	waitKey(10000);
	destroyAllWindows();
	imshow("COLOR", t);
	waitKey(1000);
	destroyAllWindows();*/
	double minval, maxval;
	Point minloc, maxloc;

	Mat res_32f(m.rows - t.rows + 1, m.cols - t.cols + 1, CV_32FC1);
	matchTemplate(m, t, res_32f, CV_TM_CCOEFF_NORMED);

	Mat res, resThres1, resThres2;
	res_32f.convertTo(res, CV_8U, 255.0);
	/*imshow("result", res);
	waitKey(100000);
	destroyAllWindows();*/
	//imwrite("matchings.jpg", res);

	minMaxLoc(res, &minval, &maxval, &minloc, &maxloc);//Valor máximo de coincidencia
	//cout << maxval << endl;
	inRange(res, maxval * T1, maxval, resThres1); //Coincidencias exactas
	inRange(res, maxval * T2, maxval * T1, resThres2); //Coincidencias menos precisas
	/*
	imshow("result_thresh1", resThres1);
	waitKey(100000);
	destroyAllWindows();*/
	//imwrite("localizacionesExactas.jpg", resThres1);
	/*imshow("result_thresh2", resThres2);
	waitKey(100000);
	destroyAllWindows();*/
	//imwrite("localizacionesInexactas.jpg", resThres2);
	bool fin = 0;
	
	do{
		
		minMaxLoc(resThres1, &minval, &maxval, &minloc, &maxloc);

		if (maxval > 0)
		{
			floodFill(resThres1, maxloc, 0); //borrar el punto de la imagen
			a.push_back(maxloc);//guardar la localización
		}
		else
			fin = 1;
	} while (fin==0);
	fin = 0;
	do{
		
		minMaxLoc(resThres2, &minval, &maxval, &minloc, &maxloc);

		if (maxval > 0)
		{
			
			floodFill(resThres2, maxloc, 0); //borrar el punto de la imagen
			b.push_back(maxloc);//guardar la localización
		}
		else
			fin = 1;
	} while (fin==0);

	//imshow("final", m);
	//waitKey(0);

};

void Detector::colorClassifier(Mat &roi, vector<Point> &src, vector<Block> &dst) {
	//Comprobación del color del posible cubo del vector de detecciones seguras usando segmentación por HSV
	Vec4b color, color1, color2, color3, color4;
	float hue, sat;
	float r, g, b, max, min;
	float mR, mG, mB;
	int size = src.size();

	for (int j = 0; j < size; j++) {
		color = roi.at<Vec4b>(Point(src[j].x + 22, src[j].y + 22)); //Coger el color del centro del cubo
		color1 = roi.at<Vec4b>(Point(src[j].x + 22, src[j].y + 29)); //Coger el color justo encima del centro del cubo
		color2 = roi.at<Vec4b>(Point(src[j].x + 22, src[j].y + 15)); //Coger el color justo debajo del centro del cubo
		color3 = roi.at<Vec4b>(Point(src[j].x + 29, src[j].y + 22)); //Coger el color justo a la derecha del centro del cubo
		color4 = roi.at<Vec4b>(Point(src[j].x + 15, src[j].y + 22)); //Coger el color justo a la izquierda del centro del cubo

		///Cálculo de la media de los píxeles adyacentes al centro
		mR = (color1.val[2] + color2.val[2] + color3.val[2] + color4.val[2]) / 4;
		mG = (color1.val[1] + color2.val[1] + color3.val[1] + color4.val[1]) / 4;
		mB = (color1.val[0] + color2.val[0] + color3.val[0] + color4.val[0]) / 4;

		///Cálculo del valor de Hue del píxel seleccionado
		r = color.val[2];
		g = color.val[1];
		b = color.val[0];
		if ((r > g) && (g >= b)) {
			max = r;
			min = b;
			hue = 60 * ((g - b) / (max - min));
		}
		else if ((r > g) && (g < b)) {
			max = r;
			min = g;
			hue = (60 * ((g - b) / (max - min))) + 360;
		}
		else if ((g > r) && (g > b)) {
			max = g;
			min = r;
			if (r>b) min = b;
			hue = (60 * ((b - r) / (max - min))) + 120;
		}
		else if ((b > r) && (b > g)) {
			max = b;
			min = r;
			if (r > g) min = g;
			hue = (60 * ((r - g) / (max - min))) + 240;
		}
		else hue = 39;

		sat = max - min;

		//Discriminación por colores y adición al vector de cubos, con color y posición
		Block block;
		if((abs(r - mR) < 5)&&((abs(g - mG) < 5))&&((abs(b - mB) < 5))){
			if (!((hue>BACKGROUNDINF) && (hue<BACKGROUNDSUP) && (sat<50))) { //No es fondo
				if ((hue>REDINF) || (hue<REDSUP)) { //Rojo
					block.setColor(1);
					block.setPos(src[j]);
					block.setSize(22);
					dst.push_back(block);
				}
				else if ((hue>GREENINF) && (hue<GREENSUP)) { //Verde
					block.setColor(2);
					block.setPos(src[j]);
					block.setSize(22);
					dst.push_back(block);
				}
				else if ((hue>BLUEINF) && (hue<BLUESUP)) { //Azul
					block.setColor(3);
					block.setPos(src[j]);
					block.setSize(22);
					dst.push_back(block);
				}

				else if (((hue>YELLOWINF) && (hue<YELLOWSUP)) || ((hue>BACKGROUNDINF) && (hue<BACKGROUNDSUP) && (sat>50))) { //Amarillo
					block.setColor(4);
					block.setPos(src[j]);
					block.setSize(22);
					dst.push_back(block);
				}
				else { //Color no determinado
					block.setColor(0);
					block.setPos(src[j]);
					block.setSize(22);
					dst.push_back(block);
				}
			}
		}
		

	}

}

void Detector::detection(Mat & m, Mat &n) {
	//Agrandar y recolocar el rectángulo del compartimento vacío

	/*Rect emptyBoxColor = emptyBoxRect;
	Point topLeft = emptyBoxColor.tl(); 
	Point offset(( topLeft.x * FX) + 140, (topLeft.y * FY) - 165);
	Point offset1((topLeft.x * FX) - 280, (topLeft.y * FY) - 165);
	emptyBoxColor.height = emptyBoxColor.height * FY;
	emptyBoxColor.width = emptyBoxColor.width * FX;
	if (emptyBoxFlag == 1) emptyBoxColor = emptyBoxColor + offset;
	if (emptyBoxFlag == 0)emptyBoxColor = emptyBoxColor + offset1;*/
	
	Point topLeft = emptyBoxRect.tl();
	Rect emptyBoxDepth = Rect(topLeft.x * FX, topLeft.y * FY, emptyBoxRect.width, emptyBoxRect.width);
	emptyBoxDepth.height = emptyBoxDepth.height * FY;
	emptyBoxDepth.width = emptyBoxDepth.width * FX;

	Rect emptyBoxColor;
	if (emptyBoxFlag == 1) emptyBoxColor = Rect((topLeft.x * FX) + 255, (topLeft.y * FY) - 62, emptyBoxRect.width, emptyBoxRect.width);
	else emptyBoxColor = Rect((topLeft.x * FX) + 218, (topLeft.y * FY) - 62, emptyBoxRect.width, emptyBoxRect.width);
	emptyBoxColor.height = emptyBoxColor.height * FY;
	emptyBoxColor.width = emptyBoxColor.width * FX;

	rectangle(m, emptyBoxColor.tl(), emptyBoxColor.br(), (0,0,255), 2, 8, 0);
	/*namedWindow("Color Box", CV_WINDOW_AUTOSIZE);
	imshow("Color Box", m);
	waitKey(100000);
	destroyAllWindows();*/
	//imwrite("colorRect.jpg", m);

	rectangle(n, emptyBoxDepth.tl(), emptyBoxDepth.br(), (255, 255, 255), 1, 8, 0);
	/*namedWindow("Depth Box", CV_WINDOW_AUTOSIZE);
	imshow("Depth Box", n);
	waitKey(100000);
	destroyAllWindows();*/
	//imwrite("depthRect.jpg", n);

	//Obtener las ROI de la imagen de color y de la de profundidad
	Mat colorROI, colorBoxROI, depthROI;
	colorROI = m(emptyBoxColor);
	colorBoxROI = colorROI.clone();
	depthROI = n(emptyBoxDepth);
	if (first == 1){
		emptyBoxROI = depthROI.clone();
		first = 0;
	}
	
	/*imshow("COLORROI", colorROI);
	waitKey(10000);
	destroyAllWindows();*/
	//imwrite("colorROI.jpg", colorROI);

	/*imshow("PROFROI", depthROI);
	waitKey(10000);
	destroyAllWindows();*/
	//imwrite("depthROI.jpg", depthROI);

	//Filtro Canny
	Mat gref, blurred, colorROIgrey;
	cvtColor(colorROI, colorROIgrey, CV_BGR2GRAY);

	/*imshow("COLOR", colorROIgrey);
	waitKey(100000);
	destroyAllWindows();*/
	//imwrite("colorROIgrey.jpg", colorROIgrey);

	bilateralFilter(colorROIgrey, blurred, blurCoeff,blurCoeff * 2, blurCoeff/2 );
	//GaussianBlur(colorROI, blurred, Size(3, 3), 0, 0);
	//blur(colorROI, blurred, Size(3, 3));
	//medianBlur(colorROI, blurred, medianCoeff);
	/*imshow("Blurred", blurred);
	waitKey(100000);
	destroyAllWindows();*/
	//imwrite("blurred.jpg", blurred);
	/*
	imshow("COLOR", colorROI);
	waitKey(1000);
	destroyAllWindows();
	imshow("COLOR", zero);
	waitKey(1000);
	destroyAllWindows();*/
	//imwrite("zeroC.jpg", zero);
	/*imshow("COLOR", thirty);
	waitKey(1000);
	destroyAllWindows();*/
	//imwrite("thirtyC.jpg", thirty);
	/*imshow("COLOR", fortyFive);
	waitKey(1000);
	destroyAllWindows();*/
	//imwrite("fortyFiveC.jpg", fortyFive);
	/*imshow("COLOR", sixty);
	waitKey(1000);
	destroyAllWindows();*/
	//imwrite("sixtyC.jpg", sixty);
	
	Canny(blurred, gref, lowCanny, lowCanny * ratio);
	
	/*imshow("file", gref);
	waitKey(100000);
	destroyAllWindows();*/
	//imwrite("canny.jpg", gref);
	
	//Hacer los template matching
	vector <Point> highProbPoints, lowProbPoints;
	templateMatching(gref, zero, highProbPoints, lowProbPoints);
	templateMatching(gref, thirty, highProbPoints, lowProbPoints);
	templateMatching(gref, fortyFive, highProbPoints, lowProbPoints);
	templateMatching(gref, sixty, highProbPoints, lowProbPoints);

	//Descartar los candidatos que contengan color de fondo
	vector <Block> highProbBlocks, lowProbBlocks;
	colorClassifier(colorROI, highProbPoints, highProbBlocks);
	colorClassifier(colorROI, lowProbPoints, lowProbBlocks);

	//Eliminar posiciones redundantes
	vector <Block> detectedBlocks, probablyBlocks;
	///Ordenar primero los vectores para facilitar su comparación
	//std::sort(highProbPoints.begin(), highProbPoints.end(), [](Point const &l, Point const &r) { return l.x < r.x; });
	//std::sort(lowProbPoints.begin(), lowProbPoints.end(), [](Point const &l, Point const &r) { return l.x < r.x; });

	///Bucle para el primer vector
	bool coincidence = 0;
	int size, size1 = highProbBlocks.size();
	detectedBlocks.push_back(highProbBlocks[0]);
	for (int i = 1; i < size1; i++) {
		
			size = detectedBlocks.size();
			for (int j = 0; j < size; j++) {
				if ((abs(highProbBlocks[i].getPos().x - detectedBlocks[j].getPos().x) < distanceBetweenBlocks) && (abs(highProbBlocks[i].getPos().y - detectedBlocks[j].getPos().y) < distanceBetweenBlocks)) {
					coincidence = 1;
				}
			}
			if (coincidence == 0) {
				detectedBlocks.push_back(highProbBlocks[i]);
			}	
			coincidence = 0;
	}
	///Bucle para el segundo vector
	coincidence = 0;
	size1 = lowProbBlocks.size();
	probablyBlocks.push_back(lowProbBlocks[0]);
	for (int i = 1; i < size1; i++) {

			size = probablyBlocks.size();
			for (int j = 0; j < size; j++) {
				if ((abs(lowProbBlocks[i].getPos().x - probablyBlocks[j].getPos().x) < distanceBetweenBlocks) && (abs(lowProbBlocks[i].getPos().y - probablyBlocks[j].getPos().y) < distanceBetweenBlocks)) {
					coincidence = 1;
				}
			}
			if (coincidence == 0) {
				probablyBlocks.push_back(lowProbBlocks[i]);
			}
			coincidence = 0;
	}

	///Comprobar la validez de los puntos del segundo vector
	coincidence = 0;
	size1 = probablyBlocks.size();
	for (int i = 0; i < size1; i++) {

		size = detectedBlocks.size();
		for (int j = 0; j < size; j++) {
			if ((abs(detectedBlocks[j].getPos().x - probablyBlocks[i].getPos().x) < distanceBetweenBlocks) && (abs(detectedBlocks[j].getPos().y - probablyBlocks[i].getPos().y) < distanceBetweenBlocks)) {
				coincidence = 1;
			}
		}
		if (coincidence == 0) {//El bloque se encontraba en una posición rotada ligeramente respecto a las de las plantillas y no ha producido una detección perfecta. Hay que añadirlo
			detectedBlocks.push_back(probablyBlocks[i]);
		}
		coincidence = 0;
	}

	//Comprobación de la profundidad de las detecciones -> adición de cubos apilados
	Point translatedPoint;
	vector <Block> blocksVector;
	Block block;
	redBlocks = 0; 
	greenBlocks = 0;
	blueBlocks = 0; 
	yellowBlocks = 0; 
	blackBlocks = 0;
	size = detectedBlocks.size();
	for (int j = 0; j < size; j++) {
		blocksVector.push_back(detectedBlocks[j]);
		translatedPoint.x = detectedBlocks[j].getPos().x + 21 + 22 /** FXINV*/;//La primera cifra es la interpolación de una imagen a otra. La segunda para colocarnos en el centro del cubo
		translatedPoint.y = detectedBlocks[j].getPos().y + 22 /** FYINV*/;
		int height = depthROI.at<uchar>(translatedPoint);
		emptyDepth = emptyBoxROI.at<uchar>(translatedPoint);
		if (emptyDepth - height > 50) {//Pila de al menos 2 bloques -> Añadimos un bloque negro 
			block.setColor(0);
			block.setPos(detectedBlocks[j].getPos());
			block.setSize(24);
			blocksVector.push_back(block);
			blackBlocks++;
		}
		if (emptyDepth - height > 70) {//Pila de al menos 3 bloques -> Añadimos otro bloque negro
			block.setColor(0);
			block.setPos(detectedBlocks[j].getPos());
			block.setSize(26);
			blocksVector.push_back(block);
			blackBlocks++;
		}
	}	
	

	//Representación en la imagen
	int tempColor, rectSize;
	Scalar drawingColor;
	Point blockPosition;
	for (int i = 0; i < blocksVector.size(); i++) {
		tempColor = blocksVector[i].getColor();
		switch (tempColor){
			
			case 1:
				drawingColor = Scalar(0,0,255);
				redBlocks++;
				break;
			case 2:
				drawingColor = Scalar(0,255,0);
				greenBlocks++;
				break;
			case 3:
				drawingColor = Scalar(255,0,0);
				blueBlocks++;
				break;
			case 4:
				drawingColor = Scalar(0,255,255);
				yellowBlocks++;
				break;
			default:
				drawingColor = Scalar(0,0,0);
				blackBlocks++;
				break;
		}
		blockPosition = blocksVector[i].getPos();
		rectSize = 2 * blocksVector[i].getSize();
		rectangle(colorBoxROI, blockPosition, Point(blockPosition.x + rectSize, blockPosition.y + rectSize), drawingColor, 1);
	}
		
	/*imshow("Cubos detectados", colorBoxROI);
	waitKey(10000);
	destroyAllWindows();*/
	//imwrite("cubosDetectados.jpg", colorBoxROI);
	finalResult = colorBoxROI.clone();
	countedBlocks = blocksVector.size();
	
	cout<<"Se han detectado "<< countedBlocks<<" cubos ("<<redBlocks<<" cubos rojos, "<<greenBlocks<<" cubos verdes, "<<blueBlocks<<" cubos azules, "<<yellowBlocks<<" cubos amarillos y "<<blackBlocks<<" cubos de un color sin determinar"<<") y "<<handDetections<< " pasos de la mano."<<endl;
};

void Detector::setZero(Mat &m) { 
	Mat blurred;
	zero = m.clone(); 
	cvtColor(zero, zero, CV_BGR2GRAY);
	bilateralFilter(zero, blurred, blurCoeff1, blurCoeff1 * 2, blurCoeff1 / 2);
	Canny(blurred, zero, lowCanny, lowCanny * 2);

};

void Detector::setThirty(Mat &m) {
	Mat blurred;
	thirty = m.clone(); 
	cvtColor(thirty, thirty, CV_BGR2GRAY);
	bilateralFilter(thirty, blurred, blurCoeff1, blurCoeff1 * 2, blurCoeff1 / 2);
	Canny(blurred, thirty, lowCanny, lowCanny * 2);
	
};

void Detector::setFortyFive(Mat &m) { 
	Mat blurred;
	fortyFive = m.clone(); 
	cvtColor(fortyFive, fortyFive, CV_BGR2GRAY);
	bilateralFilter(fortyFive, blurred, blurCoeff1, blurCoeff1 * 2, blurCoeff1 / 2);
	Canny(fortyFive, fortyFive, lowCanny, lowCanny * 2);
	
};

void Detector::setSixty(Mat &m) { 
	Mat blurred;
	sixty = m.clone(); 
	cvtColor(sixty, sixty, CV_BGR2GRAY);
	bilateralFilter(sixty, blurred, blurCoeff1, blurCoeff1 * 2, blurCoeff1 / 2);
	Canny(blurred, sixty, lowCanny, lowCanny * 2);
};










