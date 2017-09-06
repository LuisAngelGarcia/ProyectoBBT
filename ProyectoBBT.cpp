/*********************************************************/
//SISTEMA DE ENSAYO DE CAJA Y CUBOS AUTOMATIZADO (ABBT)//
	    //Luis Ángel García Astudillo// 
  //Grado en Ingeniería en Tecnologías Industriales//
         //Universidad Carlos III de Madrid//
/********************************************************/

#ifdef _WIN32
#define _CRT_SECURE_NO_WARNINGS
#endif


#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "ColorBasics.h"
#include "DepthBasics.h"
#include "Detector.h"
#include <opencv2/opencv.hpp>
#include <Windows.h>
#include <Kinect.h>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>

#define TESTTIME 60 //Duración de la prueba

using std::cout;
using std::cin;
using std::getline;
using namespace cv;

Mat resizedDepth;
int main(void) {
	//Definición de caracteres especiales para su impresión por consola
	char v = 160; //a con tilde
	char w = 130; //e con tilde
	char x = 161; //i con tilde
	char y = 162; //o con tilde
	char z = 163; //u con tilde
	char zz = 168; //cierre interrogación
	char ac = 174; //abrir comillas
	char cc = 175; //cerrar comillas

	Detector detector;

	Mat bufferColorMat(detector.getColorHeight(), detector.getColorWidth(), CV_8UC4);
	Mat bufferColorMat2(detector.getColorHeight(), detector.getColorWidth(), CV_8UC4);
	Mat bufferDepthMat(detector.getDepthHeight(), detector.getDepthWidth(), CV_8UC1);
	Mat bufferDepthMat2(detector.getDepthHeight(), detector.getDepthWidth(), CV_8UC1);
	Mat bufferDepthMat3(detector.getDepthHeight(), detector.getDepthWidth(), CV_8UC1);

	//Mat bufferDepthMat(detector.getDepthHeight(), detector.getDepthWidth(), CV_16UC1);
	//Mat bufferDepthMat2(detector.getDepthHeight(), detector.getDepthWidth(), CV_16UC1);
	//Mat bufferDepthMat3(detector.getDepthHeight(), detector.getDepthWidth(), CV_16UC1);

	HRESULT hr = detector.initializeSensors();
	if (FAILED(hr)) return 0;

	//Cargar los archivos de las plantillas de cubos
	
	Mat t1 = imread("zero.jpg", IMREAD_UNCHANGED);
	Mat t2 = imread("thirty.jpg", IMREAD_UNCHANGED);
	Mat t3 = imread("fortyfive.jpg", IMREAD_UNCHANGED);
	Mat t4 = imread("sixty.jpg", IMREAD_UNCHANGED);
	if (t1.empty() || t2.empty() || t3.empty() || t4.empty()){
		std::cout << "Las imágenes no fueron leídas correctamente" << endl;
		return 0;
	}
	
	detector.setZero(t1);
	detector.setThirty(t2);
	detector.setFortyFive(t3);
	detector.setSixty(t4);
	
	ofstream data;
	string name;
	vector <int> times;
	cout << "Bienvenido al sistema Automatizado del Ensayo de Caja y Cubos (ABBT)."<<endl<<"Introduzca a continuaci"<< y <<"n el nombre del paciente" << endl;
	getline(cin, name);
	cout << "Gracias. A continuaci" << y << "n aparecer" << v << "n en recuadros blancos las posiciones estimadas de la pantalla divisoria y de las cajas. La caja vac" << x << "a deber" << v << " estar inscrita en un rect" << v << "ngulo de l" << x << "nea gruesa." << endl;
	Sleep(5000);
		
		bool a, b;
		char answer;
		do{
			a = 0;
			b = 0;
			do{
				detector.updateColor(bufferColorMat);
				detector.updateDepth(bufferDepthMat);

				if (detector.getDepthCaptured() && a==0) {
					/*imshow("PROFUNDIDAD", bufferDepthMat);
					waitKey(100000);
					destroyAllWindows();*/
					//imwrite("ProfundidadCompleta.jpg", bufferDepthMat);
					//detector.locateBoxes(bufferDepthMat);//Localizar los compartimentos y la pantalla divisoria
					a = 1;
				}	
				if (detector.getColorCaptured() && b==0) {
					/*imshow("COLOR", bufferColorMat);
					waitKey(1000);
					destroyAllWindows();*/
					//imwrite("ColorCompleta.jpg", bufferColorMat);
					//detector.setCoefs(bufferColorMat);
					b = 1;
				}
				}while (a == 0 || b == 0);

			detector.locateBoxes(bufferDepthMat, bufferColorMat);//Localizar los compartimentos y la pantalla divisoria
			cout<< zz << "Se han posicionado correctamente la caja y el divisor\? Si la localizaci" << y << "n ha sido correcta, pulse s. En caso de no ser as" << x << ", mueva ligeramente la caja y pulse la letra n. S" << x << ",No [s,n]"<<endl;
			cin>>answer;
			//cout << answer<< endl;
		} while ((answer != 's'));// || (answer != 'S')); //Confirmación del usuario del posicionamiento correcto de las cajas y la pantalla
		
		cout << "Ya est" << v << " todo listo. Podemos empezar." << endl;
		Sleep(1000);
		cout << "La prueba comienza en: " << endl << "3" << endl;
		Sleep(1000);
		cout<<"2"<<endl;
		Sleep(1000);
		cout<<"1"<<endl;
		Sleep(1000);
		cout<<"Ya"<<endl;
		unsigned t0 = clock();
		unsigned newTime, oldTime = t0;

		while (((clock() - t0)/CLOCKS_PER_SEC) < TESTTIME) {//Tiempo de la prueba: 60 segundos
				//Detecta entrada de la mano
				//detector.setHandDetected();
				do {
					detector.updateDepth(bufferDepthMat2);
					if (detector.getDepthCaptured()) {
						detector.detectHand(bufferDepthMat2, 0);
						/*imshow("PROFUNDIDAD", bufferDepthMat2);
						waitKey(100000);
						destroyAllWindows();*/
					}
				} while (detector.getHandDetected() == 0);
				std::cout << "Mano dentro" << endl;
				//Detecta salida de la mano
				do {
					detector.updateDepth(bufferDepthMat2);
					if (detector.getDepthCaptured()) {
						detector.detectHand(bufferDepthMat2, 1);
						/*imshow("PROFUNDIDAD", bufferDepthMat2);
						waitKey(100000);
						destroyAllWindows();*/
					}
				} while (detector.getHandDetected());
				std::cout << "Mano fuera" << endl;
				newTime = clock();
				times.push_back((newTime - oldTime)/(CLOCKS_PER_SEC/1000.0));
				//oldTime = newTime;
				detector.increaseHandDetected();
				int siz = times.size();
 				if ((siz >= 2) && ((times[siz - 1] - times[siz - 2]) < 285)) { //Si el paciente ha movido la caja o la pantalla, la detección de la mano se descontrola (aumenta la velocidad), por lo que es necesario recalibrar las ROI's
					bool detectionFailed = 0;
					do {
						detectionFailed = 0;
						do{
							detector.updateDepth(bufferDepthMat3);
						} while (detector.getDepthCaptured() == 0);
						
						if (detector.getDepthCaptured()) {
							detector.correctBoxesPosition(bufferDepthMat3);//Volver a situar la pantalla divisoria por si el paciente la hubiese desplazado
							
							Rect box = detector.getEmptyBox();
							Rect divisor = detector.getDivisor();
					
							if (abs(box.height - box.width) > 50) detectionFailed = 1;
							if ((divisor.height / divisor.width) < 2) detectionFailed = 1;
						}

					} while ((detectionFailed == 1));
				}

					
				Sleep(75);
				do {
					detector.updateColor(bufferColorMat2);
				} while (detector.getColorCaptured() == 0);
				
				//detector.whitePatchTransf(bufferColorMat2);
				/*
				imshow("white", bufferColorMat2);
				waitKey(100000);
				destroyAllWindows();*/
				//imwrite("Image.jpg", bufferColorMat);

				do {
					detector.updateDepth(bufferDepthMat3);
				} while (detector.getDepthCaptured() == 0);

				//imwrite("colorCompleta.jpg", bufferColorMat);
				//imwrite("profundidadCompleta.jpg", bufferColorMat);
				
				Size dsize = Size(1506, 1206);
				resize(bufferDepthMat3, resizedDepth, dsize, 0, 0);

				detector.detection(bufferColorMat2, resizedDepth);

			}

	cout<<endl<<"FIN DE LA PRUEBA"<<endl;
	detector.setEnd();

	//Forzar a que la última detección sea más precisa
	int h = detector.getHandDetections();
	int c = detector.getCountedBlocks();
	int colors[5];
	Mat detection;
	Mat tempDetection = detector.getFinalResult().clone();
	colors[0] = detector.getRed();
	colors[1] = detector.getGreen();
	colors[2] = detector.getBlue();
	colors[3] = detector.getYellow();
	colors[4] = detector.getBlack();

	if ((abs(h - c)) > 0){
		cout << "Espere un momento mientras se realizan unas " << z << "ltimas comprobaciones" << endl;
		
		for (int i = 0; i < 5; i++){
			detector.detection(bufferColorMat2, resizedDepth);
			if (detector.getCountedBlocks() > c){
				tempDetection = detector.getFinalResult();
				c = detector.getCountedBlocks();
				colors[0] = detector.getRed();
				colors[1] = detector.getGreen();
				colors[2] = detector.getBlue();
				colors[3] = detector.getYellow();
				colors[4] = detector.getBlack();
			}		
		}
		
		cout << "Fin de las comprobaciones." << endl;
	}
	detection = tempDetection.clone();

	cout<<"Finalmente, se han detectado "<< detector.getCountedBlocks()<<" cubos ("<<colors[0]<<" cubos rojos, "<<colors[1]<<" cubos verdes, "<<colors[2]<<" cubos azules, "<<colors[3]<<" cubos amarillos y "<<colors[4]<<" cubos de un color sin determinar"<<") y "<<detector.getHandDetections()<< " pasos de la mano."<<endl;
	cout<<endl<<"Los registros de tiempo en segundos entre pasos de la mano han sido los siguientes: "<<endl;
	for (int i = 0; i<times.size(); i++){
		cout<<times[i]/1000.0<<" ";	
	}
	cout<<endl;
	float num;
	cout << "Introduzca el n" << z << "mero real de cubos movidos para calcular el porcentaje de acierto del sistema" << endl;
	cin >> num;
	float handPrecission = (detector.getHandDetections() / num) * 100.0;
	float blocksPrecission = (detector.getCountedBlocks() / num) * 100.0;

	//Guardar los datos del ensayo en el fichero
	data.open("resultados.csv", ios::out|ios::app);
	data<<endl<<"'"<<name<<"'"<<",";
	if (detector.getHandTested()) data<<"'Derecha',";
	else data << "'Izquierda',";
	data<<detector.getCountedBlocks()<<",";
	data << blocksPrecission << "% ,";
	data<<times.size()<<",";
	data << handPrecission << "% ,";
	for (int i = 0; i<times.size(); i++) data<<times[i]/1000.0<<",";
	data.close();
	cout<<"Los datos han sido guardados en el documento " << ac << "resultados.csv" << cc << " para su consulta y an" << v << "lisis."<<endl;
	
	
	imwrite((String(name) + ".jpg"), detection);
	imshow("Resultado de la deteccion", detection);
	waitKey(500000);
	destroyAllWindows();
	return 0;
}
/*Utilidades

//MOSTRAR IMAGEN DE COLOR
if (detector.getColorCaptured()) {
imshow("COLOR", bufferColorMat);
waitKey(5000);
destroyAllWindows();
}

//MOSTRAR IMAGEN DE PROFUNDIDAD
if (detector.getDepthCaptured()) {
imshow("PROFUNDIDAD", bufferDepthMat);
waitKey(5000);
destroyAllWindows();
} 

//ESCALADO
Mat bufferColorMat2(detector.getColorHeight(), detector.getColorWidth(), CV_8UC4);
Size dsize = Size(15, 10);
resize(bufferColorMat, bufferColorMat2, dsize, 0, 0, CV_INTER_AREA);

//MOSTRAR MATRIZ EN FORMATO NÚMEROS
cout << "Color: " << endl << bufferColorMat2 << endl << endl;

//DIBUJAR UN RECTÁNGULO
 void rectangle(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)

//GUARDAR UNA IMAGEN EN EL DISCO
imwrite("Image.jpg", bufferColorMat);
*/
