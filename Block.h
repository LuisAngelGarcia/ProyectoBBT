/*********************************************************/
//SISTEMA DE ENSAYO DE CAJA Y CUBOS AUTOMATIZADO (ABBT)//
	    //Luis Ángel García Astudillo// 
  //Grado en Ingeniería en Tecnologías Industriales//
         //Universidad Carlos III de Madrid//
/********************************************************/

#include <opencv2/opencv.hpp>
#include <Windows.h>
#include <Kinect.h>

using namespace std;
using namespace cv;

class Block {
	Point pos;
	int color; //0 -> Color no determinado (negro), 1 -> Rojo, 2 -> Verde, 3 -> Azul, 4 -> Amarillo
	int size; //lado del cuadrado que lo contiene: 22, 24 o 26

public:
	//Constructores y destructores
	Block() {};
	Block(Point &p, int c = 0) : pos(p), color(c) {};
	~Block() {};

	//Getters
	int getColor(){ return color; }
	Point &getPos(){return pos; }
	int getSize(){return size; }

	//Setters
	void setColor(int a) { color = a; }
	void setPos(Point &p) { pos = p; }
	void setSize(int a) {size = a;}
};
