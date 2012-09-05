#pragma once

#include <assert.h>
#include <math.h>

#include "opencv2\opencv.hpp"

class Point2D;
class Size;

class Color
{
public: 

	Color();
	Color(int red, int green, int blue);
	Color(int idx);

	int red;
	int green;
	int blue;
};

class Rect
{
public:

	Rect();
	Rect(int upper, int left, int height, int width);

	int upper;
	int left;
	int height;
	int width;

	float confidence;

	Rect operator+ (Point2D p);
	Rect operator+ (Rect r);
	Rect operator- (Point2D p);
	Rect operator* (float f);
	Rect operator= (Size s);
	Rect operator= (Rect r);
	bool operator== (Rect r);
	bool isValid (Rect validROI);
	
	int checkOverlap (Rect rect);
	int getArea(){return height*width;};
    bool isDetection(Rect eval, unsigned char *labeledImg, int imgWidth);
	Point2D GetCenter();
	Rect getCvRect();
}; 

class Size
{
public:

	Size();
	Size(int height, int width);

	int height;
	int width;

	Size operator= (Rect r);
	Size operator= (Size s);
	Size operator* (float f);
	bool operator== (Size s);

	int getArea();
};


class Point2D
{
public:

	Point2D();
	Point2D(int row, int col);
	Point2D(double dr,double dc);

	int row;
	int col;
	double drow;
	double dcol;

	Point2D operator+ (Point2D p);
	Point2D operator- (Point2D p);
	Point2D operator= (Point2D p);
	Point2D operator= (Rect r);
	
};