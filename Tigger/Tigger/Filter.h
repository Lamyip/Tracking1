#ifndef FILTER_H
#define FILTER_H

#include "stdafx.h"
#include "stdlib.h"
#include <stdio.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <vector>
#include <math.h>
#include <algorithm>
using namespace std;
using namespace cv;

void Filter_Process(vector<Point2f>*);
void Filter_Sort(vector<Point2f>* InputVec, vector<Point2f>* OutputVec);
void Filter_Swap(vector<Point2f>* Vec, int a, int b);
float Filter_Dist(Point2f PointA, Point2f PointB);
void Filter_Resize(vector<Point2f>* VecA, vector<Point2f>* VecB);
vector<Point2f> Filter_Sort2(vector<Point2f>* InputVec, vector<Point2f>* OutputVec);
void DataUDPSend(vector<Point2f> DataTab, vector<float> panoff, vector<float> tiltoff);
void line_add(ifstream& finput, vector<float>& Xcam, vector<float>& Ycam, vector<float>& Pan, vector<float>& Tilt);
extern vector<Point2f> VecOut;
void VectDisplay(vector<Point2f> *vinput);
void DataUDPReceive();

#endif // !FILTER_H

