#ifndef IMG_H
#define IMG_H
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <ctime>


 using namespace cv;


int IMG_Main(VideoCapture CapInput, vector<Point2f> *TrackedPoint);


#endif // !IMG_H

