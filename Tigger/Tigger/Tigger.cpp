// Tigger.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <opencv2/video/background_segm.hpp>
#include <string>
#include "stdlib.h"
#include <stdio.h>
#include "Tigger.h"
#include <boost\thread.hpp>


using namespace cv;
using namespace std;

int IMG_Main(VideoCapture CapInput, vector<Point2f> *TrackedPoint);

void LaunchCap()
{
	//VideoCapture CapTest(1);
	//VideoCapture CapTest("tiger2602.mp4");
	VideoCapture CapTest("Video2402.mp4");
	//VideoCapture CapTest("Film Tigre 12022015comp.mp4");
	//VideoCapture CapTest("TigerShow2402.mp4");
	//VideoCapture CapTest("TigerShow0503_1.mp4");	
	vector<Point2f> VecInt;
	if (CapTest.isOpened())
	{

		
		IMG_Main(CapTest,&VecInt);

	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	//boost::thread receiveUDP(DataUDPReceive);
	//boost::thread Capture(LaunchCap);
	//cout << getBuildInformation();
	//receiveUDP.join();
	//Capture.join();

	//VideoCapture CapTest(1);
	//VideoCapture CapTest("tiger2602.mp4");
	VideoCapture CapTest("Video2402.mp4");
	//VideoCapture CapTest("Film Tigre 12022015comp.mp4");
	//VideoCapture CapTest("TigerShow2402.mp4");
	//VideoCapture CapTest("TigerShow0503_1.mp4");	
	vector<Point2f> VecInt;
	if (CapTest.isOpened())
	{
		IMG_Main(CapTest, &VecInt);
	}
	return 0;
}

