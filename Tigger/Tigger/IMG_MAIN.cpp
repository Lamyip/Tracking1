#include "IMG.h"
#include "stdafx.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/video/background_segm.hpp>
#include <string>
#include "stdlib.h"
#include <stdio.h>
#include "Cvblob.h"
#include "Filter.h"
#include "Target.h"
#include "LightGobo.h"
#include <time.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ocl/ocl.hpp"
#include <opencv2/nonfree/ocl.hpp>
#include "opencv2/nonfree/nonfree.hpp"

using namespace cv;
using namespace std;
using namespace cvb;


void
randSeed(void) {
	srand(time(0));
}

double
unifRand(void) {
	return rand() / double(RAND_MAX);
}


typedef double TimeStamp; // in seconds

static const TimeStamp UndefinedTime = -1.0;
#define _HSV2RGB_(H, S, V, R, G, B) \
		  { \
    double _h = H/60.; \
    int _hf = (int)floor(_h); \
    int _hi = ((int)_h)%6; \
    double _f = _h - _hf; \
    \
    double _p = V * (1. - S); \
    double _q = V * (1. - _f * S); \
    double _t = V * (1. - (1. - _f) * S); \
    \
    switch (_hi) \
		    { \
      case 0: \
	      R = 255.*V; G = 255.*_t; B = 255.*_p; \
      break; \
      case 1: \
	      R = 255.*_q; G = 255.*V; B = 255.*_p; \
      break; \
      case 2: \
	      R = 255.*_p; G = 255.*V; B = 255.*_t; \
      break; \
      case 3: \
	      R = 255.*_p; G = 255.*_q; B = 255.*V; \
      break; \
      case 4: \
	      R = 255.*_t; G = 255.*_p; B = 255.*V; \
      break; \
      case 5: \
	      R = 255.*V; G = 255.*_p; B = 255.*_q; \
      break; \
		    } \
		  }


/*****************************************************************************/
//					One Euro Filter Classes Definition		 				  //
/*****************************************************************************/

class LowPassFilter {

	double y, a, s;
	bool initialized;

	void setAlpha(double alpha) {
		if (alpha <= 0.0 || alpha > 1.0)
			throw std::range_error("alpha should be in (0.0., 1.0]");
		a = alpha;
	}

public:

	LowPassFilter(double alpha, double initval = 0.0) {
		y = s = initval;
		setAlpha(alpha);
		initialized = false;
	}

	double filter(double value) {
		double result;
		if (initialized)
			result = a*value + (1.0 - a)*s;
		else {
			result = value;
			initialized = true;
		}
		y = value;
		s = result;
		return result;
	}

	double filterWithAlpha(double value, double alpha) {
		setAlpha(alpha);
		return filter(value);
	}

	bool hasLastRawValue(void) {
		return initialized;
	}

	double lastRawValue(void) {
		return y;
	}

};
class OneEuroFilter {

	double freq;
	double mincutoff;
	double beta_;
	double dcutoff;
	LowPassFilter *x;
	LowPassFilter *dx;
	//LowPassFilter *x1;
	//LowPassFilter *dx1;
	TimeStamp lasttime;

	double alpha(double cutoff) {
		double te = 1.0 / freq;
		double tau = 1.0 / (2 * 14159265358979323846 * cutoff);
		return 1.0 / (1.0 + tau / te);
	}

	void setFrequency(double f) {
		if (f <= 0) throw std::range_error("freq should be >0");
		freq = f;
	}

	void setMinCutoff(double mc) {
		if (mc <= 0) throw std::range_error("mincutoff should be >0");
		mincutoff = mc;
	}

	void setBeta(double b) {
		beta_ = b;
	}

	void setDerivateCutoff(double dc) {
		if (dc <= 0) throw std::range_error("dcutoff should be >0");
		dcutoff = dc;
	}

public:

	OneEuroFilter(double freq,
		double mincutoff = 1.0, double beta_ = 0.0, double dcutoff = 1.0) {
		setFrequency(freq);
		setMinCutoff(mincutoff);
		setBeta(beta_);
		setDerivateCutoff(dcutoff);
		x = new LowPassFilter(alpha(mincutoff));
		dx = new LowPassFilter(alpha(dcutoff));
// 		x1 = new LowPassFilter(alpha(mincutoff));
// 		dx1 = new LowPassFilter(alpha(dcutoff));
		lasttime = UndefinedTime;
	}

	double filter(double value, TimeStamp timestamp = UndefinedTime) {
		// update the sampling frequency based on timestamps
		if (lasttime != UndefinedTime && timestamp != UndefinedTime)
			freq = 1.0 / (timestamp - lasttime);
		lasttime = timestamp;
		// estimate the current variation per second 
		double dvalue = x->hasLastRawValue() ? (value - x->lastRawValue())*freq : 0.0; // FIXME: 0.0 or value?
		double edvalue = dx->filterWithAlpha(dvalue, alpha(dcutoff));
		// use it to update the cutoff frequency
		double cutoff = mincutoff + beta_*fabs(edvalue);
		// filter the given value
		return x->filterWithAlpha(value, alpha(cutoff));
	}
// 	double filter1(double value, TimeStamp timestamp = UndefinedTime) {
// 		// update the sampling frequency based on timestamps
// 		if (lasttime != UndefinedTime && timestamp != UndefinedTime)
// 			freq = 1.0 / (timestamp - lasttime);
// 		lasttime = timestamp;
// 		// estimate the current variation per second 
// 		double dvalue = x1->hasLastRawValue() ? (value - x1->lastRawValue())*freq : 0.0; // FIXME: 0.0 or value?
// 		double edvalue = dx1->filterWithAlpha(dvalue, alpha(dcutoff));
// 		// use it to update the cutoff frequency
// 		double cutoff = mincutoff + beta_*fabs(edvalue);
// 		// filter the given value
// 		return x1->filterWithAlpha(value, alpha(cutoff));
// 	}

	~OneEuroFilter(void) {
		delete x;
		delete dx;
	}

};

/*****************************************************************************/
//					Global variable declaration				 				  //
/*****************************************************************************/
Ptr<BackgroundSubtractorMOG> pMOG; // Background Substractor Pointer Object
int lval = 80; //Low Value for Canny Filter
int hval = 100; //High Value for Canny Filter
int light_threshold = 230; // Light Threshold Value for very bright blob
int Minval = 30;
int Maxval = 40000;
int WindowSizeCatch = 150; // Default value for Window Size
//---------------------------Functions for Trackbar------------------------//
void on_win(int position){ WindowSizeCatch = position; };
void on_h(int position){ hval = position; };//High Value for Canny Filter
void on_l(int position){ lval = position; };//Low Value for Canny Filter
void on_lthr(int position){ light_threshold = position; };// Light Threshold Value for very bright blob
void on_minlength(int position){ Minval = position; };
void on_maxlength(int position){ Maxval = position; };
//---------------------------RGB Image Equalizing functino---------------------------------//
void ImgEqualizer(Mat Input_Array, Mat Output_Array)
{
	Mat TmpArray;
	Mat TmpArray1, TmpArray2,TmpArray3;
	Mat GroupMat[] = { TmpArray1, TmpArray2, TmpArray3 };
	std::vector<cv::Mat> IMG(3);
	cvtColor(Input_Array, TmpArray, CV_BGR2GRAY);
	/*split(TmpArray, GroupMat);
	equalizeHist(TmpArray1, TmpArray1);
	equalizeHist(TmpArray2, TmpArray2);
	equalizeHist(TmpArray3, TmpArray3);
	IMG.at(0) = GroupMat[0];
	IMG.at(1) = GroupMat[1];
	IMG.at(2) = GroupMat[2];*/
	//merge(IMG, Output_Array);
	equalizeHist(TmpArray, TmpArray);
	cvtColor(TmpArray, Output_Array, CV_GRAY2BGR);
	//imshow("output", Output_Array);

}

int fnum = 0;// Current Frame Number
bool changed = false; // Frame number changed detection
int fcount = 0; // maximum frame number of loaded video
void on_f(int position){ fnum = position; 
changed = true;
};

//--------------------------Blob filtering function by length criteria---------------------------------//
void cvFilterByLength(CvBlobs &blobs, unsigned int minLength, unsigned int maxLength)
{
	CvBlobs::iterator it = blobs.begin();
	while (it != blobs.end())
	{
		CvBlob *blob = (*it).second;
		if (((blob->maxx - blob->minx) < minLength) || ((blob->maxy - blob->miny) < minLength) || ((blob->maxy - blob->miny) > maxLength) || ((blob->maxx - blob->minx) > maxLength))
		{
			cvReleaseBlob(blob);
			CvBlobs::iterator tmp = it;
			++it;
			blobs.erase(tmp);
		}
		else
			++it;
	}
}

//--------------------------Blob filtering function by pixel value criteria---------------------------------//
void cvFilterByColor(CvBlobs &blobs, int threshold,  Mat Img, IplImage *msk2)
{
	CvBlobs::iterator it = blobs.begin();
	Mat LBL = Mat(msk2);
	while (it != blobs.end())
	{
		CvBlob *blob = (*it).second;
		Rect region_of_interest = Rect(blob->minx, blob->miny, (blob->maxx - blob->minx), (blob->maxy - blob->miny));
		Mat Roi = Img(region_of_interest);
		Mat Roimsk = LBL(region_of_interest);
		Mat tmpRoi;
		Roi.copyTo(tmpRoi);
		int cnt = 0;
		int avg = 0;
		Vec3b tmp = 0;
		for (int j = 0; j < Roi.rows; j++)
			for (int i = 0; i < Roi.cols; i++)
			{
				if (Roimsk.at<float>(Point(i, j))>0 && Roimsk.at<float>(Point(i, j))!= 0xFFFFFFFF)
				{
					tmp = Roi.at<Vec3b>(Point(i, j));
					Roi.at<Vec3b>(Point(i, j)) = Vec3b(255, 0, 0);
					avg += tmp.val[0];
					cnt++;
				}
			}
		if (cnt !=0)
		avg /= cnt;
		if (avg> threshold)
		{
			cvReleaseBlob(blob);
			CvBlobs::iterator tmp = it;
			++it;
			blobs.erase(tmp);
			cout << "Average Blob = " << avg << endl;
		}
			++it;
	}
}


//---------------------------Variables and functions for fps computing---------------------------------//
int _fpsstart = 0;
double _avgfps = 0;
double _fps1sec = 0;
int CLOCK()
{
	return clock();
}
double avgfps()
{
	if (CLOCK() - _fpsstart>1000)
	{
		_fpsstart = CLOCK();
		_avgfps = 0.7*_avgfps + 0.3*_fps1sec;
		_fps1sec = 0;
	}

	_fps1sec++;
	return _avgfps;
}



int IMG_Main(VideoCapture CapInput, vector<Point2f> *TrackedPoint)
{
	if (!CapInput.isOpened())
		return -1;
	/*****************************************************************************/
	//					Variables Definition and Initialization 				  //
	/*****************************************************************************/
	int height = 0, width = 0;
	//height = 720;
	//width = 1280;
	height = 540;
	width = 960;
	Size CameraSize = Size(width, height);

	Mat bg; // Background Image
	Mat morphKernel = getStructuringElement(MORPH_ELLIPSE, Size(11, 11), Point(3, 3));// Kernel for morphological operations
	Mat morphKernel2 = getStructuringElement(MORPH_CROSS, Size(1, 1), Point(0, 0));// Kernel for morphological operations. cross shape
	Mat Foreground; //Foreground blob image
	vector<vector<Point> > contours; // Contours list
	vector<Vec4i> hierarchy; //Contour Hierarchy

	Mat fgMaskMOG;// Foreground mask for MOG algorithm
	Mat input = Mat(height, width, CV_8UC3);// Input Image
	Mat edges = Mat(height, width, CV_8UC1);// Edges Image in Grayscale
	Mat frame = Mat(height, width, CV_8UC3); // Input Image
	Mat mask; // Outer mask image for detection
	Mat mask2; //Inner mask image to filter the edges noise from the pedestals.
	Mat mask3;
	vector<vector<Point>> Circle;// ?
	vector<Vec4i> hierarchyCircle;// ?

	std::clock_t start; // Clock for fps computation

	OneEuroFilter *f; // One Euro Filter
	double duration;
	vector<Target> Targets; // Targets list
	vector<Target> Targetsbis; // targets list for post processing
	Targets.clear();
	start = std::clock();
	double frequency = 120; // Hz
	double mincutoff = 1.0; // FIXME
	double beta = 1.0;      // FIXME
	double dcutoff = 1.0;   // this one should be ok
	Mat map_x, map_y;// x and y mapping matrix for mirroring
	CapInput.set(CV_CAP_PROP_FRAME_WIDTH, width); // Forces resolution to be 1280*720
	CapInput.set(CV_CAP_PROP_FRAME_HEIGHT, height);
	map_x.create(Size(width, height), CV_32FC1);
	map_y.create(Size(width, height), CV_32FC1);
	namedWindow("edges", 0);
	namedWindow("frame", 0);
    fcount = (int)CapInput.get(CV_CAP_PROP_FRAME_COUNT);
 	mask = imread("OverMasque.png", CV_LOAD_IMAGE_GRAYSCALE);
	mask2 = imread("TigerMasqueInterne.png", CV_LOAD_IMAGE_GRAYSCALE); //mask2 = imread("TigerShow2402MaskSeat.png", CV_LOAD_IMAGE_GRAYSCALE);
	mask3 = imread("TigerShow2402Maskinterne.png", CV_LOAD_IMAGE_GRAYSCALE);
	bg = imread("BG2.png", CV_LOAD_IMAGE_COLOR);

	// Mask size adjustements
	resize(mask, mask,Size(width,height));
	resize(mask2, mask2, Size(width, height));
	resize(bg, bg, Size(width, height));
	resize(mask3, mask3, Size(width, height));
	
	// Mirroring remapping matrixes computation 
	for (int j = 0; j < height; j++)
	{
		for (int i = 0; i < width; i++)
		{
			map_x.at<float>(j, i) = i;
			map_y.at<float>(j, i) = height - j;
		}
	}
	// 1st frame retrieval
	CapInput.set(CV_CAP_PROP_POS_FRAMES, fnum);
	CapInput >> input;
	resize(input, frame, Size(width,height));
	//remap(input, frame, map_x, map_y, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
	//pMOG = new BackgroundSubtractorMOG2();
	pMOG = new BackgroundSubtractorMOG(200,5,0.7,30*1); // Parameters setting for MOG algorithm
	//history = 200 frames
	// Number of gaussian mixtures = 5
	// Background ratio = 0.7
	// Noise strengh = 30
	pMOG->operator()(bg, fgMaskMOG); //Background loading model

	// Trackbars declaration for live adjustment
	cvCreateTrackbar("Low", "edges", &lval, 100, on_l);
	cvCreateTrackbar("High", "edges", &hval, 100, on_h);
  	//cvCreateTrackbar("Frame", "edges", &fnum, fcount, on_f);
	cvCreateTrackbar("Frame", "edges", &fnum, fcount, on_f);
	cvCreateTrackbar("threshold", "edges", &light_threshold, 255, on_lthr);
	cvCreateTrackbar("window size", "frame", &WindowSizeCatch, 250, on_win);
	//cvCreateTrackbar("Frame", "edges", &fnum, 300000, on_f);
	//bitwise_xor(mask, mask2, mask3);
	//namedWindow("rawframe", 0);
	int counter = 1;
 	OneEuroFilter *fx = new OneEuroFilter(frequency, mincutoff, beta, dcutoff);//
 	OneEuroFilter *fy = new OneEuroFilter(frequency, mincutoff, beta, dcutoff);//
	time_t tstart, tend;//
	vector<Point> TrackedPointPix; // Tracked point position vector 
	vector<Point2f> OutputTrackedPoint(12,Point2f(0.0, 0.0));
	vector<bool> IDTab(12,true);
	Mat Unequalized_frame;
	Scalar TargetsColor[12]; // Color representation for reprojection for each gobo.
	vector<LightGobo> LightsGobos; // Gobos projection coordinate vector
	// color computation
	for (int l = 0; l < 12; l++)
	{
		int r, g, b;
		_HSV2RGB_((double)((l * 77) % 360), .5, 1., r, g, b);
		TargetsColor[l] = Scalar(r,g,b,0);
	}
	//Gobo initialized on tiger seats.
	LightsGobos.push_back(LightGobo(182, 245));// Light 0...
	LightsGobos.push_back(LightGobo(173, 376));//...
	LightsGobos.push_back(LightGobo(199, 520));
	LightsGobos.push_back(LightGobo(274, 631));
	LightsGobos.push_back(LightGobo(376, 712));
	LightsGobos.push_back(LightGobo(586, 713));
	LightsGobos.push_back(LightGobo(704, 643));
	LightsGobos.push_back(LightGobo(776, 519));
	LightsGobos.push_back(LightGobo(812, 371));
	LightsGobos.push_back(LightGobo(518, 150));
	LightsGobos.push_back(LightGobo(510, 1));// ...
	LightsGobos.push_back(LightGobo(540, 360));// Light 11
	//for (int k = 0; k < 9;k++)
	//{
	//	IDTab[k] = false;
	//}

	cv::ocl::PlatformsInfo platforms;
	cv::ocl::DevicesInfo Devices;
	cv::ocl::getOpenCLPlatforms(platforms);
	cv::ocl::getOpenCLDevices(Devices,cv::ocl::CVCL_DEVICE_TYPE_GPU);
	cv::ocl::setDevice(Devices[0]);
	cv::ocl::oclMat Oclmask = cv::ocl::oclMat(mask);
	cv::ocl::oclMat Oclmask2 = cv::ocl::oclMat(mask2);
	cv::ocl::oclMat Oclfg = cv::ocl::oclMat(fgMaskMOG);
	cv::ocl::oclMat Oclbg = cv::ocl::oclMat(bg);
	cv::ocl::oclMat Oclframe = cv::ocl::oclMat(input);
	cv::ocl::oclMat OclMskFg;
	cv::ocl::resize(Oclframe, Oclframe, Size(width, height));
	cv::ocl::MOG OclMOG(5);
	OclMOG.history = 200;
	OclMOG.backgroundRatio = 0.7;
	OclMOG.noiseSigma = 30*1;
	OclMOG.varThreshold = 6.25;
	Mat input1;
	Mat edges1 = Mat(height, width, CV_8UC1);
	OclMOG(Oclbg, Oclfg);
	
	/*****************************************************************************/
	//					End of initialization					 				  //
	/*****************************************************************************/


	/*****************************************************************************/
	//					Real time Image Processing				 				  //
	/*****************************************************************************/
	for (;;)
	{
		TrackedPoint->clear();
		if (changed) {// Check if Frame count has changed
			CapInput.set(CV_CAP_PROP_POS_FRAMES, fnum);
			changed = false;
		}
		CapInput >> frame;// grab next frame
		MedialonCommand.lock();
		if (!MedialonCommand.activation)
		{
			

			//Oclframe.upload(frame);
			//
			Oclframe = cv::ocl::oclMat(frame);// attempt to use OpenCl
			cv::ocl::resize(Oclframe.clone(), Oclframe, Size(width, height));
			//imshow("Ocl", (cv::Mat)Oclframe);
			//Oclframe.download(input1);
			//imshow("Imshow", input1);
			//resize(frame, frame, Size(width, height)); // Size reduction for fast processing
			TrackedPointPix.clear();
			//remap(frame, frame, map_x, map_y, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
			Oclframe.download(frame);
			frame.copyTo(input);
			Targetsbis.clear();
			/*****************************************************************************/
			//					Canny Filtering  					 					  //
			/*****************************************************************************/
			cvtColor(frame, edges, CV_BGR2GRAY); // Conversion of current frame in grayscale

			cv::ocl::oclMat OclEdges(edges);
			cv::ocl::resize(OclEdges.clone(), OclEdges, Size(width, height));
			cv::ocl::bitwise_and(OclEdges, Oclmask, OclEdges);
			cv::ocl::GaussianBlur(OclEdges, OclEdges, Size(7, 7), 1.5, 1.5);
			cv::ocl::Canny(OclEdges.clone(), OclEdges, lval, hval, 3);
			cv::ocl::morphologyEx(OclEdges.clone(), OclEdges, CV_MOP_CLOSE, morphKernel);
			cv::ocl::bitwise_and(OclEdges, Oclmask2, OclEdges);
			//cv::ocl::erode(OclEdges, OclEdges, morphKernel2);
			cv::ocl::morphologyEx(OclEdges, OclEdges, CV_MOP_CLOSE, morphKernel);
			imshow("OCL", (cv::Mat)OclEdges);

			//Mat alt;
			//OclAlt.download(alt);
			//

				/*****************************************************************************/
				//					Canny filtering without OCl module (to comment)			  //
				/*****************************************************************************/
				//bitwise_and(edges, mask, edges); // masking the scene in image
				//GaussianBlur(edges, edges, Size(7, 7), 1.5, 1.5); // Blurring
				//Canny(edges, edges, lval, hval, 3);
				//imshow("Canny", edges);
				//morphologyEx(edges, edges, CV_MOP_CLOSE, morphKernel); // closure operation

				////resize(mask, mask, Size(width,height));
				//bitwise_and(edges, mask2, edges); // Innermask
				//bitwise_and(edges, mask2, edges); // Application du mask
				//erode(edges, edges, morphKernel2);
				//morphologyEx(edges, edges, CV_MOP_CLOSE, morphKernel);



			

			/*****************************************************************************/
			//					Background Substraction 				 				  //
			/*****************************************************************************/
			//pMOG->operator()(frame, fgMaskMOG, 0.009);
			//OclMOG(Oclframe, Oclfg, 0.009f);
			//cv::ocl::oclMat Oclforeground;
			//Oclfg.copyTo(Oclforeground);
			//cv::ocl::bitwise_and(Oclforeground, Oclmask, Oclforeground);
			OclMOG(Oclframe, Oclfg, 0.001);
			//imshow("Ocl", (cv::Mat)Oclfg);
			ocl::bitwise_and(Oclfg, Oclmask, OclMskFg);


				/*****************************************************************************/
				//					Background Without Ocl to be commented	 				  //
				/*****************************************************************************/
				//pMOG->operator()(frame, fgMaskMOG, 0.001); // Background update and Foreground mask retrieval
				////pMOG->apply(frame, fgMaskMOG);
				//static Mat Foreground;			
				//fgMaskMOG.copyTo(Foreground);
				//imshow("fg", Foreground);
				////dilate(Foreground, Foreground,morphKernel);
				////edges.copyTo(EdgesContours);
				//bitwise_and(Foreground, mask, Foreground); // Masking with scene


			// 		if (contours.size() != 0)
			// 		{
			// 			drawContours(edges, contours, contours.size() - 1, Scalar(0), 4, 8, hierarchy, 2, Point());
			// 		}
			//bitwise_and(mask, fgMaskMOG, fgMaskMOG);
			//bitwise_xor(edges, MaskProcess, edges);

			/*****************************************************************************/
			//					Methods fusion (Edges et Background Substractor)  //
			/*****************************************************************************/
			ocl::bitwise_or(OclEdges, OclMskFg, OclMskFg);//Blob Fusion using OpenCL
			OclMskFg.download(Foreground);
			//bitwise_or(edges, Foreground, Foreground); // blob fusion
			
			
			//morphologyEx(Foreground, Foreground, CV_MOP_CLOSE, morphKernel); // closure
			//dilate(Foreground, Foreground, morphKernel);// dilatation
			//dilate(Foreground, Foreground, morphKernel);

			//cv::ocl::bitwise_or(OclEdges, Oclforeground, Oclforeground);
			//cv::ocl::morphologyEx(Oclforeground, Oclforeground, CV_MOP_CLOSE, morphKernel);
			//cv::ocl::dilate(Oclforeground, Oclforeground, morphKernel);
			//Oclforeground.download(Foreground);


			/*****************************************************************************/
			//					Contours list computing									  //
			/*****************************************************************************/
			findContours(Foreground, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_L1, Point(0, 0));

			/*****************************************************************************/
			//					Blob Identification										 //
			/*****************************************************************************/

			static IplImage raw = Foreground;
			static IplImage rawframe = frame;
			static IplImage *labelImg = cvCreateImage(cvGetSize(&raw), IPL_DEPTH_LABEL, 1);

			CvBlobs blobs;// blobs vector reinitialization
			IplImage *copytmp;
			copytmp = cvCloneImage(&rawframe);
			unsigned int result = cvLabel(&raw, labelImg, blobs); // blobs labeling
			cvFilterByArea(blobs, 100, 30000);// blobs filtering by area
			cvFilterByLength(blobs, 40, 400);
			cvFilterByColor(blobs, light_threshold, frame, labelImg);
			cvRenderBlobs(labelImg, blobs, &rawframe, &rawframe); //draw blob on image
			int i = 0;
			system("cls");
			//imshow("rawframe", Mat(&rawframe, false));
			//static Mat framex = imgOut;

			/*****************************************************************************/
			//					Targets coordinate retrieval							 //
			/*****************************************************************************/
			static float x, y;
			int ib = 0;
			if (blobs.size() != 0)
			{
				for (CvBlobs::iterator itblob = blobs.begin(); itblob != blobs.end(); ++itblob)
				{
					CvBlob *blob = (*itblob).second;
					x = (float)blob->centroid.x;
					y = (float)blob->centroid.y;
					//TrackedPoint->push_back(Point2f(y - height / 2, x - width / 2));// Add Center Offset
					TrackedPointPix.push_back(Point((int)x, (int)y));
					ib++;
				}
			}

			//Filtering
			//if (!TrackedPoint->empty())
			//{
			//	vector<Point2f>::iterator it2 = TrackedPoint->begin();
			//	it2->x = fx->filter(it2->x, duration);
			//	it2->y = fy->filter(it2->y, duration);
			//}

			/*****************************************************************************/
			//					Target Initialization									  //
			/*****************************************************************************/
			//for (int r = 0; r < Targets.size(); r++)
			//{
			//	Targets[r].DisableTrack();
			//}

			/*****************************************************************************/
			//					Target Matching with blobs								  //
			/*****************************************************************************/
			//for (int p = 0; p < TrackedPointPix.size(); p++)
			//{
			//	bool tmp = false;
			//	for (int r = 0; r < Targets.size();r++)
			//	{
			//		tmp |= Targets[r].MatchWindow(TrackedPointPix[p].x, TrackedPointPix[p].y);
			//		Targets[r].CheckTrack(tmp);
			//	}
			//	if (!tmp)
			//		Targets.push_back(Target((float)TrackedPointPix[p].x, (float)TrackedPointPix[p].y));
			//}

			//for (int r = 0; r < Targets.size(); r++)
			//{
			//	Targets[r].LostPositionStatusUpdate();
			//	if (Targets[r].GetStatus() == Lost)
			//	{
			//		IDTab[Targets[r].GetTargetID()] = true;
			//		Targets.erase(Targets.begin() + r);
			//		cout << "Erasing" << endl;
			//	}
			//}
			//// polling
			//for (int r = 0; r < Targets.size(); r++)
			//{			
			//	if (Targets[r].EventNewTrack())
			//	{
			//		Targets[r].IDAttribution(&IDTab);
			//	}
			//	if (Targets[r].GetStatus() == Tracked)
			//	{
			//		OutputTrackedPoint[Targets[r].GetTargetID()] = Targets[r].GetCenter();
			//		circle(frame, Targets[r].GetCenter(), 1, Targets[r].GetColor(), 4);
			//	}
			//}
			//for (int r = 0; r < 12; r++)
			//{
			//	circle(frame, OutputTrackedPoint[r], 50, TargetsColor[r], 4);
			//	rectangle(frame, Rect(OutputTrackedPoint[r].x, OutputTrackedPoint[r].y, 50, 50), TargetsColor[r], 1);
			//}
			//imshow("FrameTarget", frame);
			//Targetsbis = Targets;

			if (!TrackedPointPix.empty())
			{
				Mat Distance = Mat(12, TrackedPointPix.size(), CV_32F); // Lights-Target Distance Matrix initialization
				for (int i = 0; i < Distance.rows; i++)// Rows = Lights Number
					for (int j = 0; j < Distance.cols; j++) // Columns = Targets Number
					{
						//-----------------------Squared Distance ------------------------//
						int Xsquare = TrackedPointPix[j].x - LightsGobos[i].GetCenter().x;
						int Ysquare = TrackedPointPix[j].y - LightsGobos[i].GetCenter().y;
						Distance.at<float>(i, j) = sqrt((Xsquare)*(Xsquare)+(Ysquare)*(Ysquare));
					}
				for (int j = 0; j < 12; j++)
				{
					static double min; // Minimum Value
					Point minloc; // Minimum Location
					//static int Idx;
					//minMaxLoc(DistSparse, &min,NULL,&Idx);
					minMaxLoc(Distance, &min, NULL, &minloc, NULL); // Minimal Light Target retrieval computation
					//if (min < LightsGobos[minloc.y].GetWinSize())
					if (min < WindowSizeCatch) // Light Position update if Distance < adjustable threshold
					{
						LightsGobos[minloc.y].UpdatePos(TrackedPointPix[minloc.x].x, TrackedPointPix[minloc.x].y);
						LightsGobos[minloc.y].resetCounter();
					}
					else
					{
						LightsGobos[minloc.y].updateCounter();
						LightsGobos[minloc.y].UpdateWindow();
					}
					//-----------------------Fill chosen column and row with arbitrary value----------------//
					for (int k = 0; k < 12; k++)
					{
						Distance.at<float>(k, minloc.x) = 1000.0;
					}
					for (int k = 0; k < TrackedPointPix.size(); k++)
					{
						Distance.at<float>(minloc.y, k) = 1000.0;
					}
					//Draw circle
					circle(input, LightsGobos[j].GetCenter(), 35, TargetsColor[j], 2);
					//rectangle(input, Rect(LightsGobos[j].GetCenter().x, LightsGobos[j].GetCenter().y, 50, 50), TargetsColor[j], 1);
					//Draw center point
					line(input, LightsGobos[j].GetCenter(), LightsGobos[j].GetCenter(), TargetsColor[j], 5);
					// Draw Light Number
					putText(input, to_string(j), LightsGobos[j].GetCenter(), FONT_HERSHEY_PLAIN, 2.0, TargetsColor[j], 2);
				}
			}

			//if (!Targets.empty())
			//{
			//	Mat Distance = Mat(12, Targets.size(), CV_32F);
			//	for (int i = 0; i < Distance.rows; i++)
			//		for (int j = 0; j < Distance.cols; j++)
			//		{
			//			int Xsquare = Targets[j].GetCenter().x - LightsGobos[i].GetCenter().x;
			//			int Ysquare = Targets[j].GetCenter().y - LightsGobos[i].GetCenter().y;
			//			Distance.at<float>(i, j) = sqrt((Xsquare)*(Xsquare)+(Ysquare)*(Ysquare));
			//		}
			//	//static SparseMat DistSparse = SparseMat(Distance);
			//	for (int j = 0; j < 12; j++)
			//	{
			//		static double min;
			//		Point minloc;
			//		//static int Idx;
			//		//minMaxLoc(DistSparse, &min,NULL,&Idx);
			//		minMaxLoc(Distance, &min, NULL, &minloc, NULL);
			//		if (min < 200.0)
			//		{
			//			LightsGobos[minloc.y].UpdatePos(Targets[minloc.x].GetCenter().x, Targets[minloc.x].GetCenter().x);
			//		}
			//		for (int k = 0; k < 12; k++)
			//		{
			//			Distance.at<float>(k, minloc.x) = 1000.0;
			//		}
			//		for (int k = 0; k < Targets.size(); k++)
			//		{
			//			Distance.at<float>(minloc.y, k) = 1000.0;
			//		}
			//		circle(input, LightsGobos[j].GetCenter(), 50, TargetsColor[j], 4);
			//		rectangle(input, Rect(LightsGobos[j].GetCenter().x, LightsGobos[j].GetCenter().y, 50, 50), TargetsColor[j], 1);
			//	}
			//}
			//for (int r = 0; r < 12; r++)
			//{
			//	bool temp = false;
			//	for (int l = 0; l < Targetsbis.size(); l++)
			//	{
			//		
			//		//if (Targets[l].GetStatus() == Tracked)
			//		temp |= LightsGobos[r].MatchWindow(Targetsbis[l].GetCenter().x, Targetsbis[l].GetCenter().y);
			//		if (temp|| !Targetsbis.empty())
			//		{
			//			Targetsbis.erase(Targetsbis.begin() + l);
			//			break;
			//		}
			//	}
			//	if (!temp)
			//		LightsGobos[r].updateCounter();
			//	circle(input, LightsGobos[r].GetCenter(),50,TargetsColor[r], 4);
			//	rectangle(input, Rect(LightsGobos[r].GetCenter().x, LightsGobos[r].GetCenter().y, 50, 50), TargetsColor[r], 1);
			//}

			//Filter_Process(&OutputTrackedPoint);
			//if (MedialonCommand.activation)
			//{
			//	cout << "/n Ping /n";
			//	MedialonCommand.lock();
			//	MedialonCommand.activation = false;
			//	MedialonCommand.unlock();
			//}

			imshow("frame", input);
			cout << endl << " FPS: " << avgfps();
			//counter++;
		}
		MedialonCommand.unlock();
		//cout << CapInput.get(CV_CAP_PROP_FRAME_COUNT);
		if (waitKey(1) >= 0) break;
	}
	//delete fx,fy;
	cv::destroyAllWindows();
	return 0;
}

