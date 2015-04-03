#include "stdafx.h"
#include "math.h"
#include "vector"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include "LIT.h"

#define HEIGHT 11.0
#define focal 0.12 
using namespace std;
using namespace cv;

Point2f LIT_Angle(Point2f *Position,float xoff, float yoff)//, Point2f *Angle)
{
	Point2f tmp;//Angle point
				// x = Pan
				// y = Tilt
	float x = Position->x ; // X coordinate retrieval
	float y = Position->y; // Y coordinate retrieval
	x *= focal / HEIGHT; // Real world coordinate 
	y *= focal / HEIGHT;
	x -= xoff; // Offset
	y -= yoff;
	//cout << "x :" << x << " m " << endl;
	//cout << "y :" << y << " m " << endl;
	if (x == 0)
	{
		tmp.x = 90.0;
		tmp.y = atan(sqrt(x*x +y*y) / HEIGHT) * 180 / 3.14;
	}
	else if (x < 0 && y >0)
	{
		tmp.x = 180 - atan(-y / x) * 180 / 3.14;
		tmp.y = atan(sqrt(x*x + y*y) / HEIGHT) * 180 / 3.14;
	}
	else if (x < 0 && y <0)
	{
		tmp.x = -180 - atan(y / x) * 180 / 3.14;
		tmp.y = atan(sqrt(x*x + y*y) / HEIGHT) * 180 / 3.14;
	}
	else
	{
		tmp.x = atan(y/x)*180/3.14;
		tmp.y = atan(sqrt(x*x+y*y)/HEIGHT)*180/3.14;
	}
	
	return tmp;
}

// void LIT_Data_Encode(vector<Point2f> *Position, T_ArtDmx *Tx_Data)
// /*{*/
// 	int k =  0;
// 	(Tx_Data->ID) = "Art-Net";
// 	Tx_Data->Sequence = 0;
// 
// 	for each (Point2f Pt in *Position)
// 	{
// 		Tx_Data->Data[k*30 + 1] = (uchar) Pt.x ;
// 		Tx_Data->Data[k*30 + 3] = (uchar) Pt.y ;
// 		k++;
// 	}
// }