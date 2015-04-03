#include "stdafx.h"
#include "Parameters.h"


Parameters::Parameters()
{
}


Parameters::~Parameters()
{
}


void PointVector::WriteSharedPoints(std::vector<cv::Point2f> &a)
{
	for (int k = 0; k < a.size(); k++)
		SharedPoints[k] = a[k];
}
