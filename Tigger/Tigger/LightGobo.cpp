#include "stdafx.h"
#include "LightGobo.h"


LightGobo::LightGobo()// Class Default Contructor
{
}


LightGobo::~LightGobo()// Class Destructor
{
}

LightGobo::LightGobo(int x, int y) // Class constructor
{
	Center = cv::Point(x, y); // Center Position Initialization
	Origin = cv::Point(x, y); // Origin Position Initialization
}

void LightGobo::UpdatePos(float x, float y)
{
	Center.x = (int)x; // Position Update
	Center.y = (int)y;
}



bool LightGobo::MatchWindow(float x, float y)// Checks if a target fit in the tracking window and update position if true
{
	bool MatchingBool = false;
	MatchingBool = (x >= Center.x - WinSize) && (x <= Center.x + WinSize) ? true : false;
	MatchingBool &= (y >= Center.y - WinSize) && (y <= Center.y + WinSize) ? true : false;
	if (MatchingBool)
	{
		UpdatePos(x, y);// Position update
		counter = 0; // counter reset
	}
	return MatchingBool;
}

cv::Point2f LightGobo::GetCenter() // return Current position
{ 
	cv::Point2f tmp; 
	tmp.x = (float)Center.x; 
	tmp.y = (float)Center.y; 
	return tmp; 
}


void LightGobo::resetCounter()
{
	counter = 0; // Counter reset
	WinSize = 100; // Window reset
}

void LightGobo::updateCounter()// Increment counter by one if no target detected for current frame
{
	if (counter > 10)  // return to origin point if no target detected for n frames
	{
		Center = Origin;
		counter = 0;
	}
	counter++;
}