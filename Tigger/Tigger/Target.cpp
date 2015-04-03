#include "stdafx.h"
#include "Target.h"
#include <opencv2/core/core.hpp>
#include <stdlib.h> 
#include <iostream>
#include <vector>


Target::~Target()
{
}

Target::Target(float x, float y)
{
	TargetStatus = CandidateForTracking;
	SquareSizeWin = 50;
	XCentroid = x;
	YCentroid = y;
	TargetColor = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);
	TrackedCounter = 0;
	UpdateWin();
}
cv::Point Target::GetCenter()
{
	cv::Point tmp;
	tmp.x = this->XCentroid;
	tmp.y = this->YCentroid;
	return tmp;

}

cv::Scalar Target::GetColor()
{
	return TargetColor;
}

void Target::LostPositionStatusUpdate()
{
	//std::cout << "hello<" << " " ;
	if (PositionTracked == false)
	{
		//std::cout << "hello<";
		switch (TargetStatus)
		{
		case CandidateForTracking:
			this->TargetStatus = LostPositionCandidate;
			this->SquareSizeWin = 100;
			this->TrackedCounter = 0;
			break;
		case LostPositionCandidate:
			this->TrackedCounter++;
			this->SquareSizeWin = 100;
			UpdateWin();
			if (this->TrackedCounter > 5)
			{
				this->TargetStatus = Lost;
				this->TrackedCounter = 0;
			}
			break;
		case Tracked:
			this->TargetStatus = TrackedLostPosition;
			this->SquareSizeWin = 100;
			break;
		case TrackedLostPosition:
			this->TrackedCounter++;
			this->SquareSizeWin = 100;
			UpdateWin();
			if (this->TrackedCounter > 10)
			{
				this->TargetStatus = Lost;
				this->TrackedCounter = 0;
			}
			break;
		case Lost:
			this->~Target();
			break;
		default:
			break;
		}
	}
}

void Target::DisableTrack()
{
	this->PositionTracked = false;
}

void Target::CheckTrack(bool b_input)
{
	this->PositionTracked |= b_input;
}

void Target::UpdateWin()
{
	this->XminWin = XCentroid - SquareSizeWin > 0 ? XCentroid - SquareSizeWin : 0; this->XmaxWin = XCentroid + SquareSizeWin > 1080 ? 1080 : XCentroid + SquareSizeWin;
	this->YminWin = YCentroid - SquareSizeWin > 0 ? YCentroid - SquareSizeWin : 0;  this->YmaxWin = YCentroid + SquareSizeWin > 720 ? 720 : YCentroid + SquareSizeWin;
}

void Target::UpdatePos(float x, float y)
{
	this->XCentroid = x; this->YCentroid = y;//Position Update
	UpdateWin();
	switch (this->TargetStatus)
	{
	case Tracked:
		//TrackedCounter++;
		break;
	case TrackedLostPosition:
		TargetStatus = Tracked;
		this->SquareSizeWin = 50;
		break;
	case CandidateForTracking:
		this->TrackedCounter = TrackedCounter + 1;

		if (TrackedCounter > 5)
		{
			this->TrackedCounter = 0;
			this->NewTracked = true;
			this->TargetStatus = Tracked;
		}
		break;
	case LostPositionCandidate:
		this->TargetStatus = CandidateForTracking;
		this->TrackedCounter = 0;
		this->SquareSizeWin = 50;
		break;
	default:
		break;
	}
	UpdateWin();
}

bool Target::MatchWindow(float x, float y) // Return true if Point is in Window
{
	bool MatchingBool = false;
	MatchingBool = (x > XminWin) && (x < XmaxWin) ? true : false;
	MatchingBool &= (y > YminWin) && (y < YmaxWin) ? true : false;
	if (MatchingBool)
	{
		UpdatePos(x, y);
	}
	return MatchingBool;
}


void Target::IDAttribution(std::vector<bool> *IdTab)
{
	int k = 0;
	while (k<12)
	{
		if ((*IdTab)[k] == true )
		{
			TrackedTargetID = k;
			(*IdTab)[k] = false;
			break;
		}
		k++;
	}
}