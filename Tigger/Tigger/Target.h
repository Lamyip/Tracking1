#pragma once
#include <opencv2/core/core.hpp>
#include <vector>

enum TargetState{
	CandidateForTracking, LostPositionCandidate, Tracked, TrackedLostPosition, Lost
};

class Target
{
public:
	void UpdatePos(float x, float y);
	bool MatchWindow(float x, float y);
	void UpdateWin();
	void CheckTrack(bool b_input);
	void DisableTrack();
	void LostPositionStatusUpdate();
	bool EventNewTrack(){ bool tmp= false; if (NewTracked){this->NewTracked = false;tmp = true;}
	return tmp; };
	int GetTargetID(){ return TrackedTargetID; };
	void IDAttribution(bool * IdTab);
	cv::Point GetCenter();
	bool Target::GetTrackedBool(){ bool tmp = false; if (TargetStatus == Tracked)tmp = true; return tmp; };
	cv::Scalar GetColor();
	bool GetTrack(){ return this->PositionTracked; };
	cv::Rect GetWin(){ return cv::Rect(XminWin, YminWin, 2 * SquareSizeWin, 2 * SquareSizeWin); }
	TargetState GetStatus(){ return TargetStatus; };
	int GetCount(){ return TrackedCounter; }
	void IDAttribution(std::vector<bool> *IdTab);
	Target(float x, float y);
	~Target();

protected:
	TargetState TargetStatus;
	cv::Scalar TargetColor;
	float XCentroid, YCentroid, XminWin, XmaxWin, YminWin, YmaxWin;
	int TrackedCounter = 0, SquareSizeWin = 50;
	bool PositionTracked = true;
	bool NewTracked = false;
	int TrackedTargetID = 0;
};

