#pragma once
#include <boost\thread.hpp>
#include <vector>
#include <opencv2\core\core.hpp>


class Parameters
{
	boost::mutex mtx;
public:
	void lock(){
		mtx.lock();
	}
	void unlock()
	{
		mtx.unlock();
	}
	Parameters();
	~Parameters();
};

class Command : public Parameters
{	
public:
	bool activation = false; // Medialon Command for on/off
	bool ExpandTrackingWindowSize = false; //Medialon command for expanding Tracking Window Size if Target loss
	bool ActivateLight[12]; // Individual light activation
};

class PointVector : public Parameters // Position vector between image processing threads
{
protected :
	std::vector<cv::Point2f> SharedPoints = std::vector<cv::Point2f>(12, cv::Point2f(0.0, 0.0)); // Position vector

public:
	void WriteSharedPoints(std::vector<cv::Point2f> &a);  //rewrite vector
	std::vector<cv::Point2f> ReadSharedPoints(){ return SharedPoints; }; //Read Vector
	
};