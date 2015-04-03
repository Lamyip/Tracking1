#pragma once
#include <opencv2/core/core.hpp>
#include <vector>


class LightGobo
	{
	public:
		LightGobo(int x, int y);
		LightGobo();
		~LightGobo();
		void UpdatePos(float x, float y);
		bool MatchWindow(float x, float y);
		cv::Point2f GetCenter();
		void updateCounter();
		void resetCounter();
		int GetWinSize()
		{ 
			return WinSize; 
		};
		void UpdateWindow() // Enlarge window if no target detected
		{ WinSize = 100 +  counter * 50 ; };

	protected:
		cv::Point Center = cv::Point(0, 0); // Center Point
		cv::Point Origin = cv::Point(0, 0); // Origin
		int WinSize = 100; // Window Size
		int counter = 0; // Successive lost frames count
	};