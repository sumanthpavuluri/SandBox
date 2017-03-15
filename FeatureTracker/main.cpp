
#include <iostream>
#include "FeatureTracker.h"

int main()
{
	std::string videofile;
	std::cout << "Enter video file path: ";
	std::cin >> videofile;
	// Open the video file
	cv::VideoCapture capture(videofile); //bike.avi, shark.mp4								   
	if (!capture.isOpened()) // check if video successfully opened
		return 1;
	ft::FeatureTracker tracks;
	ft::Output out;
	out = tracks.track(capture);

	return 0;
}
