#pragma once
#if !defined FTRACKER
#define FTRACKER

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

namespace ft {
	struct Output {
		cv::Mat output_video;
		cv::FileStorage fs; //file containing the feature tracks
	};


	class FeatureTracker {

		cv::Mat gray;			// current gray-level image
		cv::Mat gray_prev;		// previous gray-level image
		std::vector<cv::Point2f> points[2]; // tracked features from 0->1
		std::vector<cv::Point2f> initial;   // initial position of tracked points
		std::vector<cv::Point2f> features;  // detected features
		int max_count;	  // maximum number of features to detect
		double qlevel;    // quality level for feature detection
		double minDist;   // minimum distance between two feature points
		std::vector<uchar> status; // status of tracked features
		std::vector<float> err;    // error in tracking

	public:

		FeatureTracker() : max_count(500), qlevel(0.01), minDist(10.) {}

		// tracking method
		//Output track(cv::Mat &frame, int & frame_width,
			//int & frame_height);
		ft::Output ft::FeatureTracker::track(cv::VideoCapture &capture);

		// feature point detection
		void detectFeaturePoints() {
			// detect the features
			cv::goodFeaturesToTrack(gray, // the image 
				features,   // the output detected features
				max_count,  // the maximum number of features 
				qlevel,     // quality level
				minDist);   // min distance between two features
		}

		// determine if new points should be added
		bool addNewPoints() {
			// if too few points
			return points[0].size() <= 10;
		}

		// determine which tracked point should be accepted
		bool acceptTrackedPoint(int i);

		// handle the currently tracked points
		cv::Mat handleTrackedPoints(cv::Mat &frame, cv::Mat &output);

		// write tracked points to an output text file
		void writeToFile(cv::FileStorage &fs, std::string name,
			std::vector<std::vector<cv::Point2f>> &points);
	};
}// end namespace

#endif


