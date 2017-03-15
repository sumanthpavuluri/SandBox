#include <iostream>
#include "FeatureTracker.h"

//ft::Output ft::FeatureTracker::track(cv::Mat &frame,
						//int & frame_width, int & frame_height){

ft::Output ft::FeatureTracker::track(cv::VideoCapture &capture) {
	Output out;
	cv::Mat frame;
	int frame_width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
	int frame_height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
	cv::Mat output;
	cv::VideoWriter video("output_tracks.avi", CV_FOURCC('M', 'J', 'P', 'G'),
		10, cv::Size(frame_width, frame_height), true);
	cv::namedWindow("Tracked Features");
	for (;;) {
		// convert to gray-level image
		//	read next frame if any
		if (!capture.read(frame))
			break;
		cv::cvtColor(frame, gray, CV_BGR2GRAY);
		frame.copyTo(output);

		// 1. if new feature points must be added
		if (addNewPoints())
		{
			// detect feature points
			detectFeaturePoints();
			// add the detected features to the currently tracked features
			points[0].insert(points[0].end(), features.begin(), features.end());
			initial.insert(initial.end(), features.begin(), features.end());
		}

		// for first image of the sequence
		if (gray_prev.empty())
			gray.copyTo(gray_prev);

		// 2. track features
		cv::calcOpticalFlowPyrLK(gray_prev, gray, // 2 consecutive images
			points[0], // input point position in first image
			points[1], // output point postion in the second image
			status,    // tracking success
			err);      // tracking error

					   // 2. loop over the tracked points to reject the undesirables
		int k = 0;
		for (int i = 0; i < points[1].size(); i++) {

			// do we keep this point?
			if (acceptTrackedPoint(i)) {

				// keep this point in vector
				initial[k] = initial[i];
				points[1][k++] = points[1][i];
			}
		}

		// eliminate unsuccesful points
		points[1].resize(k);
		initial.resize(k);

		// 3. handle the accepted tracked points
		out.output_video = handleTrackedPoints(frame, output);

		// 4. current points and image become previous ones
		std::swap(points[1], points[0]);
		cv::swap(gray_prev, gray);
		if (cv::waitKey(33) >= 0)
		{
			cv::imshow("Tracked Features", output);
		}

		//save output video
		video.write(out.output_video);

		////write tracks to a file
		//cv::FileStorage fs("Tracked_Points.yml", cv::FileStorage::WRITE);
		////out.fs = writeToFile(fs, "TF", p);
		//std::vector<std::vector<cv::Point2f>> p = { points[0], points[1] };
		//writeToFile(fs, "TF", p);
		//p.clear();
	}

	return out;
}

bool ft::FeatureTracker::acceptTrackedPoint(int i) {
	return status[i] &&
		// if point has moved
		(abs(points[0][i].x - points[1][i].x) +
		(abs(points[0][i].y - points[1][i].y)) > 2);
}

cv::Mat ft::FeatureTracker::handleTrackedPoints(cv::Mat &frame, cv::Mat &output) {

	// for all tracked points
	for (int i = 0; i < points[1].size(); i++) {

		// draw line and circle
		cv::line(output, initial[i], points[1][i], cv::Scalar(0, 0, 255));
		cv::circle(output, points[1][i], 3, cv::Scalar(0, 255, 0), -1);
	}
	return output;
}

void ft::FeatureTracker::writeToFile(cv::FileStorage &fs, std::string name, std::vector<std::vector<cv::Point2f>> &p)
{
	//cv::FileStorage fs("Tracked_Points.yml", cv::FileStorage::WRITE);
	fs << name;
	fs << "{";
	for (int i = 0; i < p.size(); i++)
	{
		fs << name + "_" + std::to_string(i);
		std::vector<cv::Point2f> tmp = p[i];
		fs << tmp;
	}
	fs << "}";
}