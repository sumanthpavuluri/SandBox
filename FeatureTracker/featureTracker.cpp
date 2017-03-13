
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

void writeToFile(cv::FileStorage &fs, std::string name,
	std::vector<std::vector<cv::Point2f>> &points);

int main()
{
	// Open the video file
	cv::VideoCapture capture("../shark.mp4"); //bike.avi, shark.mp4
												   // check if video successfully opened
	if (!capture.isOpened())
		return 1;
	cv::Mat frame; // current video frame
	cv::namedWindow("Tracked Features");
	cv::Mat gray, gray_prev;
	std::vector<cv::Point2f> points[2]; //points from previous and current frame
	std::vector<cv::Point2f> features;
	const int MAX_COUNT = 500; //max number of corners to return
	cv::Size winSize(10, 10);
	cv::TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS,
		20, 0.03);
	bool firstframe = true;
	int frame_width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
	int frame_height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
	cv::VideoWriter video("shark_ft1.avi", CV_FOURCC('M', 'J', 'P', 'G'),
		10, cv::Size(frame_width, frame_height), true);

	cv::FileStorage fs("Tracked points.yml", cv::FileStorage::WRITE);
	// for all frames in video
	for (;;) {

		// read next frame if any
		if (!capture.read(frame))
			break;

		cv::cvtColor(frame, gray, CV_BGR2GRAY);
		if (firstframe)
		{
			cv::goodFeaturesToTrack(gray, features, MAX_COUNT, 0.01, 10, cv::Mat(), 3, 0, 0.04);
			cv::cornerSubPix(gray, features, winSize, cv::Size(-1, -1), termcrit);
			points[0].insert(points[0].end(), features.begin(), features.end());
			firstframe = false;
		}

		{
			std::vector<uchar> status;
			std::vector<float> err;
			if (gray_prev.empty())
				gray.copyTo(gray_prev);
			cv::calcOpticalFlowPyrLK(gray_prev, gray, points[0], points[1], status, err, winSize, 3, termcrit, 0);
			size_t i, k;
			for (i = k = 0; i < points[1].size(); i++)
			{
				if (!status[i])
					continue;
				if ((abs(points[0][i].x - points[1][i].x) + (abs(points[0][i].y - points[1][i].y)) < 2))
					continue;

				cv::line(frame, points[0][i], points[1][i], cv::Scalar(0, 0, 255));
				points[1][k++] = points[1][i];
				cv::circle(frame, points[1][i], 3, cv::Scalar(0, 255, 0), -1, 8);
			}
			points[1].resize(k);
		}
		if (points[1].size() <= 10)
			firstframe = true;
		std::swap(points[1], points[0]);
		cv::swap(gray_prev, gray);

		if (cv::waitKey(33) >= 0)
		{
			cv::imshow("Tracked Features", frame);
		}

		//write frame to output file
		{
			video.write(frame);
		}
		std::vector<std::vector<cv::Point2f>> p = { points[0], points[1] };

		//write to a file
		writeToFile(fs, "TF", p);
		//cv::write(fs, "TF", p);
		p.clear();
	}

	//close the yml file
	fs.release();
	// Close the video file
	capture.release();

	cv::waitKey();


}


void writeToFile(cv::FileStorage &fs, std::string name, std::vector<std::vector<cv::Point2f>> &p)
{
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