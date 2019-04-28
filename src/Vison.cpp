#include "Client.h"
//#include "opencv2/opencv2.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Vison");
	ros::NodeHandle n;

	cv::VideoCapture cap;
	cap.open(0);
	if(!cap.isOpened()){
		ROS_INFO("Couldn't open Capture!'");
		return -1;
	}
	cv::namedWindow("opencv_ros_window", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("opencv_ros_window2", cv::WINDOW_AUTOSIZE);
	cv::Mat frame, out;
	while(true){
		cap >> frame;
		cv::GaussianBlur(frame, out, cv::Size(5,5), 3, 3);
		cv::imshow("opencv_ros_window", frame);
		cv::imshow("opencv_ros_window2", out);
		char c = (char)cv::waitKey(10);
		if(c == 27)
			break;
	}

	return 0;
}
