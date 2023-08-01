
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>
#include "../include/common.hpp"	//公共类方法文件

using namespace std;
using namespace cv;

std::vector<cv::Rect> searchCones(cv::Mat img_rgb);

// 设置锥桶颜色的RGB范围（黄色）
cv::Scalar lowerThreshold(0, 100, 100);
cv::Scalar upperThreshold(100, 255, 255);

bool colorThresholdSelected = true;

int main(int argc, char const *argv[])
{
	
	Mat image;
	std::string indexCapture = "/dev/video0";
	VideoCapture capture("/dev/video0");
	if (!capture.isOpened())
	{
		std::cout << "can not open video device " << std::endl;
		return 1;
	}
    capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture.set(cv::CAP_PROP_FPS, 90);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, COLSIMAGE);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, ROWSIMAGE);
    capture.set(cv::CAP_PROP_ZOOM, 12);

	while (1)
	{
		// {
		// 	static auto preTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
		// 	auto startTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
		// 	float detFPS = (float)1000.f / (startTime - preTime);
		// 	cout << "run frame time : " << startTime - preTime << "ms  " << "FPS: " << (int)detFPS << endl;
		// 	preTime = startTime;
		// }
		
		Mat frame;
		if (!capture.read(frame))
		{
			std::cout << "no video frame" << std::endl;
			continue;
		}
		Mat image_cone = frame.clone();

		if(colorThresholdSelected)
		{
			std::vector<cv::Rect> coneRects = searchCones(image_cone);
			for(const auto& rect : coneRects)
			{
				cv::rectangle(image_cone, rect, cv::Scalar(0, 255, 0), 2);
				circle(image_cone, Point(rect.x + rect.width / 2, rect.y + rect.height / 2), 5, Scalar(200, 200, 200), -1);
			}
			cv::imshow("Result", image_cone);
			if(waitKey(5) == 13) break;
		}
		else
		{
			// 绘制方框在图像中心
            int centerX = image_cone.cols / 2;
            int centerY = image_cone.rows / 2;
            int boxSize = 60; // 方框的大小
            cv::Rect boxRect(centerX - boxSize / 2, centerY - boxSize / 2, boxSize, boxSize);

            // 在方框内提取颜色阈值参
            cv::Mat boxROI = image_cone(boxRect);
            cv::Scalar meanColor = cv::mean(boxROI);
            cv::rectangle(image_cone, boxRect, cv::Scalar(0, 255, 0), 2);

			// 绘制颜色阈值提示框在图像中心左下方
			int roiX = image_cone.cols / 2 - 20;
			int roiY = image_cone.rows / 2 + 20;
			cv::Rect roiRect(roiX- boxSize / 2, roiY - boxSize / 2, boxSize, boxSize);
			cv::rectangle(image_cone, roiRect, cv::Scalar(meanColor[0], meanColor[1], meanColor[2]), -1);

            cv::imshow("Select Color Threshold", image_cone);
			if(cv::waitKey(5) == 13)
			{
				// 设置锥桶颜色的RGB范围
				lowerThreshold = cv::Scalar(meanColor[0] - 10, meanColor[1] - 50, meanColor[2] - 50);
				upperThreshold = cv::Scalar(meanColor[0] + 10, meanColor[1] + 50, meanColor[2] + 50);
				colorThresholdSelected = true;
				cout << "Selected Thresholds: " << lowerThreshold << " - " << upperThreshold << endl;
			}
		}
	}
	return 0;
}

std::vector<cv::Rect> searchCones(cv::Mat img_rgb)
{
	std::vector<cv::Rect> coneRects;

	// 在RGB图像中根据颜色范围提取锥桶区域
	cv::Mat mask;
	cv::inRange(img_rgb, lowerThreshold, upperThreshold, mask);

	// 进行形态学操作，去除噪声并提取锥桶区域的轮廓
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	// 查找最大轮廓
	size_t maxContourIndex = 0;
	double maxContourArea = 0.0;
	for (size_t i = 0; i < contours.size(); ++i)
	{
		double contourArea = cv::contourArea(contours[i]);
		if (contourArea > maxContourArea)
		{
			maxContourArea = contourArea;
			maxContourIndex = i;
		}
	}

	// 绘制正方形框选中锥桶区域
	for (const auto& contour : contours)
	{
		cv::Rect boundingRect = cv::boundingRect(contour);
		coneRects.push_back(boundingRect);
	}	
	return coneRects;
}
