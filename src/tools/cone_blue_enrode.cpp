#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/common.hpp"
#include "../src/recognize/track_recognition.cpp"

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
	TrackRecognition trackRecognition;

    std::string windowName = "frame";
    cv::namedWindow(windowName, WINDOW_NORMAL); // 图像名称
    cv::resizeWindow(windowName, COLSIMAGE, ROWSIMAGE);     // 分辨率

    /*注意：
      使用 0 和 /dev/video0 的分辨率不同：
        0           : opencv 内部的采集，可能是基于 V4L2, 分辨率：1280 * 960
        /dev/video0 : 基于Gstreamer ， 分辨率：640 * 480
    */
    VideoCapture capture("/dev/video0");
    if (!capture.isOpened())
    {
        std::cout << "can not open video device " << std::endl;
        return 1;
    }
    capture.set(cv::CAP_PROP_FRAME_WIDTH, COLSIMAGE);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, ROWSIMAGE);

    double rate = capture.get(CAP_PROP_FPS);
    double width = capture.get(CAP_PROP_FRAME_WIDTH);
    double height = capture.get(CAP_PROP_FRAME_HEIGHT);
    std::cout << "Camera Param: frame rate = " << rate << " width = " << width
              << " height = " << height << std::endl;

    while (1)
    {
        Mat frame;
        if (!capture.read(frame))
        {
            std::cout << "no video frame" << std::endl;
            continue;
        }

        std::vector<cv::Mat> channels;
        split(frame, channels);
        cv::Mat blueChannel = channels[0];

		Mat imageGray, imageBinary;

        Mat kernel_close = getStructuringElement(MORPH_RECT, Size(3, 3));//创建结构元
		Mat kernel_enrode = getStructuringElement(MORPH_ELLIPSE, Size(120, 60));
		morphologyEx(blueChannel, blueChannel, MORPH_CLOSE, kernel_close, Point(-1, -1));//闭运算
		morphologyEx(blueChannel, imageGray, MORPH_ERODE, kernel_enrode, Point(-1, -1));//腐蚀运算
		threshold(imageGray, imageBinary, 0, 255, THRESH_OTSU);

        trackRecognition.trackRecognition(imageBinary);
        trackRecognition.drawImage(frame);
        
        imshow("frame", frame);
        imshow("blueChannel", blueChannel);
        imshow("enrode", imageGray);
        imshow("enrode_binary", imageBinary);
        waitKey(5);
    }
    capture.release();
}
