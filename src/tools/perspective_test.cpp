#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/common.hpp"
#include "../src/image_preprocess.cpp"
#include "../include/capture.hpp"
#include "../include/stop_watch.hpp"
#include "./recognize/track_recognition.cpp"

using namespace std;
using namespace cv;

CaptureInterface captureInterface("/dev/video0");

int main(int argc, char *argv[])
{
    TrackRecognition track;          // 赛道识别
	ImagePreprocess imagePreprocess;            // 图像预处理类
    StopWatch watch;

    ipm.init(Size(COLSIMAGE, ROWSIMAGE),
             Size(COLSIMAGEIPM, ROWSIMAGEIPM)); // IPM逆透视变换初始化

    captureInterface.Start();

    watch.tic();
    while (1)
    {
        double frame_time = watch.toc();
        if(frame_time < 15)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }
        else
        {
            float detFPS = (float)1000.f / frame_time;
            std::cout << "run frame time: " << std::fixed << std::setprecision(3) << frame_time << "ms  ";
            std::cout << "FPS: " << (int)detFPS << std::endl;
            watch.tic();
        }

        Mat frame;
        frame = captureInterface.get_frame();

        Mat imgaeCorrect = frame;                                          // RGB
        Mat imageBinary = imagePreprocess.imageBinaryzation(imgaeCorrect); // Gray
        track.trackRecognition(imageBinary, 0); // 赛道线识别

        Mat remapImg = Mat::zeros(Size(COLSIMAGEIPM, ROWSIMAGEIPM), CV_8UC3); // 初始化图像
        // ipm.homography(frame, remapImg);
        //得到俯视域左、右边缘
        std::vector<POINT> perspectivePointsLeft;
        std::vector<POINT> perspectivePointsRight;
        for(int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            Point2d point2d = ipm.homography(Point2d(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x)); // 透视变换
            perspectivePointsLeft.push_back(POINT(point2d.y, point2d.x));
        }
        for(int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            Point2d point2d = ipm.homography(Point2d(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x)); // 透视变换
            perspectivePointsRight.push_back(POINT(point2d.y, point2d.x));
        }
        for (int i = 0; i < perspectivePointsLeft.size(); i++)
        {
            if(perspectivePointsLeft[i].x < 0 || perspectivePointsLeft[i].x > ROWSIMAGEIPM - 1 ||
                perspectivePointsLeft[i].y < 0 || perspectivePointsLeft[i].y > COLSIMAGEIPM - 1)
                continue;
            circle(remapImg, Point(perspectivePointsLeft[i].y, perspectivePointsLeft[i].x), 1,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < perspectivePointsRight.size(); i++)
        {
            if(perspectivePointsRight[i].x < 0 || perspectivePointsRight[i].x > ROWSIMAGEIPM - 1 ||
                perspectivePointsRight[i].y < 0 || perspectivePointsRight[i].y > COLSIMAGEIPM - 1)
                continue;
            circle(remapImg, Point(perspectivePointsRight[i].y, perspectivePointsRight[i].x), 1,
                   Scalar(0, 255, 255), -1); // 黄色点
        }
        imshow("frame", frame);
        imshow("remap", remapImg);
        
        waitKey(2);
    }
    captureInterface.Stop();
    return 0;
}
