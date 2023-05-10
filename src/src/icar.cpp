#include <iostream>
#include <memory>
#include <string>
#include <ctime>
#include <signal.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "../include/uart.hpp"              //串口通信
#include "./recognize/track_recognition.cpp"//基础巡线
#include "./recognize/cross_recognition.cpp"//十字赛道
#include "./recognize/ring_recognize.cpp"
#include "image_preprocess.cpp"             //图像预处理
#include "controlcenter_cal.cpp"
#include "motion_controller.cpp"

using namespace cv;
using namespace std;

enum RoadType
{
  BaseHandle = 0, // 基础赛道处理
  RingHandle,     // 环岛赛道处理
  CrossHandle,    // 十字道路处理
  FreezoneHandle, // 泛行区处理
  GarageHandle,   // 车库处理
  GranaryHandle,  // 粮仓处理
  DepotHandle,    // 修车厂处理
  BridgeHandle,   // 坡道(桥)处理
  SlowzoneHandle, // 慢行区（动物出没）处理
  FarmlandHandle, // 农田区域处理
};

void callbackSignal(int signum);//系统退出回调函数
void displayWindowInit(void);//显示窗口初始化
void put_text2img(Mat &imgaeCorrect, RoadType roadType);

shared_ptr<Driver> driver = nullptr;
shared_ptr<VideoCapture> _cap = nullptr;

int main(int argc, char *argv[]) 
{
	RoadType roadType = RoadType::BaseHandle;       // 初始化赛道类型
	ControlCenterCal controlCenterCal;        		// 控制中心计算
	MotionController motionController;       		// 运动控制
	ImagePreprocess imagePreprocess;                // 图像预处理类
	TrackRecognition trackRecognition;              // 赛道识别
	RingRecognition ringRecognition;                // 环岛识别
    CrossroadRecognition crossroadRecognition;      // 十字道路处理
	uint16_t counterRunBegin = 1;              		// 智能车启动计数器：等待摄像头图像帧稳定
	uint16_t counterOutTrackA = 0;             		// 车辆冲出赛道计数器A
	uint16_t counterOutTrackB = 0;             		// 车辆冲出赛道计数器B


	// USB转串口的设备名为 / dev/ttyUSB0
	driver = std::make_shared<Driver>("/dev/ttyUSB0", BaudRate::BAUD_115200);
	if (driver == nullptr)
	{
		std::cout << "Create uart-driver error!" << std::endl;
		return -1;
	}
	// 串口初始化，打开串口设备及配置串口数据格式
	int ret = driver->open();
	if (ret != 0)
	{
		std::cout << "Uart open failed!" << std::endl;
		return -1;
	}

	_cap = make_shared<VideoCapture>();
	_cap->open("/dev/video0");
	if(_cap == nullptr)
	{
		cout << "Camera create failed!" << std::endl;
		return -1;
	}
	if(!_cap->isOpened())
	{
		cout << "Camera open failed!" << std::endl;
		return -1;
	}
    _cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    _cap->set(cv::CAP_PROP_FPS, 60);
    _cap->set(cv::CAP_PROP_FRAME_WIDTH, COLSIMAGE);
    _cap->set(cv::CAP_PROP_FRAME_HEIGHT, ROWSIMAGE);
	double rate = _cap->get(CAP_PROP_FPS);
    double width = _cap->get(CAP_PROP_FRAME_WIDTH);
    double height = _cap->get(CAP_PROP_FRAME_HEIGHT);
    std::cout << "Camera Param: frame rate = " << rate << " width = " << width
              << " height = " << height << std::endl;
	
	signal(SIGINT, callbackSignal);      // 程序退出信号

	imagePreprocess.imageCorrecteInit(); // 图像矫正参数初始化
	motionController.loadParams();       // 读取配置文件
	trackRecognition.rowCutUp = motionController.params.rowCutUp;
	trackRecognition.rowCutBottom = motionController.params.rowCutBottom;
	// garageRecognition.disGarageEntry = motionController.params.disGarageEntry;
	// if (motionController.params.GarageEnable) // 出入库使能
	// 	roadType = RoadType::GarageHandle;      // 初始赛道元素为出库
	if(motionController.params.CloseLoop)
	{
		driver->PID_init(motionController.params.Kp, motionController.params.Ki, motionController.params.Kd);
		cout << "-------- 速度闭环控制 -------" << endl;
		cout << "Kp = " << motionController.params.Kp << endl;
		cout << "Ki = " << motionController.params.Ki << endl;
		cout << "Kd = " << motionController.params.Kd << endl;
	}
	else
	{
		driver->PID_init(0, 0, 0);
		cout << "-------- 速度开环控制 -------" << endl;
	}
	if(motionController.params.Debug)
	{
		displayWindowInit();//调试模式初始化图像窗口
	}

    cout << "等待发车!!!" << endl;

    while (motionController.params.Debug/*!driver->receiveStartSignal()*/)
    {
		Mat frame;
		*_cap >> frame;
		imshow("imageTrack", frame);	
		char key = waitKey(5);
		if(key == 13)//回车
		{
			cout << "即将发车！！！" << endl;
			break;
		}
    }
	for (int i = 3; i > 0; i--) // 3秒后发车
    {
		cout << "------------- " << i << " -----------" << endl;
		driver->carControl(0, PWMSERVOMID); // 智能车停止运动|建立下位机通信
		waitKey(1000);
    }
    cout << "--------- System start!!! -------" << endl;

	while(1)
	{
		// 处理帧时长监测 速度监测
		{
			static auto preTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
			auto startTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
			float detFPS = (float)1000.f / (startTime - preTime);
			cout << "run frame time : " << startTime - preTime << "ms  " << "FPS: " << (int)detFPS << endl;
			preTime = startTime;
		}


		/*图像预处理*/
		Mat frame;
		*_cap >> frame;
		Mat imgaeCorrect = imagePreprocess.imageCorrection(frame);         // RGB
		Mat imageBinary = imagePreprocess.imageBinaryzation(imgaeCorrect); // Gray

		/*基础赛道识别*/
		trackRecognition.trackRecognition(imageBinary); // 赛道线识别
		Mat imageTrack = imgaeCorrect.clone();  // RGB
		// trackRecognition.drawImage(imageTrack); // 图像显示赛道线识别结果

		/*环岛识别与处理*/
		if (motionController.params.RingEnable) // 赛道元素是否使能
		{
			if (roadType == RoadType::RingHandle || roadType == RoadType::BaseHandle)
			{
				if (ringRecognition.ringRecognition(trackRecognition, imageBinary))
				{
					if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
						driver->buzzerSound(1);             // OK
					roadType = RoadType::RingHandle;
					
					if (motionController.params.Debug)
					{
						Mat imageRing = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
						ringRecognition.drawImage(trackRecognition, imageRing);
						imshow("imageRecognition", imageRing);
					}
				}
				else
					roadType = RoadType::BaseHandle;
			}
		}


		// /*十字赛道*/
		if (motionController.params.CrossEnable) // 赛道元素是否使能
		{
			if (roadType == RoadType::CrossHandle || roadType == RoadType::BaseHandle)
			{
				if (crossroadRecognition.crossroadRecognition(trackRecognition))
				{
					roadType = RoadType::CrossHandle;
					if(motionController.params.Debug)
					{
						Mat imageCross = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
						crossroadRecognition.drawImage(trackRecognition, imageCross);
						imshow("imageRecognition", imageCross);
					}
				}
				else
				{
					roadType = RoadType::BaseHandle;
				}
			}
		}

		/*控制中心计算*/
		if (trackRecognition.pointsEdgeLeft.size() < 60 &&
			trackRecognition.pointsEdgeRight.size() < 60 &&
			motionController.params.StopEnable &&
			roadType != RoadType::BridgeHandle &&
			roadType != RoadType::GranaryHandle &&
			roadType != RoadType::DepotHandle &&
			roadType != RoadType::FarmlandHandle) // 防止车辆冲出赛道
		{
			counterOutTrackA++;
			counterOutTrackB = 0;
			if (counterOutTrackA > 20)
				callbackSignal(0);
		}
		else
		{
			counterOutTrackB++;
			if (counterOutTrackB > 50)
			{
				counterOutTrackA = 0;
				counterOutTrackB = 50;
			}
		}
		controlCenterCal.controlCenterCal(trackRecognition); // 根据赛道边缘信息拟合运动控制中心

		/*智能车运动控制 通讯*/
		if(counterRunBegin > 30)
		{
			// 智能车方向控制
			motionController.pdController(controlCenterCal.controlCenter); // PD控制器姿态控制

			// 智能车速度控制
			motionController.motorSpeed = motionController.params.speedLow; // 匀速控制
			// motionController.speedController(true, 0, controlCenterCal); // 变加速控制

			// 串口通信，姿态与速度控制
			driver->carControl(motionController.motorSpeed, motionController.servoPwm);
		}
		else
		{
			counterRunBegin++;
		}


		/*图像显示*/
		if(motionController.params.Debug)
		{
			controlCenterCal.drawImage(trackRecognition, imageTrack);// 绘制中线
			trackRecognition.drawImage(imageTrack); // 图像显示赛道线识别结果
			imshow("imageTrack", imageTrack);	
		}

		char key = waitKey(1);
		if(key == 13)//回车
		{
			callbackSignal(0);
		}

	}

	_cap->release();
	destroyAllWindows();
	cout << "Camera closed" << endl;
    return 0;
}

/**
 * @brief 系统信号回调函数：系统退出
 *
 * @param signum 信号量
 */
void callbackSignal(int signum)
{
  driver->carControl(0, PWMSERVOMID); // 智能车停止运动
  cout << "====System Exit!!!  -->  CarStopping! " << signum << endl;
  exit(signum);
}

/**
 * @brief OpenCV图像显示窗口初始化（详细参数/Debug模式）
 *
 */
void displayWindowInit(void)
{
  //[1] 二值化图像：Gray
  string windowName = "imageTrack";
  cv::namedWindow(windowName, WINDOW_NORMAL); // 图像名称
  cv::resizeWindow(windowName, COLSIMAGE, ROWSIMAGE);     // 分辨率
  cv::moveWindow(windowName, 10, 10);         // 布局位置

  //[2] 赛道边缘图像：RGB
  windowName = "imageRecognition";
  cv::namedWindow(windowName, WINDOW_NORMAL); // 图像名称
  cv::resizeWindow(windowName, COLSIMAGE, ROWSIMAGE);     // 分辨率
  cv::moveWindow(windowName, 10, 700);        // 布局位置

//   //[3] 原始图像/矫正后：RGB
//   windowName = "imageControl";
//   cv::namedWindow(windowName, WINDOW_NORMAL); // 图像名称
//   cv::resizeWindow(windowName, COLSIMAGE, ROWSIMAGE);     // 分辨率
//   cv::moveWindow(windowName, 350, 20);        // 布局位置
}

void put_text2img(Mat &imgaeCorrect, RoadType roadType)
{
	switch (roadType)
    {
	case RoadType::BaseHandle: // 基础赛道处理
		putText(imgaeCorrect, "[1] Track", Point(10, 20),
				cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1,
				CV_AA); // 显示赛道识别类型
		break;
	case RoadType::RingHandle: // 环岛赛道处理
		putText(imgaeCorrect, "[2] Ring", Point(10, 20),
				cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
				CV_AA); // 显示赛道识别类型
		break;
	case RoadType::CrossHandle: // 十字道路处理
		putText(imgaeCorrect, "[3] Cross", Point(10, 20),
				cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
				CV_AA); // 显示赛道识别类型
		break;
	case RoadType::FreezoneHandle: // 泛行区处理
		putText(imgaeCorrect, "[4] Freezone", Point(10, 20),
				cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
				CV_AA); // 显示赛道识别类型
		break;
	case RoadType::GarageHandle: // 车库处理
		putText(imgaeCorrect, "[5] Garage", Point(10, 20),
				cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
				CV_AA); // 显示赛道识别类型
		break;
	case RoadType::GranaryHandle: // 粮仓处理
		putText(imgaeCorrect, "[6] Granary", Point(10, 20),
				cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
				CV_AA); // 显示赛道识别类型
		break;
	case RoadType::DepotHandle: // 修车厂处理
		putText(imgaeCorrect, "[7] Depot", Point(10, 20),
				cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
				CV_AA); // 显示赛道识别类型
		break;
	case RoadType::BridgeHandle: // 坡道(桥)处理
		putText(imgaeCorrect, "[8] Bridge", Point(10, 20),
				cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
				CV_AA); // 显示赛道识别类型
		break;
	case RoadType::SlowzoneHandle: // 慢行区（动物出没）处理
		putText(imgaeCorrect, "[9] Slowzone", Point(10, 20),
				cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
				CV_AA); // 显示赛道识别类型
		break;
	case RoadType::FarmlandHandle: // 农田区域处理
		putText(imgaeCorrect, "[10] Farmland", Point(10, 20),
				cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
				CV_AA); // 显示赛道识别类型
		break;
	}
}