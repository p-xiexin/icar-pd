#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2022; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-FZ3B),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file image_preprocess.cpp
 * @author Leo
 * @brief 图像预处理：RGB转灰度图，图像二值化
 * @version 0.1
 * @date 2022-02-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/common.hpp"

using namespace cv;
using namespace std;

/**
**[1] 读取视频
**[2] 图像二值化
*/

class ImagePreprocess
{
public:



	
	Mat yellowToBinary(Mat &frame,Mat &bin) {  
		Mat hsv, mask, imageBinary , openingResult;  
	
		// 转换BGR到HSV  
		cvtColor(frame, hsv, COLOR_BGR2HSV);  
	
		// 定义黄色在HSV中的范围  
		// 注意：HSV值范围通常是H: 0-179, S: 0-255, V: 0-255  
		// 黄色范围的选择可能需要根据实际情况进行调整  
		Scalar lowerb = Scalar(15, 100, 100); // 黄色下限  
		Scalar upperb = Scalar(35, 255, 255); // 黄色上限  
	
		// 创建黄色区域的掩码  
		inRange(hsv, lowerb, upperb, mask);  
		
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_DILATE, cv::Size(30, 30));
        cv::morphologyEx(mask, mask, cv::MORPH_ERODE, kernel);

		// 将掩码之外的区域置零（如果需要保持原图的其他部分不变）  
		// 如果只需要黄色区域，可以跳过这一步  
		Mat yellowRegion;  
		frame.copyTo(yellowRegion, mask);  
		
		// 将黄色区域转换为灰度图（如果需要的话）  
		// 这里我们直接对掩码进行二值化，所以不需要转换为灰度图  
	
		// 对黄色区域进行二值化  
		// 因为掩码已经是二值图像（0和255），所以这一步可能只是为了明确目的  
		threshold(mask, imageBinary, 128, 255, THRESH_BINARY_INV); // 假设黄色在掩码中是白色，我们将其反转为黑色  
		
		// 如果需要，可以将二值化后的图像应用到原图上  
		//frame.copyTo(frame, Scalar::all(0), mask); // 这将非黄色区域置为黑色  
		bin.setTo(Scalar(0, 0, 0), mask); // OpenCV 4.x及以上版本使用setTo  


			//cv::circle(contoursImage, p, 10, cv::Scalar(0, 0, 0), -1); // 绿色表示中心点  
		// 定义结构元素（这里使用5x5的矩形）  
		Mat element = getStructuringElement(MORPH_DILATE, Size(5, 5));  
	
		// 对二值图像执行开运算  
		morphologyEx(imageBinary, openingResult, MORPH_OPEN, element); 
			
		// 返回二值化后的图像（只包含黄色区域的二值化）  
		return openingResult;  
	}


	void addd( Mat &cone , Mat &mapp,bool &ifcone)
	{


		// 查找轮廓  
		std::vector<std::vector<cv::Point>> contours;  
		std::vector<cv::Vec4i> hierarchy;  
		cv::findContours(cone, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);  
	
		// 创建一个与原图像同样大小的彩色图像用于绘制轮廓  
		cv::Mat contoursImage = cv::Mat::zeros(mapp.size(), CV_8UC3);  

		
		// 定义最小和最大面积  
		double minArea = 50; // 锥桶轮廓的最小面积  
		double maxArea = 2500; // 锥桶轮廓的最大面积  
		cv::Point center;  
	// cout<<contours.size()<<endl;
		// 遍历轮廓  
		for (size_t i = 0; i < contours.size(); i++) {  
			double area = cv::contourArea(contours[i]); 
			cout<<area<<endl; 
			if (area < minArea || area > maxArea) {  
				continue;
			}
				//  cout<<area<<endl;

				// 计算轮廓的矩  
				cv::Moments m = cv::moments(contours[i]);  
	
				// 计算中心点（注意：如果m00为0，则轮廓太小或为空，不能计算中心点）  
				if (m.m00 != 0.0) {  
					center.x = static_cast<int>(m.m10 / m.m00 + 0.5);  
					center.y = static_cast<int>(m.m01 / m.m00 + 0.5);  
					cout <<center.x<<" "<<center.y<<endl;
					// // 绘制轮廓  
					// cv::Scalar color = cv::Scalar(rand() & 255, rand() & 255, rand() & 255); // 随机颜色  
					// cv::drawContours(contoursImage, contours, static_cast<int>(i), color, 2, cv::LINE_8, hierarchy, 0, cv::Point());  
					// // 绘制中心点（使用小圆圈）  
					cv::circle(contoursImage, center, 5, cv::Scalar(255, 255, 0), -1); // 绿色表示中心点  

				} 
			// 初始化边界变量  
			int left_first_change = -1, left_second_change = -1;  
			int right_first_change = contoursImage.cols, right_second_change = contoursImage.cols;  
			int Llength,Rlength;
			// 3. 从中心点分别向左、右遍历，找到第一个和第二个黑白变化的位置  
			bool is_left_inside_block = false; // 标记是否处于色块内部（从左到右）  
			// 使用ptr<>()方法访问像素值（通常用于遍历图像）  
			const uchar* ptr = mapp.ptr<uchar>(center.y);  
			Point Lp;

			for (int col = center.x; col >= 0; --col) {  ///LEFT
				uchar pixelValueWithPtr = ptr[col]; 
				//cout<< int(pixelValueWithPtr);
				if (pixelValueWithPtr == 255 && !is_left_inside_block) {  
					// 第一个变化点（进入色块）  
					left_first_change = col;  
					is_left_inside_block = true;  
				} else if (pixelValueWithPtr == 0 && is_left_inside_block) {  
					// 第二个变化点（离开色块）  
					left_second_change = col;  
					Lp.x=0;Lp.y=center.y;
					//cout<<p.x<<" "<<p.y<<","<<mapp.at<uchar>(center.x, col)<<","<<mapp.at<uchar>(center.x, col+1)<<endl;
					cv::circle(contoursImage, Lp, 3, cv::Scalar(0, 255, 0), -1); // 绿色表示中心点  
					Llength=center.x-col;
					break; // 不再需要继续向左遍历  
					}  
				}
			if(left_first_change == -1 || left_second_change == -1)
				continue;
			// 3. 从中心点分别向左、右遍历，找到第一个和第二个黑白变化的位置  
			bool is_right_inside_block = false; // 标记是否处于色块内部（从左到右）  
			Point Rp;

			for (int col = center.x; col < mapp.cols; ++col) {  ///RIGHT
			uchar pixelValueWithPtr = ptr[col]; 
			//cout<< int(pixelValueWithPtr);
			if (pixelValueWithPtr == 255 && !is_right_inside_block) {  
				// 第一个变化点（进入色块）  
				right_first_change = col;  
				is_right_inside_block = true;  
			} else if (pixelValueWithPtr == 0 && is_right_inside_block) {  
				// 第二个变化点（离开色块）  
				right_second_change = col;  
				Rp.x=mapp.cols;Rp.y=center.y;
				//cout<<Rp.x<<" "<<p.y<<","<<mapp.at<uchar>(center.x, col)<<","<<mapp.at<uchar>(center.x, col+1)<<endl;
				//cv::circle(contoursImage, p, 3, cv::Scalar(0, 255, 0), -1); // 绿色表示中心点  
				Rlength = col-center.x;
				break; // 不再需要继续向左遍历  
				}  
			}

			/**********比较长度+补线**********/
			std::vector<cv::Point> contour = contours[i];  

			if(Llength<Rlength)
			{
				// // 初始化y坐标的最大值和最小值  
				// int y_max = std::numeric_limits<int>::min();  
				// int y_min = std::numeric_limits<int>::max();  
				
				// // 遍历轮廓中的所有点  
				// for (size_t j = 0; j < contour.size(); j++) {  
				// 	cv::line(mapp, contour[j], Lp, 0, 5); // 最后一个参数是线条的粗细，这里设置为2  
				// 	//cout<<"successful111!!";

				// }  
				cv::line(mapp, Lp, center, cv::Scalar(0), 10); // 最后一个参数是线条的粗细，这里设置为2  
				ifcone = true;
				// 计算高度（确保y_min不大于y_max）  
				// int height = (y_max >= y_min) ? (y_max - y_min) : 0;  
				// cv::line(mapp, center, Lp, 0, height); // 最后一个参数是线条的粗细，这里设置为2  


			}
			else{
				// 初始化y坐标的最大值和最小值  
				int y_max = std::numeric_limits<int>::min();  
				int y_min = std::numeric_limits<int>::max();  
				Mat temimg=mapp.clone();
				cv::line(mapp, Rp, center, cv::Scalar(0), 10); // 最后一个参数是线条的粗细，这里设置为2  
				ifcone = true;
				// // 遍历轮廓中的所有点  
				// for (size_t j = 0; j < contour.size(); j++) {  
				// 	Point a,b;a.x=0;a.y=0;b.x=100;b.y=100;;
				// 	cv::line(temimg, Rp, contour[j], cv::Scalar(0), 100); // 最后一个参数是线条的粗细，这里设置为2  
				// 	//cout<<"successful!!";
				// 	cout<<contour[j]<<","<<Rp<<endl;

				// }  
					string binarypath ="../src/tools/after/"+ to_string(i) +"binary.jpg";
					cv::imwrite(binarypath, mapp);

				// 计算高度（确保y_min不大于y_max）  
				// int height = (y_max >= y_min) ? (y_max - y_min) : 0;  
				// cv::line(mapp, center, Rp, 0, height); // 最后一个参数是线条的粗细，这里设置为2  

			}

		}  
		string binarypath ="../src/tools/after/123binary.jpg";
		cv::imwrite(binarypath, mapp);

		
	}





	/**
	 * @brief 图像二值化
	 *
	 * @param frame	输入原始帧
	 * @return Mat	二值化图像
	 */
	Mat imageBinaryzation(Mat &frame, bool blue = true)
	{
		Mat imageGray, imageBinary;
		Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));//创建结构元

		if(blue)
		{
			std::vector<cv::Mat> channels;
			split(frame, channels);
			cv::Mat redChannel = channels[2];
			morphologyEx(redChannel, imageGray, MORPH_CLOSE, kernel, Point(-1, -1));//闭运算
		}
		else
		{
			cvtColor(frame, imageGray, COLOR_BGR2GRAY); // RGB转灰度图
			morphologyEx(imageGray, imageGray, MORPH_CLOSE, kernel, Point(-1, -1));//闭运算
		}


		threshold(imageGray, imageBinary, 0, 255, THRESH_OTSU); // OTSU二值化方法

		return imageBinary;
	}

	/**
	 * @brief 图像矫正参数初始化
	 *
	 */
	void imageCorrecteInit(void)
	{
		// 读取xml中的相机标定参数
		cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 摄像机内参矩阵
		distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));	// 相机的畸变矩阵
		FileStorage file;
		if (file.open("../res/calibration/valid/calibration.xml", FileStorage::READ)) // 读取本地保存的标定文件
		{
			file["cameraMatrix"] >> cameraMatrix;
			file["distCoeffs"] >> distCoeffs;
			cout << "相机矫正参数初始化成功!" << endl;
			correctionEnable = true;
		}
		else
		{
			cout << "打开相机矫正参数失败!!!" << endl;
			correctionEnable = false;
		}
	}

	/**
	 * @brief 矫正图像
	 *
	 * @param imagesPath 图像路径
	 */
	Mat imageCorrection(Mat &image)
	{
		if (correctionEnable)
		{
			Size sizeImage; // 图像的尺寸
			sizeImage.width = image.cols;
			sizeImage.height = image.rows;

			Mat mapx = Mat(sizeImage, CV_32FC1);	// 经过矫正后的X坐标重映射参数
			Mat mapy = Mat(sizeImage, CV_32FC1);	// 经过矫正后的Y坐标重映射参数
			Mat rotMatrix = Mat::eye(3, 3, CV_32F); // 内参矩阵与畸变矩阵之间的旋转矩阵

			// 采用initUndistortRectifyMap+remap进行图像矫正
			initUndistortRectifyMap(cameraMatrix, distCoeffs, rotMatrix, cameraMatrix, sizeImage, CV_32FC1, mapx, mapy);
			Mat imageCorrect = image.clone();
			remap(image, imageCorrect, mapx, mapy, INTER_LINEAR);

			// 采用undistort进行图像矫正
			//  undistort(image, imageCorrect, cameraMatrix, distCoeffs);

			return imageCorrect;
		}
		else
		{
			return image;
		}
	}

private:
	bool correctionEnable = false; // 图像矫正使能：初始化完成
	Mat cameraMatrix;			   // 摄像机内参矩阵
	Mat distCoeffs;				   // 相机的畸变矩阵
};
