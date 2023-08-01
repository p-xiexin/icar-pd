/**
 * @file granary_detect.cpp
 * @author pxx
 * @brief 粮仓区域检测与路径规划
 * @version 0.1
 * @date 2023-07-24
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/predictor.hpp"
#include "../recognize/track_recognition.cpp"

using namespace cv;
using namespace std;

class GranaryDetection
{
public:
    GranaryDetection()
    {
        loadParams();
    }
    ~GranaryDetection() {}

    /**
     * @brief 粮仓初始化
     *
     */
    void reset()
    {
        granaryStep = GranaryStep::None;
        granaryType = GranaryType::ExitNone;
        lastPointsEdgeLeft.clear();
        lastPointsEdgeRight.clear();
        counterSession = 0;
        counterRec = 0;
    	counterExit = 0;
        exitTwoEnable = false;
        numGranary = 0;
    }

    /**
     * @brief 粮仓检测确认 运行于AI线程
     *
     * @param predict AI 检测结果
     */
    void granaryCheck(std::vector<PredictResult> predict)
    {
        if(counterShield < 30)
        {
            counterShield++;
            return;
        }

        if(granaryType == GranaryType::ExitNone && granaryStep == GranaryStep::None)
        {
            std::vector<POINT> granarys = searchGranary(predict);
            if(granarys.size() > 0)
                counterRec++;
            if(granarys.size() > 1)
                numGranary++;

            if(counterRec)
            {
                counterSession++;
                if(counterRec > params.GranaryCheck && counterSession < params.GranaryCheck * 2 + 1)
                {
                    counterRec = 0;
                    counterSession = 0;
                    granaryStep = GranaryStep::Enable;
                    if(numGranary > params.GranaryCheck / 2)
                        granaryType = GranaryType::ExitTwo;
                    else
                        granaryType = GranaryType::ExitOne;
                }
                else if(counterSession >= params.GranaryCheck * 2 + 1)
                {
                    counterRec = 0;
                    counterSession = 0;
                }
            }
        }
    }

    /**
     * @brief 粮仓区路径规划 运行于主线程
     *
     * @param track
     * @param frame
     */
    bool granaryDetection(TrackRecognition& track, cv::Mat img_rgb)
    {
        _pointNearCone = POINT(0, 0);
        pointEdgeDet.clear();
        _coneRects.clear();

        switch(granaryStep)
        {
        case GranaryStep::Enable:
        {
			// counterExit++;
			// if (counterExit > 100) {
			// 	reset();
			// 	return false;
			// }

            _coneRects = detectCones(img_rgb);
			searchCones(_coneRects, track.rowCutUp);
            _pointNearCone = searchNearestCone(track.pointsEdgeLeft, pointEdgeDet); // 搜索左下锥桶

			if (_pointNearCone.x > params.ServoRow && _pointNearCone.x < params.ServoRow + ROWSIMAGE / 3
				&& _pointNearCone.y != 0) // 当车辆开始靠近右边锥桶：准备入库
            {
                counterRec++;
				if (counterRec > 2)
				{
					// granaryStep = GranaryStep::Enter; // 进站使能
					counterRec = 0;
					counterSession = 0;
				}
            }
            // else if (_pointNearCone.x > 0.1 && _pointNearCone.x < ROWSIMAGE * 0.4)
            // {
            //     slowDown = true; // 进站减速
            // }

            break;
        }
        case GranaryStep::Enter:
        {
			if (track.pointsEdgeLeft.size() > ROWSIMAGE / 2) // 第一阶段：当赛道边缘存在时
            {

            }
			else // 第二阶段：检查右下锥桶坐标满足巡航条件
            {

            }
            break;
        }
        case GranaryStep::Exit:
        {
            
            break;
        }
        }

        if(granaryStep == GranaryStep::None)
            return false;
        else
            return true;
    }

    /**
     * @brief 识别结果图像绘制
     *
     */
    void drawImage(TrackRecognition track, Mat &image)
    {
        // 绘制锥桶坐标
        //for (int i = 0; i < pointEdgeDet.size(); i++)
        //{
        //   circle(image, Point(pointEdgeDet[i].y, pointEdgeDet[i].x), 2, Scalar(92, 92, 205), -1); // 锥桶坐标：红色
        //}
		// for (int i = 0; i < pointEdgeDet.size(); i++)
		// {
        //     putText(image, to_string(i+1), Point(pointEdgeDet[i].y, pointEdgeDet[i].x), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1, CV_AA);
		// }


        // 赛道边缘
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(image, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1,
                   Scalar(0, 255, 255), -1); // 黄色点
        }

        // 显示粮仓状态
        string state = "None";
        switch (granaryStep)
        {
        case GranaryStep::Enable:
            state = "GranaryEnable";
            break;
        case GranaryStep::Enter:
            state = "GranaryEnter";
            break;
        case GranaryStep::Cruise:
            state = "GranaryCruise";
            break;
        case GranaryStep::Exit:
            state = "GranaryExit";
            break;
        }
        if(granaryType == GranaryType::ExitOne)
            state += "-one";
        else if(granaryType == GranaryType::ExitTwo)
            state += "-two";

        putText(image, state, Point(COLSIMAGE / 2 - 40, 20), cv::FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(0, 255, 0), 1, CV_AA);
        putText(image, to_string(granaryStep), Point(COLSIMAGE / 2, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1, CV_AA);

        if (_pointNearCone.x > 0)
            circle(image, Point(_pointNearCone.y, _pointNearCone.x), 4, Scalar(226, 43, 138), -1);//紫色
    }

private:
    /**
     * @brief 从AI检测结果中检索谷仓标志
     *
     * @param predict
     * @return vector<POINT>
     */
    std::vector<POINT> searchGranary(std::vector<PredictResult> predict)
    {
        vector<POINT> granarys;
        for (int i = 0; i < predict.size(); i++)
        {
            if (predict[i].label == LABEL_GRANARY)
            {
                granarys.push_back(POINT(predict[i].y + predict[i].height / 2, predict[i].x + predict[i].width / 2));
            }
        }

        return granarys;
    }

	/**
	 * @brief 从AI检测结果中检索锥桶坐标集合
	 *
	 * @param predict AI检测结果
	 * @return vector<POINT>
	 */
	void searchCones(vector<PredictResult> predict)
	{
		pointEdgeDet.clear();
		for (int i = 0; i < predict.size(); i++)
		{
			if (predict[i].label == LABEL_CONE) // 锥桶检测
			{
				pointEdgeDet.push_back(POINT(predict[i].y + predict[i].height / 2,
											 predict[i].x + predict[i].width / 2));
			}
		}
	}

	/**
	 * @brief 从视觉结果中检索锥桶坐标集合
	 *
	 * @param predict 检测结果
	 * @param rowCutUp 滤除掉图片最上方部分的色块
	 * @return vector<POINT>
	 */
	void searchCones(vector<Rect> predict, uint16_t rowCutUp = 0)
	{
		pointEdgeDet.clear();
		for (int i = 0; i < predict.size(); i++)
		{
			if(predict[i].y + predict[i].height / 2 > rowCutUp)
				pointEdgeDet.push_back(POINT(predict[i].y + predict[i].height / 2,
												predict[i].x + predict[i].width / 2));
		}
	}

    /**
     * @brief 加载配置参数Json
     */
    void loadParams() 
    {
        string jsonPath = "../src/config/granary.json";
        std::ifstream config_is(jsonPath);
        if (!config_is.good()) 
        {
            std::cout << "Error: Params file path:[" << jsonPath << "] not find .\n";
            exit(-1);
        }

        nlohmann::json js_value;
        config_is >> js_value;

        try 
        {
            params = js_value.get<Params>();
        }
        catch (const nlohmann::detail::exception &e) 
        {
            std::cerr << "Json Params Parse failed :" << e.what() << '\n';
            exit(-1);
        }
    }

	/**
	 * @brief 搜索距离赛道边缘最近的锥桶坐标（左下 / 右下）
	 *
	 * @param pointsEdgeLine 赛道边缘点集
	 * @param pointsCone     锥桶检测结果
	 * @return POINT
	 */
	POINT searchNearestCone(vector<POINT> pointsEdgeLine, vector<POINT> pointsCone)
	{
		POINT point(0, 0);
		double disMin = 50; // 右边缘锥桶离赛道左边缘最小距离

		if (pointsCone.size() <= 0 || pointsEdgeLine.size() < 10)
			return point;

		POINT a = pointsEdgeLine[pointsEdgeLine.size() / 4];
		POINT b = pointsEdgeLine[pointsEdgeLine.size() / 2];

		for (int i = 0; i < pointsCone.size(); i++)
		{
			double dis = distanceForPoint2Line(a, b, pointsCone[i]);
			if (dis < disMin && pointsCone[i].x > point.x)
			{
				point = pointsCone[i];
			}
		}

		return point;
	}

	//传统视觉识别锥桶
	std::vector<cv::Rect> detectCones(cv::Mat img_rgb)
	{
		std::vector<cv::Rect> coneRects;
		// 设置锥桶颜色的RGB范围（黄色）
		cv::Scalar lowerYellow(0, 100, 100);
		cv::Scalar upperYellow(100, 255, 255);

		// 在RGB图像中根据颜色范围提取锥桶区域
		cv::Mat mask;
		cv::inRange(img_rgb, lowerYellow, upperYellow, mask);

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

private:
    POINT _pointNearCone;
    std::vector<POINT> pointEdgeDet;        // 锥桶检测边缘点集
	std::vector<cv::Rect> _coneRects;       // 传统视觉识别锥桶的方框点集
    std::vector<POINT> lastPointsEdgeLeft;  // 记录上一场边缘点集（丢失边）
    std::vector<POINT> lastPointsEdgeRight; // 记录上一场边缘点集（丢失边）
    bool exitTwoEnable = false;             // 二号出口使能标志
    bool slowDown = false;                  // 减速使能

    enum GranaryStep
    {
        None = 0, // 未触发
        Enable,   // 粮仓操作使能（标志识别成功）
        Enter,    // 粮仓进站
        Cruise,   // 粮仓巡航
        Exit      // 粮仓出站
    };

    enum GranaryType
    {
        ExitNone = 0, // 未触发
        ExitOne,  // 一号出口
        ExitTwo   // 二号出口
    };

    GranaryStep granaryStep = GranaryStep::None;
    GranaryType granaryType = GranaryType::ExitNone;
    uint16_t counterSession = 0; // 图像场次计数器
    uint16_t counterRec = 0;     // 粮仓标志检测计数器
    uint16_t counterShield = 0;  // 粮仓屏蔽计数器
	uint16_t counterExit = 0;	 // 标志结束计数器
    uint16_t numGranary = 0;     // 粮仓标志的数量

    /**
     * @brief 粮仓区核心参数
     *
     */
    struct Params
    {
        uint16_t GranaryCheck = 3;
        uint16_t ServoRow = 50;
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Params, GranaryCheck, ServoRow); // 添加构造函数
    };
    Params params;
};
