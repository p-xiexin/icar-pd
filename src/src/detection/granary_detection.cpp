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
    bool granaryDetection(TrackRecognition& track, cv::Mat frame)
    {
        switch(granaryStep)
        {
        case GranaryStep::Enable:
        {
			counterExit++;
			if (counterExit > 100) {
				reset();
				return false;
			}

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
        //for (int i = 0; i < pointEdgeDet.size(); i++)
        //{
        //   circle(image, Point(pointEdgeDet[i].y, pointEdgeDet[i].x), 2, Scalar(92, 92, 205), -1); // 锥桶坐标：红色
        //}

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

        if (_pointNearCone.x > 0)
            circle(image, Point(_pointNearCone.y, _pointNearCone.x), 3, Scalar(200, 200, 200), -1);
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
private:
    POINT _pointNearCone;
    std::vector<POINT> pointEdgeDet;        // AI元素检测边缘点集
    std::vector<POINT> lastPointsEdgeLeft;  // 记录上一场边缘点集（丢失边）
    std::vector<POINT> lastPointsEdgeRight; // 记录上一场边缘点集（丢失边）
    bool exitTwoEnable = false;        // 二号出口使能标志

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
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Params, GranaryCheck); // 添加构造函数
    };
    Params params;
};
