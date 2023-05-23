#pragma once

/**
 * @file farmland_avoidance.cpp
 * @author pxx
 * @brief 农田断路区避障算法
 * @version 0.1
 * @date 2023-05-22
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


class FarmlandAvoidance
{
public:
    FarmlandAvoidance()
    {
        loadParams();
        kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(params.CloseWidth, params.CloseHeigth));//创建结构元
		kernel_enrode = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(params.EnrodeWidth, params.EnrodeHeigth));
    }
    /**
     * @brief 初始化
     *
     */
    void reset(void)
    {
        farmlandStep = FarmlandStep::None;
    }
    
    /**
     * @brief 农田区域检测与路径规划
     *
     * @param track 赛道识别结果
     * @param detection AI检测结果
     */
    bool farmlandAvoidance(TrackRecognition &track, vector<PredictResult> predict)
    {
        switch(farmlandStep)
        {
        case FarmlandDetection::None:
        {
            searchCorn(predict); // 玉米检测
            if (pointCorn.x > 0 || pointCorn.y > 0)
                counterRec++;

            if (counterRec)
            {
                counterSession++;
                if (counterRec > params.FarmlandCheck && counterSession < params.FarmlandCheck * 2)
                {
                    farmlandStep = FarmlandStep::Enable; // 农田区域使能
                    counterRec = 0;
                    counterSession = 0;
                }
                else if (counterSession >= params.FarmlandCheck)
                {
                    counterRec = 0;
                    counterSession = 0;
                }
            }
            // pointsEdgeLeftLast = track.pointsEdgeLeft; // 记录前一场数据
            // pointsEdgeRightLast = track.pointsEdgeRight;
            break;
        }
        }
        case FarmlandDetection::Enable:
        {
            if (track.pointsEdgeLeft.size() < params.EnterLine && track.pointsEdgeRight.size() < params.EnterLine)
            {
                counterRec++;
                if (counterRec > 2)
                {
                    counterRec = 0;
                    farmlandStep = FarmlandStep::Enter; // 进入农田
                }
            }

            uint16_t rowEndLeft = searchEndLeft(track.pointsEdgeLeft);
            uint16_t rowEndRight = searchEndRight(track.pointsEdgeRight);
            if(rowEndLeft)
            {
                track.pointsEdgeLeft.resize(rowEndLeft);
            }
            if(rowEndRight)
            {
                track.pointsEdgeRight.resize(rowEndRight);
            }
            
            break;
        }
        case FarmlandDetection::Enter:
        {
            
            break;
        }
        case FarmlandDetection::Cruise:
        {
            
            break;
        }
    }

    /**
     * @brief 加载配置参数Json
     */
    void loadParams() 
    {
        string jsonPath = "../src/config/motion.json";
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
     * @brief 对锥桶做腐蚀处理，以便于重新得到边线
     *
     * @param image 彩色通道图
     */
    cv::Mat ConeEnrode(cv::Mat image)
    {
        std::vector<cv::Mat> channels;
        cv::split(image, channels);
        cv::Mat blueChannel = channels[0];

		cv::Mat imageGray, imageBinary;

        // cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));//创建结构元
		// cv::Mat kernel_enrode = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(120, 60));
		cv::morphologyEx(blueChannel, blueChannel, cv::MORPH_CLOSE, kernel_close, cv::Point(-1, -1));//闭运算
		cv::morphologyEx(blueChannel, imageGray, cv::MORPH_ERODE, kernel_enrode, cv::Point(-1, -1));//腐蚀运算
		cv::threshold(imageGray, imageBinary, 0, 255, cv::THRESH_OTSU);

        return imageBinary;
    }
private:
    /**
     * @brief 玉米坐标检测
     *
     * @param predict
     */
    void searchCorn(vector<PredictResult> predict)
    {
        POINT corn = POINT(0, 0);
        int distance = COLSIMAGE / 2;
        for (int i = 0; i < predict.size(); i++)
        {
            if (predict[i].label == LABEL_CORN && abs(predict[i].x + predict[i].width / 2 - COLSIMAGE / 2) < distance) // 玉米检测
            {
                corn = POINT(predict[i].y + predict[i].height, predict[i].x + predict[i].width / 2);
                distance = abs(predict[i].x + predict[i].width / 2 - COLSIMAGE / 2);
            }
        }

        pointCorn = corn;
    }
    /**
     * @brief 从AI检测结果中检索锥桶坐标集合
     *
     * @param predict AI检测结果
     */
    void searchCones(vector<PredictResult> predict)
    {
        // AI结果检索
        pointEdgeDet.clear();
        for (int i = 0; i < predict.size(); i++)
        {
            if (predict[i].label == LABEL_CONE) // 锥桶检测
            {
                pointEdgeDet.push_back(POINT(predict[i].y + predict[i].height / 2, predict[i].x + predict[i].width / 2));
            }
        }
    }

    /**
     * @brief 左边线突变行检测
     *
     * @param pointsEdgeLine
     */
    uint16_t searchEndLeft(vector<POINT> pointsEdgeLeft)
    {
        uint16_t rowEndLine = pointsEdgeLeft.size() - 1;
        uint16_t counter = 0;
        
        for(int i = pointsEdgeLeft.size() - 1; i > 0; i--)
        {
            if(pointsEdgeLeft[i].y < pointsEdgeLeft[rowEndLine].y)
            {
                rowEndLine = i;
                counter = 0;
            }
            else
            {
                counter++:
                if(counter > 8)
                {
                    return rowEndLine;
                }
            }
        }

        return 0;
    }
    /**
     * @brief 右边线突变行检测
     *
     * @param pointsEdgeLine
     */
    uint16_t searchEndRight(vector<POINT> pointsEdgeRight)
    {
        uint16_t rowEndLine = pointsEdgeRight.size() - 1;
        uint16_t counter = 0;
        
        for(int i = pointsEdgeRight.size() - 1; i > 0; i--)
        {
            if(pointsEdgeRight[i].y > pointsEdgeRight[rowEndLine].y)
            {
                rowEndLine = i;
                counter = 0;
            }
            else
            {
                counter++:
                if(counter > 8)
                {
                    return rowEndLine;
                }
            }
        }

        return 0;
    }

    cv::Mat kernel_close;//闭运算滤波结构元
	cv::Mat kernel_enrode;//腐蚀运算滤波结构元
    vector<POINT> pointEdgeDet;        // AI元素检测边缘点集
    POINT pointCorn = POINT(0, 0);     // 玉米检测坐标
    uint16_t counterSession = 0;       // 图像场次计数器
    uint16_t counterRec = 0;           // 农田区标志检测计数器
    int indexDebug = 0;
    enum FarmlandStep
    {
        None = 0, // 未触发
        Enable,   // 农田区域检测使能（检测到玉米）
        Enter,    // 完全驶入农田
        Cruise
    };
    FarmlandStep farmlandStep = FarmlandStep::None;

    /**
     * @brief 农田区域核心参数
     */
    struct Params 
    {
        uint16_t FarmlandCheck = 4;
        float Speed = 0.6;
        uint16_t CloseWidth = 3;
        uint16_t CloseHeigth = 3;
        uint16_t EnrodeWidth = 80;
        uint16_t EnrodeHeigth = 40;
        uint16_t EnterLine = 50;
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Params, FarmlandCheck, Speed, CloseWidth, CloseHeigth, EnrodeWidth, EnrodeHeigth, EnterLine); // 添加构造函数
    };
    Params params;
}
