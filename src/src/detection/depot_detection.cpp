#pragma once

/**
 * @file depot_detection.cpp
 * @author pxx
 * @brief 维修厂检测
 * @version 0.1
 * @date 2023-05-17
 *
 */

#include "../../include/common.hpp"
#include "../../include/predictor.hpp"
#include "../recognize/track_recognition.cpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class DepotDetection
{
public:
	DepotDetection()
	{
		loadParams();
	}
public:

    /**
     * @brief 控制器核心参数
     */
    struct Params 
    {
		uint16_t DepotCheck = 3;
		uint16_t DepotDir = 0;
		double DangerClose = 100.0;       // 智能车危险距离
		uint16_t ServoRow = 120;
		uint16_t ServoValue = 15;
		float DepotSpeed = 0.5;
		uint16_t DelayCnt = 3;
		uint16_t BrakeCnt = 5;
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Params, DepotCheck, DepotDir, DangerClose, ServoRow, ServoValue, DepotSpeed, DelayCnt, BrakeCnt); // 添加构造函数
    };

	enum DepotStep
	{
		DepotNone = 0, // 未触发
		DepotEnable,   // 维修厂操作使能（标志识别成功）
		DepotEnter,	   // 维修厂进站
		DepotCruise,   // 维修站巡航
		DepotStop,	   // 停车
		DepotExit	   // 维修厂出站
	};
    /**
     * @brief 环岛类型
     *
     */
    enum DepotType
    {
        None = 0, 
        DepotLeft,     // 左维修区
        DepotRight     // 右维修区
    };
	DepotType depotType = DepotType::None;


	/**
	 * @brief 维修厂检测初始化
	 *
	 */
	void reset(void)
	{
		depotStep = DepotStep::DepotNone;
		depotType = DepotType::None;
		counterSession = 0;			// 图像场次计数器
		counterRec = 0;				// 维修厂标志检测计数器
		counterExit = 0;
		counterImmunity = 0;
		_speed = 0.0f;
		_slowdown = false;

		for (std::vector<POINT>& innerVec : pathsEdgeLeft) {
			innerVec.clear();
		}
		pathsEdgeLeft.clear();

		for (std::vector<POINT>& innerVec : pathsEdgeRight) {
			innerVec.clear();
		}
		pathsEdgeRight.clear();
	}

	/**
	 * @brief 维修厂检测，在Ai线程运行
	 *
	 * @param detection AI检测结果
	 */
	void depotDetection(vector<PredictResult> predict)
	{
		if(counterImmunity < 50)
		{
			return;
		}

		switch (depotStep)
		{
		case DepotStep::DepotNone: //[01] 维修厂标志检测
		{
			for (int i = 0; i < predict.size(); i++)
			{
				if (predict[i].label == LABEL_TRACTOR && predict[i].y + predict[i].height / 2 > 60) // 拖拉机标志检测
				{
					counterRec++;
					break;
				}
			}
			if (counterRec)
			{
				counterSession++;
				if (counterRec > params.DepotCheck && counterSession < params.DepotCheck + 3)
				{
					// if(params.DepotDir == 0)
					// 	depotType = DepotType::DepotLeft;
					// else if(params.DepotDir == 1)
					// 	depotType = DepotType::DepotRight;
					depotStep = DepotStep::DepotEnable; // 维修厂使能
					counterRec = 0;
					counterSession = 0;
				}
				else if (counterSession >= params.DepotCheck + 3)
				{
					counterRec = 0;
					counterSession = 0;
				}
			}
			break;
		}
		case DepotStep::DepotEnable: //[02] 维修厂使能
		{
			POINT tractor(0, 0);
			for (int i = 0; i < predict.size(); i++)
			{
				if (predict[i].label == LABEL_TRACTOR && predict[i].y + predict[i].height / 2 < ROWSIMAGE / 2) // 拖拉机标志检测
				{
					tractor = POINT(predict[i].y + predict[i].height / 2, predict[i].x + predict[i].width / 2);
					break;
				}
			}

			if(tractor.x == 0)
			{
				counterExit++;
				if(counterRec)
					counterSession++;
			}
			else if(tractor.x > params.ServoRow + ROWSIMAGE / 6)
				counterRec++;
			else 
			{
				counterSession = 0;
				counterExit = 0;
			}

			if(counterSession > 1 || counterExit > 4)
			{
				counterRec = 0;
				counterSession = 0;
				counterExit = 0;
				if(params.DepotDir == 0)
					depotType = DepotType::DepotLeft;
				else if(params.DepotDir == 1)
					depotType = DepotType::DepotRight;
			}
			break;
		}
		}
	}


	/**
	 * @brief 维修厂路径规划，在主线程运行
	 *
	 * @param track 赛道识别结果
	 * @param detection AI检测结果
	 */
	bool depotDetection(TrackRecognition &track, cv::Mat img_rgb)
	{
		if(counterImmunity < 50)
		{
			counterImmunity++;
			return false;
		}

		_pointNearCone = POINT(0, 0);
		_distance = 0;
		pointEdgeDet.clear();
		_coneRects.clear();
		indexDebug = 0;

		switch (depotStep)
		{
		case DepotStep::DepotEnable: //[02] 维修厂使能
		{

			if(depotType == DepotType::None)
				return true;

			counterExit++;
			if (counterExit > 100) {
				reset();
				return false;
			}
			counterSession++;//刚进入维修区，延时等待知道能看到所有锥桶
			
			_coneRects = detectCones(img_rgb);
			searchCones(_coneRects, track.rowCutUp);

			if(depotType == DepotType::DepotLeft)
				_pointNearCone = searchNearestCone(track.pointsEdgeLeft, pointEdgeDet);		 // 搜索右下锥桶
			else if(depotType == DepotType::DepotRight)
				_pointNearCone = searchNearestCone(track.pointsEdgeRight, pointEdgeDet);		 // 搜索右下锥桶

			if (_pointNearCone.x > params.ServoRow && _pointNearCone.x < params.ServoRow + ROWSIMAGE / 3
				&& _pointNearCone.y != 0 && counterSession > params.DelayCnt/* && _slowdown*/) // 当车辆开始靠近右边锥桶：准备入库
			{
				counterRec++;
				if (counterRec > 2)
				{
					depotStep = DepotStep::DepotEnter; // 进站使能
					counterRec = 0;
					counterSession = 0;
				}
			}

			indexDebug = counterRec;
			break;
		}
		case DepotStep::DepotEnter: //[03] 进站使能
		{
			_coneRects = detectCones(img_rgb);
			searchCones(_coneRects, track.rowCutUp);
			_distance = 0;
			_pointNearCone = searchClosestCone(pointEdgeDet);
			if(_distance < params.DangerClose && _distance > 0)
			{
				counterRec++;
				if(counterRec > 1)
				{
					depotStep = DepotStep::DepotCruise;
					counterRec = 0;
				}
			}
			if(depotType == DepotType::DepotLeft)
			{
				POINT start = POINT(ROWSIMAGE - 40, COLSIMAGE - 1);
				POINT end = POINT(ROWSIMAGE / 2 - params.ServoValue, 0);
				POINT middle = POINT((start.x + end.x) * 0.4, (start.y + end.y) * 0.6);
				vector<POINT> input = {start, middle, end};
				track.pointsEdgeRight = Bezier(0.05, input); // 补线
				track.pointsEdgeLeft = predictEdgeLeft(track.pointsEdgeRight); // 由右边缘补偿左边缘
			}
			else if(depotType == DepotType::DepotRight)
			{
				POINT start = POINT(ROWSIMAGE - 40, 0);
				POINT end = POINT(ROWSIMAGE / 2 - params.ServoValue, COLSIMAGE - 1);
				POINT middle = POINT((start.x + end.x) * 0.4, (start.y + end.y) * 0.6);
				vector<POINT> input = {start, middle, end};
				track.pointsEdgeLeft = Bezier(0.05, input); // 补线
				track.pointsEdgeRight = predictEdgeRight(track.pointsEdgeLeft); // 由右边缘补偿左边缘
			}
			pathsEdgeLeft.push_back(track.pointsEdgeLeft); // 记录进厂轨迹
			pathsEdgeRight.push_back(track.pointsEdgeRight);

			break;
		}

		case DepotStep::DepotCruise: //[04] 巡航使能(Brake)
		{
			{
				// 预留给以后完善
			}
			counterRec++;
			if(counterRec > params.BrakeCnt)
			{
				counterRec = 0;
				depotStep = DepotStep::DepotStop;
			}
			track.pointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];//维持入库最后的打角
			track.pointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];
			// pathsEdgeLeft.push_back(track.pointsEdgeLeft); // 记录进厂轨迹
			// pathsEdgeRight.push_back(track.pointsEdgeRight);
			break;
		}

		case DepotStep::DepotStop: //[05] 停车使能
		{
			counterRec++;
			if (counterRec > 12) // 停车：40场 = 2s
			{
				depotStep = DepotStep::DepotExit; // 出站使能
				counterRec = 0;
			}
			track.pointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];//维持入库最后的打角
			track.pointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];
			break;
		}

		case DepotStep::DepotExit: //[06] 出站使能
		{
			if (pathsEdgeLeft.size() < 2 || pathsEdgeRight.size() < 2)
			{
				if((track.pointsEdgeLeft.size() > 160 && track.pointsEdgeRight.size() > 160) && track.widthBlock[10].y > COLSIMAGE / 2)
				{
					counterRec++;
				}
				track.pointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];
				track.pointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];

				if(counterRec > params.BrakeCnt)
				{
					depotStep = DepotStep::DepotNone; // 出厂完成
					reset();
				}
			}
			else
			{
				track.pointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];
				track.pointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];
				pathsEdgeLeft.pop_back();
				pathsEdgeRight.pop_back();
			}
			break;
		}
		}

		if (depotStep == DepotStep::DepotNone) // 返回维修厂控制模式标志
			return false;
		else
			return true;
	}


	/**
	 * @brief 维修厂检测与路径规划
	 *
	 * @param track 赛道识别结果
	 * @param detection AI检测结果
	 */
	bool depotDetection(TrackRecognition &track, vector<PredictResult> predict)
	{
		_pointNearCone = POINT(0, 0);
		_distance = 0;
		pointEdgeDet.clear();
		indexDebug = 0;

		if(counterImmunity < 30)
		{
			counterImmunity++;
			return false;
		}

		switch (depotStep)
		{
		case DepotStep::DepotNone: //[01] 维修厂标志检测
		{
			for (int i = 0; i < predict.size(); i++)
			{
				if (predict[i].label == LABEL_TRACTOR && predict[i].y + predict[i].height / 2 > 60) // 拖拉机标志检测
				{
					counterRec++;
					break;
				}
			}
			if (counterRec)
			{
				counterSession++;
				if (counterRec > params.DepotCheck && counterSession < params.DepotCheck + 3)
				{
					if(params.DepotDir == 0)
						depotType = DepotType::DepotLeft;
					else if(params.DepotDir == 1)
						depotType = DepotType::DepotRight;
					depotStep = DepotStep::DepotEnable; // 维修厂使能
					counterRec = 0;
					counterSession = 0;
				}
				else if (counterSession >= params.DepotCheck + 3)
				{
					counterRec = 0;
					counterSession = 0;
				}
			}
			break;
		}
		case DepotStep::DepotEnable: //[02] 维修厂使能
		{
			counterExit++;
			if (counterExit > 60) {
			  reset();
			  return false;
			}
			counterSession++;//刚进入维修区，延时等待知道能看到所有锥桶

			searchCones(predict);
			for (int i = 0; i < predict.size(); i++)
			{
				if (predict[i].label == LABEL_TRACTOR && predict[i].x < ROWSIMAGE / 2) // 拖拉机标志检测
				{
					_slowdown = false;
					break;
				}
				else
					_slowdown = true;
			}

			if(depotType == DepotType::DepotLeft)
				_pointNearCone = searchNearestCone(track.pointsEdgeLeft, pointEdgeDet);		 // 搜索右下锥桶
			else if(depotType == DepotType::DepotRight)
				_pointNearCone = searchNearestCone(track.pointsEdgeRight, pointEdgeDet);		 // 搜索右下锥桶

			if (_pointNearCone.x > params.ServoRow && _pointNearCone.x < params.ServoRow + ROWSIMAGE / 3
				&& _pointNearCone.y != 0 && counterSession > params.DelayCnt && _slowdown) // 当车辆开始靠近右边锥桶：准备入库
			{
				counterRec++;
				if (counterRec > 2)
				{
					depotStep = DepotStep::DepotEnter; // 进站使能
					counterRec = 0;
					counterSession = 0;
				}
			}

			indexDebug = counterRec;
			break;
		}
		case DepotStep::DepotEnter: //[03] 进站使能
		{
			searchCones(predict);
			_distance = 0;
			_pointNearCone = searchClosestCone(pointEdgeDet);
			if(_distance < params.DangerClose && _distance > 0)
			{
				counterRec++;
				if(counterRec > 1)
				{
					depotStep = DepotStep::DepotCruise;
					counterRec = 0;
				}
			}

			if(depotType == DepotType::DepotLeft)
			{
				POINT start = POINT(ROWSIMAGE - 40, COLSIMAGE - 1);
				POINT end = POINT(ROWSIMAGE / 2 - params.ServoValue, 0);
				POINT middle = POINT((start.x + end.x) * 0.4, (start.y + end.y) * 0.6);
				vector<POINT> input = {start, middle, end};
				track.pointsEdgeRight = Bezier(0.05, input); // 补线
				track.pointsEdgeLeft = predictEdgeLeft(track.pointsEdgeRight); // 由右边缘补偿左边缘
			}
			else if(depotType == DepotType::DepotRight)
			{
				POINT start = POINT(ROWSIMAGE - 40, 0);
				POINT end = POINT(ROWSIMAGE / 2 - params.ServoValue, COLSIMAGE - 1);
				POINT middle = POINT((start.x + end.x) * 0.4, (start.y + end.y) * 0.6);
				vector<POINT> input = {start, middle, end};
				track.pointsEdgeLeft = Bezier(0.05, input); // 补线
				track.pointsEdgeRight = predictEdgeRight(track.pointsEdgeLeft); // 由右边缘补偿左边缘
			}
			pathsEdgeLeft.push_back(track.pointsEdgeLeft); // 记录进厂轨迹
			pathsEdgeRight.push_back(track.pointsEdgeRight);

			break;
		}

		case DepotStep::DepotCruise: //[04] 巡航使能(Brake)
		{
			{
				// 预留给以后完善
			}
			counterRec++;
			if(counterRec > params.BrakeCnt)
			{
				counterRec = 0;
				depotStep = DepotStep::DepotStop;
			}
			track.pointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];//维持入库最后的打角
			track.pointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];
			// pathsEdgeLeft.push_back(track.pointsEdgeLeft); // 记录进厂轨迹
			// pathsEdgeRight.push_back(track.pointsEdgeRight);
			break;
		}

		case DepotStep::DepotStop: //[05] 停车使能
		{
			counterRec++;
			if (counterRec > 15) // 停车：40场 = 2s
			{
				depotStep = DepotStep::DepotExit; // 出站使能
				counterRec = params.BrakeCnt;
			}
			track.pointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];//维持入库最后的打角
			track.pointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];
			break;
		}

		case DepotStep::DepotExit: //[06] 出站使能
		{
			if (pathsEdgeLeft.size() < 1 || pathsEdgeRight.size() < 1)
			{
				if(counterRec == 0)
				{
					depotStep = DepotStep::DepotNone; // 出厂完成
					reset();
				}
				else
					counterRec--;
			}
			else
			{
				track.pointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];
				track.pointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];
				pathsEdgeLeft.pop_back();
				pathsEdgeRight.pop_back();
			}
			
			break;
		}
		}

		if (depotStep == DepotStep::DepotNone) // 返回维修厂控制模式标志
			return false;
		else
			return true;
	}

    /**
     * @brief 加载配置参数Json
     */
    void loadParams() 
    {
        string jsonPath = "../src/config/depot.json";
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
	 * @brief 维修区速度规划
	 *
	 */
	void speed_planning(float& speed)
	{
		switch(depotStep)
		{
		case DepotStep::DepotEnable:
		{
			if(_slowdown)
				speed -= 0.1f;

			if(speed < params.DepotSpeed)
				speed = params.DepotSpeed;
			break;
		}
		case DepotStep::DepotEnter:
		{
			speed = params.DepotSpeed;
			break;
		}
		case DepotStep::DepotCruise:
		{
			speed -= params.DepotSpeed / params.BrakeCnt;
			if(speed < 0.0f)
				speed = 0.0f;
			break;
		}
		case DepotStep::DepotStop:
		{
			speed = 0.0f;
			break;
		}
		case DepotStep::DepotExit:
		{
			if(pathsEdgeLeft.size() != 0 && pathsEdgeRight.size() != 0)
			{
				_speed -= params.DepotSpeed / params.BrakeCnt;
				if(_speed < -params.DepotSpeed)
					_speed = -params.DepotSpeed;
			}
			else
				speed += params.DepotSpeed / params.BrakeCnt;
			break;
		}
		}
	}


	/**
	 * @brief 获取维修区速度规划
	 *
	 */
	float get_speed()
	{
		switch(depotStep)
		{
		case DepotStep::DepotEnable:
		{
			_speed = params.DepotSpeed;
			break;
		}
		case DepotStep::DepotEnter:
		{
			_speed = params.DepotSpeed;
			break;
		}
		case DepotStep::DepotCruise:
		{
			_speed -= params.DepotSpeed / params.BrakeCnt;
			if(_speed < 0.0f)
				_speed = 0.0f;
			break;
		}
		case DepotStep::DepotStop:
		{
			_speed = 0.0f;
			break;
		}
		case DepotStep::DepotExit:
		{
			// if(pathsEdgeLeft.size() > params.BrakeCnt && pathsEdgeRight.size() > params.BrakeCnt)
			// {
			// 	_speed -= params.DepotSpeed / params.BrakeCnt;
			// 	if(_speed < -params.DepotSpeed)
			// 		_speed = -params.DepotSpeed;
			// }
			// else
			// {
			// 	_speed += params.DepotSpeed / params.BrakeCnt;
			// }
			if(!counterRec)
			{
				_speed -= params.DepotSpeed / params.BrakeCnt;
				if(_speed < -params.DepotSpeed)
					_speed = -params.DepotSpeed;
			}
			else
			{
				_speed += params.DepotSpeed / params.BrakeCnt;
			}
			break;
		}
		}
		return _speed;
	}

	/**
	 * @brief 识别结果图像绘制
	 *
	 */
	void drawImage(TrackRecognition track, Mat &image)
	{
		// 绘制锥桶坐标
		for (int i = 0; i < pointEdgeDet.size(); i++)
		{
            putText(image, to_string(i+1), Point(pointEdgeDet[i].y, pointEdgeDet[i].x), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1, CV_AA);
		}

		// 赛道边缘
		for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
		{
			circle(image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x),
				   1, Scalar(0, 255, 0), -1); // 绿色点
		}
		for (int i = 0; i < track.pointsEdgeRight.size(); i++)
		{
			circle(image,
				   Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1,
				   Scalar(0, 255, 255), -1); // 黄色点
		}

		// 显示维修厂状态
		string state = "None";
		switch (depotStep)
		{
		case DepotStep::DepotEnable:
			state = "DepotEnable";
			break;
		case DepotStep::DepotEnter:
			state = "DepotEnter";
			break;
		case DepotStep::DepotCruise:
			state = "DepotCruise";
			break;
		case DepotStep::DepotStop:
			state = "DepotStop";
			break;
		case DepotStep::DepotExit:
			state = "DepotExit";
			break;
		}
		if(depotType == DepotType::DepotLeft)
			state += " Left";
		if(depotType == DepotType::DepotRight)
			state += " Right";
		putText(image, state, Point(COLSIMAGE / 2 - 20, 20),
				cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 200), 1, CV_AA);

		putText(image, to_string(_distance), Point(COLSIMAGE / 2 - 15, 40),
				cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
				CV_AA); // 显示锥桶距离
		if (_pointNearCone.y > 0)
			circle(image, Point(_pointNearCone.y, _pointNearCone.x), 5,
				   Scalar(200, 200, 200), -1);

		putText(image, to_string(indexDebug),
				Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX,
				0.3, cv::Scalar(0, 0, 255), 1, CV_AA);
		if(_slowdown)
		{
			putText(image, "slowdown",
					Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 40), cv::FONT_HERSHEY_TRIPLEX,
					0.3, cv::Scalar(0, 0, 255), 1, CV_AA);
		}
	}

private:
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
	 * @brief 搜索距离赛道左边缘锥桶坐标(右下，非常规意义上的)
	 *
	 * @param pointsEdgeLeft 赛道边缘点集
	 * @param predict AI检测结果
	 * @return POINT
	 */
	POINT searchNearestCone(vector<POINT> pointsEdgeLeft,
							vector<POINT> pointsCone)
	{
		POINT point(ROWSIMAGE - 10, 0);
		double disMin = 80; // 右边缘锥桶离赛道左边缘最小距离

		if (pointsCone.size() <= 0 || pointsEdgeLeft.size() < 10)
			return point;

		POINT a = pointsEdgeLeft[pointsEdgeLeft.size() * 0.5];
		POINT b = pointsEdgeLeft[pointsEdgeLeft.size() * 0.8];

		for (int i = 0; i < pointsCone.size(); i++)
		{
			double dis = distanceForPoint2Line(a, b, pointsCone[i]);
			if (dis < disMin && pointsCone[i].x < point.x)
			{
				point = pointsCone[i];
				_distance = dis;
			}
		}

		return point;
	}

	/**
	 * @brief 搜索距离车最近的锥桶
	 *
	 * @param pointsEdgeLeft 赛道边缘点集
	 * @param predict AI检测结果
	 * @return POINT
	 */
	POINT searchClosestCone(vector<POINT> pointsCone)
	{
		POINT closestCone(0, 0);
		if (pointsCone.size() <= 0)
			return closestCone;

		POINT carCenter(ROWSIMAGE - 1, COLSIMAGE / 2);
		double closestLen = 0;
		closestCone = pointsCone[0];
		closestLen = distance(closestCone, carCenter);

		for (int i = 1; i < pointsCone.size(); i++)
		{
			double len = distance(pointsCone[i], carCenter);
			if(len < closestLen)
			{
				closestLen = len;
				closestCone = pointsCone[i];
			}
		}
		_distance = closestLen;

		return closestCone;
	}

	/**
	 * @brief 在俯视域由左边缘预测右边缘
	 *
	 * @param pointsEdgeLeft
	 * @return vector<POINT>
	 */
	vector<POINT> predictEdgeRight(vector<POINT> &pointsEdgeLeft)
	{
		int offset = 120; // 右边缘平移尺度
		vector<POINT> pointsEdgeRight;
		if (pointsEdgeLeft.size() < 3)
			return pointsEdgeRight;

		// Start
		Point2d startIpm = ipm.homography(
			Point2d(pointsEdgeLeft[0].y, pointsEdgeLeft[0].x)); // 透视变换
		Point2d prefictRight = Point2d(startIpm.x + offset, startIpm.y);
		Point2d startIipm = ipm.homographyInv(prefictRight); // 反透视变换
		POINT startPoint = POINT(startIipm.y, startIipm.x);

		// Middle
		Point2d middleIpm = ipm.homography(
			Point2d(pointsEdgeLeft[pointsEdgeLeft.size() / 2].y,
					pointsEdgeLeft[pointsEdgeLeft.size() / 2].x)); // 透视变换
		prefictRight = Point2d(middleIpm.x + offset, middleIpm.y);
		Point2d middleIipm = ipm.homographyInv(prefictRight); // 反透视变换
		POINT midPoint = POINT(middleIipm.y, middleIipm.x);	  // 补线中点

		// End
		Point2d endIpm = ipm.homography(
			Point2d(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y,
					pointsEdgeLeft[pointsEdgeLeft.size() - 1].x)); // 透视变换
		prefictRight = Point2d(endIpm.x + offset, endIpm.y);
		Point2d endtIipm = ipm.homographyInv(prefictRight); // 反透视变换
		POINT endPoint = POINT(endtIipm.y, endtIipm.x);

		// 补线
		vector<POINT> input = {startPoint, midPoint, endPoint};
		vector<POINT> repair = Bezier(0.05, input);

		for (int i = 0; i < repair.size(); i++)
		{
			if (repair[i].x >= ROWSIMAGE)
				repair[i].x = ROWSIMAGE - 1;

			else if (repair[i].x < 0)
				repair[i].x = 0;

			else if (repair[i].y >= COLSIMAGE)
				repair[i].y = COLSIMAGE - 1;
			else if (repair[i].y < 0)
				repair[i].y = 0;

			pointsEdgeRight.push_back(repair[i]);
		}

		return pointsEdgeRight;
	}

	/**
	 * @brief 在俯视域由右边缘预测左边缘
	 *
	 * @param pointsEdgeRight
	 * @return vector<POINT>
	 */
	vector<POINT> predictEdgeLeft(vector<POINT> &pointsEdgeRight)
	{
		int offset = 120; // 右边缘平移尺度
		vector<POINT> pointsEdgeLeft;
		if (pointsEdgeRight.size() < 3)
			return pointsEdgeLeft;

		// Start
		Point2d startIpm = ipm.homography(
			Point2d(pointsEdgeRight[0].y, pointsEdgeRight[0].x)); // 透视变换
		Point2d prefictLeft = Point2d(startIpm.x - offset, startIpm.y);
		Point2d startIipm = ipm.homographyInv(prefictLeft); // 反透视变换
		POINT startPoint = POINT(startIipm.y, startIipm.x);

		// Middle
		Point2d middleIpm = ipm.homography(
			Point2d(pointsEdgeRight[pointsEdgeRight.size() / 2].y,
					pointsEdgeRight[pointsEdgeRight.size() / 2].x)); // 透视变换
		prefictLeft = Point2d(middleIpm.x - offset, middleIpm.y);
		Point2d middleIipm = ipm.homographyInv(prefictLeft); // 反透视变换
		POINT midPoint = POINT(middleIipm.y, middleIipm.x);	 // 补线中点

		// End
		Point2d endIpm = ipm.homography(
			Point2d(pointsEdgeRight[pointsEdgeRight.size() - 1].y,
					pointsEdgeRight[pointsEdgeRight.size() - 1].x)); // 透视变换
		prefictLeft = Point2d(endIpm.x - offset, endIpm.y);
		Point2d endtIipm = ipm.homographyInv(prefictLeft); // 反透视变换
		POINT endPoint = POINT(endtIipm.y, endtIipm.x);

		// 补线

		vector<POINT> input = {startPoint, midPoint, endPoint};
		vector<POINT> repair = Bezier(0.05, input);

		for (int i = 0; i < repair.size(); i++)
		{
			if (repair[i].x >= ROWSIMAGE)
				repair[i].x = ROWSIMAGE - 1;

			else if (repair[i].x < 0)
				repair[i].x = 0;

			else if (repair[i].y >= COLSIMAGE)
				repair[i].y = COLSIMAGE - 1;
			else if (repair[i].y < 0)
				repair[i].y = 0;

			pointsEdgeLeft.push_back(repair[i]);
		}

		return pointsEdgeLeft;
	}

	//两点之间的像素距离
	double distance(POINT x1, POINT x2)
	{
		double dx = x1.x - x2.x;
		double dy = x1.y - x2.y;
		double len = std::sqrt(dx * dx + dy * dy);
		return len;
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
	DepotStep depotStep = DepotStep::DepotNone;
	Params params;                   // 读取控制参数
	
	double _distance = 0;
	POINT _pointNearCone;
	std::vector<POINT> pointEdgeDet; // AI元素检测边缘点集
	std::vector<cv::Rect> _coneRects;
	float _speed = 0.0f;

	std::vector<std::vector<POINT>> pathsEdgeLeft; // 记录入库路径
	std::vector<std::vector<POINT>> pathsEdgeRight;
	int indexDebug = 0;

	uint16_t counterSession = 0;  // 图像场次计数器
	uint16_t counterRec = 0;	  // 维修厂标志检测计数器
	uint16_t counterExit = 0;	  // 标志结束计数器
	uint16_t counterImmunity = 0; // 屏蔽计数器

	bool _slowdown = false;
};
