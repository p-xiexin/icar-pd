#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2021; SaiShu.Lcc.; Leo;
 *https://bjsstech.com 版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial
 *transactions(开源学习,请勿商用). The code ADAPTS the corresponding hardware
 *circuit board(代码适配百度最新板卡PPNCx), The specific details consult the
 *professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file motion_controller.cpp
 * @author Leo ()
 * @brief 运动控制器：PD姿态控制||速度控制
 * @version 0.1
 * @date 2022-02-22
 * @note PD控制器要求稳定的控制周期：~40ms
 * @copyright Copyright (c) 2022
 *
 */

#include "../include/common.hpp"
#include "../include/json.hpp"
#include "controlcenter_cal.cpp"
#include <cmath>
#include <fstream>
#include <iostream>

using namespace std;

class MotionController 
{
private:
  int counterShift = 0; // 变速计数器

public:
    /**
     * @brief 控制器核心参数
     */
    struct Params 
    {
        float speedLow = 1.0;       // 智能车最低速
        float speedHigh = 1.0;      // 智能车最高速
        float speedGarage = 1.0;    // 出入车库速度
        float speedCorners = 1.0;   // 贴弯速度
        float runP1 = 0.9;          // 一阶比例系数：直线控制量
        float runP2 = 0.018;        // 二阶比例系数：弯道控制量
        float runP3 = 0.0;          // 三阶比例系数：弯道控制量
        float turnP = 3.5;          // 一阶比例系数：转弯控制量
        float turnD = 3.5;          // 一阶微分系数：转弯控制量
        float Kp = 1000.0;
        float Ki = 0.0;
        float Kd = 0.0;
        uint16_t rowCutUp = 30;     // 图像顶部切行
        uint16_t rowCutBottom = 10; // 图像顶部切行
        bool Debug = false;
        bool CloseLoop = true;
        bool GarageEnable = false;   // 出入库使能
        bool RingEnable = false;     // 环岛使能
        bool CrossEnable = true;    // 十字使能
        bool StopEnable = false;     // 冲出赛道停止使能
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Params, speedLow, speedHigh, speedGarage, speedCorners, 
            runP1, runP2, runP3, turnP, turnD, rowCutUp, rowCutBottom, Kp, Ki, Kd,
            Debug, CloseLoop, GarageEnable, RingEnable, CrossEnable, StopEnable); // 添加构造函数
    };

    Params params;                   // 读取控制参数
    uint16_t servoPwm = PWMSERVOMID; // 发送给舵机的PWM
    float motorSpeed = 1.0;          // 发送给电机的速度

    /**
     * @brief 姿态PD控制器
     * @param controlCenter 智能车控制中心
     */
    void pdController(int controlCenter)
    {
        float error = controlCenter - COLSIMAGE / 2; // 图像控制中心转换偏差
        static int errorLast = 0;                    // 记录前一次的偏差
        static int T_cnt = 0;

        if(abs(error - errorLast) > COLSIMAGE / 12) 
        {
            T_cnt++;
            error = error > errorLast ? errorLast + COLSIMAGE / 12 : errorLast - COLSIMAGE / 12;
        }
        else
        {
            T_cnt = 0;
        }

        params.turnP = abs(error) * params.runP2 + params.runP1;
        if(T_cnt >= 5)
        {
            params.turnP += params.runP3 * abs(error);
        }
        // turnP = max(turnP,0.2);
        //  if(turnP<0.2){
        //      turnP = 0.2;
        //  }
        // turnP = runP1 + heightest_line * runP2;

        int pwmDiff = (error * params.turnP) + (error - errorLast) * params.turnD;
        errorLast = error;

        servoPwm = (uint16_t)(PWMSERVOMID + pwmDiff); // PWM转换
    }

  /**
   * @brief 变加速控制
   * @param up_speed_enable 加速使能
   * @param slowDown_enable 减速使能
   * @param control 小车控制类
   */
    void speedController(bool up_speed_enable, bool slowDown_enable, ControlCenterCal control) 
    {
        // 控制率，在符合加速条件，并且一段时间后，开始加速，不然低速跑
        uint8_t controlLow = 0;             // 速度控制计数器下限
        uint8_t controlspeedCorners = 3;    // 贴弯控制率
        uint8_t controlMid = 5;             // 控制率
        uint8_t controlHigh = 10;           // 速度控制计数器上限

        if(slowDown_enable) 
        {
            //变速计数器
            counterShift = controlLow;
            //发送的电机速度
            motorSpeed = params.speedLow;
        }
        else if(up_speed_enable) // 加速使能
        {
            //连续转弯，慢速行驶
            if (control.style == "RIGHTCC" || control.style == "LEFTCC")
            {
                //发送的电机速度，最低速
                motorSpeed = params.speedLow;
                //变速计数器清零
                counterShift = controlLow;
                return;
            }
            //急转弯中，且单边，说明贴线行驶，可加速
            else if (control.style == "RIGHT_D" || control.style == "LEFT_D")     
            {
                //发送的电机速度，最低速
                motorSpeed = params.speedCorners;
                return;
            }
            else if(control.style == "STRIGHT")         //直道
            {
                if (abs(control.sigmaCenter) < 100.0)
                {
                    //符合加速条件，开始计数
                    counterShift++;
                    //在符合条件下，达到计数长度上限
                    if (counterShift > controlHigh)
                        counterShift = controlHigh;

                    //判断是加速还是减速
                    if (counterShift > controlMid)
                        motorSpeed = params.speedHigh;
                    else
                        motorSpeed = params.speedLow;
                }
                else    //控制线比较弯曲，并且持续一段时间，减速
                {
                    counterShift--;
                    //在符合条件下，达到计数长度下限
                    if (counterShift < controlLow)
                        counterShift = controlLow;

                    //判断是加速还是减速
                    if (counterShift > controlMid)
                        motorSpeed = params.speedHigh;
                    else
                        motorSpeed = params.speedLow;

                }
            }
        }
        else    //没有任何操作，最低速跑车
        {
            counterShift = controlLow;
            motorSpeed = params.speedLow;
        }
    }

    /**
     * @brief 加载配置参数Json
     */
    void loadParams() 
    {
        string jsonPath = "../config/motion.json";
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

        motorSpeed = params.speedLow;
        cout << "--- runP1:" << params.runP1 << " | runP2:" << params.runP2
            << " | runP3:" << params.runP3 << endl;
        cout << "--- turnP:" << params.turnP << " | turnD:" << params.turnD << endl;
        cout << "--- speedLow:" << params.speedLow
            << "m/s  |  speedHigh:" << params.speedHigh << "m/s" << endl;
    }
};