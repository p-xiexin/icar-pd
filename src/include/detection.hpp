#pragma once

#include "capture.hpp"
#include "predictor.hpp"
#include "stop_watch.hpp"
#include <condition_variable>
#include <mutex>
#include <thread>

struct DetectionResult
{
    cv::Mat det_render_frame;//推理结果绘制图 传出
    cv::Mat rgb_frame;//摄像机原始图像  由主线程传入
    std::vector<PredictResult> predictor_results;
};

class Detection
{

public:
    Detection() {}
    ~Detection() {}

    void start()
    {
        _thread = std::make_unique<std::thread>([this]() {
            while (1)
            {
                std::shared_ptr<DetectionResult> result = std::make_shared<DetectionResult>();

                if (result->rgb_frame.empty()) {
                    std::cout << "Error: Capture Get Empty Error Frame." << std::endl;
                    exit(-1);
                }

                //AI推理
                result->predictor_results = _predictor->run(result->rgb_frame);
                result->det_render_frame = result->rgb_frame.clone();
                _predictor->render(result->det_render_frame, result->predictor_results);

            
                //多线程共享数据传递
                std::unique_lock<std::mutex> lock(_mutex);
                _lastResult = result;
                cond_.notify_all();

            } 
        });
    }

    std::shared_ptr<DetectionResult> getLastFrame()
    {
        std::shared_ptr<DetectionResult> ret = nullptr;
        {
            std::unique_lock<std::mutex> lock(_mutex);

            while (_lastResult == nullptr)
            {
                cond_.wait(lock);
            }
            ret = _lastResult;
            _lastResult = nullptr;
        }
        return ret;
    }

    static std::shared_ptr<Detection> DetectionInstance(std::string model_path)
    {
        static std::shared_ptr<Detection> detectioner = nullptr;
        if (detectioner == nullptr)
        {
            detectioner = std::make_shared<Detection>();
            int ret = detectioner->_init(model_path);
            if (ret != 0)
            {
                std::cout << "Detection init error :" << model_path << std::endl;
                exit(-1);
            }
            detectioner->start();
        }
        return detectioner;
    }

private:
    int _init(std::string model_config_path)
    {
        _predictor = std::make_shared<PPNCDetection>();
        if (_predictor == nullptr)
        {
            std::cout << "Predictor Create failed." << std::endl;
            return -1;
        }
        int ret = _predictor->init(model_config_path);
        if (ret != 0)
        {
            std::cout << "Predictor init failed." << std::endl;
            return -1;
        }
        return 0;
    }

    std::shared_ptr<PPNCDetection> _predictor;
    std::shared_ptr<DetectionResult> _lastResult;

    std::mutex _mutex;
    std::condition_variable cond_;
    std::unique_ptr<std::thread> _thread;
};