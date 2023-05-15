#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <chrono>

#include "uart.hpp"
#include "common.hpp"
#include "stop_watch.hpp"

class SerialInterface
{
public:
    SerialInterface(std::string serial_path, LibSerial::BaudRate bps) 
    : _speed(0)
    , _servo_pwm(PWMSERVOMID)
    {
        _serial_path = serial_path;
        _bps = bps;
    }
    ~SerialInterface() {}

    void set_control(float speed, uint16_t servo_pwm)
    {
        _speed = speed;
        _servo_pwm = servo_pwm;
    }

    void buzzerSound(unsigned char sound)
    {
        if(_sound == 0)
        {
            _sound = sound;
        }
    }

    void set_PID(float Kp, float Ki, float Kd)
    {
        _driver->PID_init(Kp, Ki, Kd);
    }

    int open()
    {
        return _open();
    }

    void Start()
    {
        _loop = true;
        send();
        recv();
    }

    void Stop()
    {
        _loop = false;
        _thread_send->join();
        _thread_recv->join();
        _driver->close();
    }


    void send()
    {
        _thread_send = std::make_unique<std::thread>([this]{
            while(_loop)
            {
                float speed = _speed;
                uint16_t servo_pwm = _servo_pwm;
                _driver->carControl(speed, servo_pwm);
                if(_sound)
                {
                    _driver->buzzerSound(_sound);
                    _sound = 0;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));

            }
        });
    }

    void recv()
    {
        _thread_recv = std::make_unique<std::thread>([this]{
            while(_loop)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        });
    }

private:
    int _open()
    {
        _driver = std::make_shared<Driver>(_serial_path, _bps);
        if (_driver == nullptr)
        {
            std::cout << "Create uart-driver error!" << std::endl;
            return -1;
        }
        // 串口初始化，打开串口设备及配置串口数据格式
        int ret = _driver->open();
        if (ret != 0)
        {
            std::cout << "Uart open failed!" << std::endl;
            return -1;
        }
        return 0;
    }
    bool _loop;
    std::string _serial_path;
    LibSerial::BaudRate _bps;
    std::shared_ptr<Driver> _driver;
    unsigned char _sound;
    float _speed;
    uint16_t _servo_pwm;
    float _recvSpeed;

    std::mutex _mutex;
    std::condition_variable cond_;
    std::unique_ptr<std::thread> _thread_send;
    std::unique_ptr<std::thread> _thread_recv;
};