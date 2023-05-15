#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>

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
    void run()
    {
        _thread = std::make_shared<std::thread>([this]{
            while(_loop)
            {

            }
        })
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
    }
    bool _loop;
    std::string _serial_path;
    LibSerial::BaudRate _bps;
    std::shared_ptr<Driver> _driver;
    float _speed;
    uint16_t _servo_pwm

    std::unique_ptr<std::thread> _thread;
};