#pragma once

#include <string>
#include <serial_driver/serial_driver.hpp>
#include <io_context/io_context.hpp>
#include <serial_driver/serial_port.hpp>
#include <utility>
#include <iostream>
#include <cmath>
#include <thread>

class ZP25SDriver {
public:
    ZP25SDriver(std::string device_name, uint32_t baud_rate) : io_context_(1), driver_(io_context_) {
        device_name_ = std::move(device_name);
        baud_rate_ = baud_rate;
        is_open_ = false;
        std::cout << "ZP25SDriver constructed" << std::endl;
    };

    ~ZP25SDriver() {
        if (port_ptr_->is_open()) {
            port_ptr_->close();
        }
    };


    //////////////////////////  两个测试函数  ////////////////////////////////
    void send_test(const std::string& command) {
        std::vector<uint8_t> data(command.begin(), command.end());
        port_ptr_->send(data);
    };

    void send_receive_test(const std::string& command) {
        // 1. 发送命令
        std::vector<uint8_t> data(command.begin(), command.end());
        port_ptr_->send(data);

        // 2. 等待舵机准备回复（发送指令到回复之间需要几毫秒，但为了保险给 50ms）
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        // 3. 循环接收，直到遇到 '!'
        std::vector<uint8_t> recv_buf;
        while (true) {
            std::vector<uint8_t> chunk(64);
            size_t len = port_ptr_->receive(chunk);
            if (len > 0) {
                recv_buf.insert(recv_buf.end(), chunk.begin(), chunk.begin() + len);
                if (std::find(recv_buf.begin(), recv_buf.end(), '!') != recv_buf.end()) {
                    break; // 收到完整帧
                }
            } else {
                // 如果没有数据，稍微等一等（但 read_some 通常阻塞，不会返回0，除非设置了非阻塞）
                // 这里为了保险，加个短暂休眠避免死循环
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        // 4. 打印结果
        std::string frame(recv_buf.begin(), recv_buf.end());
        std::cout << "Received: " << frame << std::endl;
    }

    ////////////////////////////////////////////////////////////////////

    int init() {
        drivers::serial_driver::SerialPortConfig config(
            baud_rate_,
            drivers::serial_driver::FlowControl::NONE,
            drivers::serial_driver::Parity::NONE,
            drivers::serial_driver::StopBits::ONE
        );

        // 4. 初始化串口（内部创建 SerialPort 对象）
        driver_.init_port(device_name_, config);

        // 5. 获取 SerialPort 指针并打开设备
        port_ptr_ = driver_.port();
        try {
            port_ptr_->open();
            is_open_ = true;
            std::cout << "Serial port " << device_name_ << " opened successfully at " << baud_rate_ << " baud." << std::endl;
            return 0;
        }
        catch (const std::exception& e) {
            std::cerr << "Failed to open serial port " << device_name_ << ": " << e.what() << std::endl;
            return -1;
        }
    }

    // 设置位置模式。模式1->顺时针，模式2->逆时针
    void activateWithPositionMode(const std::string& zp_id, const std::string& mode_num) {
        std::string command = "#" + zp_id + "PMOD" + mode_num + "!";
        std::vector<uint8_t> data(command.begin(), command.end());
        port_ptr_->send(data);
    }

    // 设置速度模式。  模式5->顺时针，模式6->逆时针
    void activateWithVelocityMode(const std::string& zp_id, const std::string& mod_num) {
        std::string command = "#" + zp_id + "PMOD" + mod_num + "!";
        std::vector<uint8_t> data(command.begin(), command.end());
        port_ptr_->send(data);
    }

    // 设置位置
    void setTargetPositionRadian(std::string zp_id, double arc) {
        // 线性映射：500 ~ 2500 对应 0 ~ 2π 弧度
        int pwm = static_cast<int>(500 + (arc / (2 * M_PI)) * 2000);
        // 边界保护
        if (pwm < 500) pwm = 500;
        if (pwm > 2500) pwm = 2500;

        // 构造PWM字符串（4位）
        std::string pwm_str = std::to_string(pwm);
        if (pwm_str.length() < 4) {
            pwm_str = std::string(4 - pwm_str.length(), '0') + pwm_str;
        }

        // 指令格式：#<ID>P<PWM>T0000! （时间设为0，立即执行）
        std::string command = "#" + zp_id + "P" + pwm_str + "T0000!";
        std::vector<uint8_t> data(command.begin(), command.end());
        port_ptr_->send(data);
    }

    // 设置速度。
    void setTargetVelocityRadian(std::string zp_id, double vel) {}

    // 获取位置
    double getTargetPositionRadian(std::string zp_id) {
        // 1. 构造读取指令
        std::string command = "#" + zp_id + "PRAD!";
        std::vector<uint8_t> cmd_data(command.begin(), command.end());
        port_ptr_->send(cmd_data);

        // 2. 异步，从串口读取反馈的信息
        port_ptr_->async_receive([this](const std::vector<uint8_t>& buffer, size_t bytes_transferred) {
                // 当串口收到数据时，这个 lambda 会被调用
                // buffer 里存放的是原始字节，有效数据长度是 bytes_transferred

                // 最简单的：打印出来（十六进制或字符串）
                std::cout << "Received " << bytes_transferred << " bytes: ";
                for (size_t i = 0; i < bytes_transferred; ++i) {
                    std::cout << std::hex << (int)buffer[i] << " ";
                }
                std::cout << std::endl;

                // 或者转成字符串（如果内容是 ASCII）
                // std::string str(buffer.begin(), buffer.begin() + bytes_transferred);
                // std::cout << "String: " << str << std::endl;

                // 您也可以把数据存到一个全局/成员变量，留待后续处理
                // 例如: this->raw_buffer_.insert(...);
            }
        );
    }

private:
    std::string device_name_;
    u_int32_t baud_rate_;
    bool is_open_;
    IoContext io_context_;
    drivers::serial_driver::SerialDriver driver_;
    std::shared_ptr<drivers::serial_driver::SerialPort> port_ptr_;
};




