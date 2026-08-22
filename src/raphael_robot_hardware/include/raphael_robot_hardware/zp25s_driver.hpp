#ifndef ZP25S_DRIVER_HPP
#define ZP25S_DRIVER_HPP

#include <string>
#include <mutex>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <regex>
#include <cstring>
#include <cerrno>
#include <iostream>
#include <chrono>
#include <thread>

// ZP25S 参数
constexpr int ZP25S_BAUDRATE = 115200;
constexpr int ZP25S_PWM_MIN = 500; // PWM最小值
constexpr int ZP25S_PWM_MAX = 2500; // PWM最大值
constexpr int ZP25S_PWM_ZERO = 1500; // 中位PWM
constexpr int ZP25S_MOVE_TIME_MS = 0; // 运动时间 0=最快
constexpr std::chrono::milliseconds ZP25S_CMD_WAIT_50MS{50};
constexpr std::chrono::milliseconds ZP25S_CMD_WAIT_10MS{10};

constexpr double M_PI_LOCAL = 3.14159265358979323846;

/**
 * 注意角度换算假设：
 * 假设PWM [500 ~ 2500] 对应转角 [0 ~ 2π] (360°)
 * 如果你的舵机实际机械限位不是一圈，此换算不成立！
 */
constexpr double ZP25S_PWM_RANGE = static_cast<double>(ZP25S_PWM_MAX - ZP25S_PWM_MIN);
constexpr double ZP25S_PWM_TO_RAD = (2.0 * M_PI_LOCAL) / ZP25S_PWM_RANGE;
constexpr double ZP25S_RAD_TO_PWM = ZP25S_PWM_RANGE / (2.0 * M_PI_LOCAL);

class ZP25SDriver {
public:
    explicit ZP25SDriver(std::string device_name) : dev_name_(std::move(device_name)) {}

    ~ZP25SDriver() {
        close();
    }

    // 打开串口并配置为ZP25S原始串口模式
    int init() {
        // 保留 O_NONBLOCK，用于用户态超时read，不要fcntl清除非阻塞标记
        fd_ = ::open(dev_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            std::cerr << "ZP25S open port failed: " << dev_name_ << std::endl;
            return -1;
        }

        struct termios tty{};
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(fd_, &tty) != 0) {
            perror("tcgetattr");
            ::close(fd_);
            fd_ = -1;
            return -1;
        }

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        // 8N1
        tty.c_cflag &= ~PARENB; // 无校验
        tty.c_cflag &= ~CSTOPB; // 1停止位
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8; // 8数据位
        tty.c_cflag |= CREAD | CLOCAL; // 开启接收，忽略Modem控制线
        tty.c_cflag &= ~CRTSCTS; // 关闭硬件流控

        // 原始模式，关闭行处理、回显、信号
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // 关闭软件流控
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;

        // VMIN=0 VTIME=0：非阻塞termios，read立刻返回，上层自己做超时
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            perror("tcsetattr");
            ::close(fd_);
            fd_ = -1;
            return -1;
        }

        tcflush(fd_, TCIOFLUSH);
        std::cout << "ZP25S port opened: " << dev_name_ << " at 115200" << std::endl;
        return 0;
    }

    /**
     * @brief 激活舵机脉冲输出（作为主机向外输出脉冲给步进驱动器）
     * @param servo_id 3位字符串ID，如 "000" "001"
     */
    void activateWithPositionMode(const std::string& servo_id) {
        send_only("#" + servo_id + "PMOD2!");
        std::this_thread::sleep_for(ZP25S_CMD_WAIT_50MS);

        send_only("#" + servo_id + "PULK!"); // 使能脉冲输出
        std::this_thread::sleep_for(ZP25S_CMD_WAIT_10MS);

        send_only("#" + servo_id + "PULM!"); // 设置脉冲输出模式(PULM指令)
        std::this_thread::sleep_for(ZP25S_CMD_WAIT_10MS);
    }

    /**
     * @brief 关闭脉冲输出
     * @param servo_id 3位字符串ID
     */
    void deactivate(const std::string& servo_id) {
        send_only("#" + servo_id + "PULR!"); // PULR关闭脉冲输出
        std::this_thread::sleep_for(ZP25S_CMD_WAIT_10MS);
    }

    /**
     * @brief 获取舵机位置，返回弧度；内部将PWM值换算弧度
     * @param servo_id 3位字符串ID
     * @return 弧度，出错返回0.0
     */
    double getPositionRadian(const std::string& servo_id) {
        std::string resp = send_and_receive("#" + servo_id + "PRAD!", 40);
        std::regex re(R"(#(\d{3})P(\d{4})!)");
        std::smatch match;

        if (std::regex_search(resp, match, re)) {
            std::string resp_id = match[1].str();
            if (resp_id != servo_id) {
                std::cerr << "ZP25S: position response ID mismatch: expected "
                    << servo_id << ", got " << resp_id << std::endl;
                return 0.0;
            }
            int pwm = std::stoi(match[2].str());
            return static_cast<double>(pwm - ZP25S_PWM_MIN) * ZP25S_PWM_TO_RAD;
        }
        std::cerr << "ZP25S: failed to parse position response: " << resp << std::endl;
        return 0.0;
    }

    /**
     * @brief 设置目标位置(弧度)，使用默认运动时间
     */
    void setTargetPositionRadian(const std::string& servo_id, double position) {
        setTargetPositionRadian(servo_id, position, ZP25S_MOVE_TIME_MS);
    }

    /**
     * @brief 设置目标位置(弧度)，指定运动时间ms
     */
    void setTargetPositionRadian(const std::string& servo_id, double position, int move_time_ms) {
        double pwm_val = static_cast<double>(ZP25S_PWM_MIN) + position * ZP25S_RAD_TO_PWM;
        int pwm = clampPwm(static_cast<int>(std::round(pwm_val)));

        std::ostringstream oss;
        oss << "#" << servo_id
            << "P" << std::setw(4) << std::setfill('0') << pwm
            << "T" << std::setw(4) << std::setfill('0') << move_time_ms
            << "!";
        send_only(oss.str());
    }

    // 手动关闭串口
    void close() {
        std::lock_guard<std::mutex> lock(serial_mtx_);
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

private:
    std::string dev_name_;
    int fd_{-1};
    std::mutex serial_mtx_;

    int clampPwm(int pwm) {
        if (pwm < ZP25S_PWM_MIN) pwm = ZP25S_PWM_MIN;
        if (pwm > ZP25S_PWM_MAX) pwm = ZP25S_PWM_MAX;
        return pwm;
    }

    // 只发送，不等待应答
    void send_only(const std::string& cmd) {
        std::lock_guard<std::mutex> lock(serial_mtx_);
        if (fd_ < 0) {
            std::cerr << "ZP25S: serial port not open" << std::endl;
            return;
        }

        ssize_t written = ::write(fd_, cmd.c_str(), cmd.size());
        if (written < 0) {
            perror("ZP25S write failed");
        } else if (static_cast<size_t>(written) != cmd.size()) {
            std::cerr << "ZP25S: incomplete write, expected " << cmd.size()
                << " bytes, wrote " << written << std::endl;
        }
        tcdrain(fd_);
    }

    /**
     * @brief 发送命令，等待以'!'结尾应答，超时返回空字符串
     * @param cmd 完整指令
     * @param timeout_ms 超时毫秒
     * @return 应答字符串，不含截断；超时/错误返回""
     */
    std::string send_and_receive(const std::string& cmd, int timeout_ms) {
        std::lock_guard<std::mutex> lock(serial_mtx_);
        if (fd_ < 0) {
            std::cerr << "ZP25S: serial port not open" << std::endl;
            return "";
        }

        ssize_t written = ::write(fd_, cmd.c_str(), cmd.size());
        if (static_cast<size_t>(written) != cmd.size()) {
            std::cerr << "ZP25S: send failed for cmd: " << cmd << std::endl;
            return "";
        }
        tcdrain(fd_);

        std::string response;
        auto start = std::chrono::steady_clock::now();
        while (true) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start).count();
            if (elapsed > timeout_ms) {
                std::cerr << "ZP25S: read timeout for cmd: " << cmd << std::endl;
                return "";
            }

            char ch;
            ssize_t n = ::read(fd_, &ch, 1);
            if (n == 1) {
                response.push_back(ch);
                if (ch == '!') {
                    break;
                }
            } else if (n == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } else {
                // n <0, EAGAIN代表无数据，继续循环；其他错误直接退出
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    perror("ZP25S read error");
                    return "";
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        return response;
    }
};

#endif // ZP25S_DRIVER_HPP
