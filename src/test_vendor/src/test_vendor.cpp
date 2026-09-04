#include <fd_vendor/dhd.hpp>
#include <fd_vendor/drd.hpp>
#include <sts_vendor/SCServo.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <array>
#include <string>

bool initFDDevice(char& outDeviceID) {
    // 1. 打开第一台可用设备
    if (dhdOpenID(0) < 0) {
        std::cerr << "[FD Init] Failed to open device: " << dhdErrorGetLastStr() << std::endl;
        return false;
    }

    // 2. 获取当前激活的设备ID
    char id = dhdGetDeviceID();

    // 3. 检查设备是否被 DRD 机器人库支持
    if (!drdIsSupported()) {
        std::cerr << "[FD Init] Unsupported device type: " << dhdGetSystemName() << std::endl;
        dhdClose();
        return false;
    }

    std::cout << "[FD Init] " << dhdGetSystemName() << " detected (ID=" << (int)id << ")" << std::endl;

    // 4. 自动初始化所有活动轴
    if (drdAutoInit() < 0) {
        std::cerr << "[FD Init] Auto-init failed: " << dhdErrorGetLastStr() << std::endl;
        dhdClose();
        return false;
    }

    // 5. 校验初始化结果
    if (drdCheckInit() < 0) {
        std::cerr << "[FD Init] Init validation failed: " << dhdErrorGetLastStr() << std::endl;
        dhdClose();
        return false;
    }

    // 6. 停止内部调节线程，但保持力输出使能
    //    ⚠️ 这一步至关重要：停止后你才能用自己的控制循环接管力渲染
    if (drdStop(true) < 0) {
        std::cerr << "[FD Init] Failed to stop regulation: " << dhdErrorGetLastStr() << std::endl;
        dhdClose();
        return false;
    }

    std::cout << "[FD Init] Device successfully initialized." << std::endl;
    outDeviceID = id;
    return true;
}


int main() {
    // Force Dimension SDK version (already verified)
    int major, minor, release, revision;
    dhdGetSDKVersion(&major, &minor, &release, &revision);
    printf("Force Dimension SDK, version %i.%i.%i\n", major, minor, release);

    if (drdOpen() < 0) {
        std::cout << "error: failed to open device (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    uint16_t serial = 0;
    dhdGetSerialNumber(&serial, 0);
    std::cout << "Device serial number: " << serial << std::endl;

    // ST3215舵机控制
    SMS_STS sms_sts;
    const char* port = "/dev/ttyACM0"; // 根据实际修改
    int baud = 1000000;

    if (!sms_sts.begin(baud, port)) {
        std::cerr << "串口初始化失败" << std::endl;
        return -1;
    }
    std::cout << "串口已打开" << std::endl;

    // 6个舵机ID 11~16
    std::array<uint8_t, 6> motor_ids = {11, 12, 13, 14, 15, 16};
    // Ping检测所有舵机
    for (auto id : motor_ids) {
        uint8_t ping_id = sms_sts.Ping(id);
        if (ping_id != id) {
            std::cerr << "舵机ID " << (int)id << " Ping失败" << std::endl;
            sms_sts.end();
            return -1;
        }
        std::cout << "舵机ID " << (int)id << " 在线" << std::endl;
    }

    // 设为位置模式
    for (auto id : motor_ids) {
        sms_sts.ServoMode(id);
    }
    std::cout << "所有舵机已设为位置模式" << std::endl;

    // 准备同步写数据
    std::array<uint16_t, 6> positions;
    std::array<uint16_t, 6> speeds; // 速度
    std::array<uint8_t, 6> accs; // 加速度
    speeds.fill(0); // 默认速度
    accs.fill(0); // 默认加速度

    // 第一次同步写：全部转到135°
    positions.fill(4095);
    sms_sts.SyncWritePosEx(motor_ids.data(), motor_ids.size(),
                           reinterpret_cast<int16_t*>(positions.data()),
                           reinterpret_cast<uint16_t*>(speeds.data()),
                           accs.data());
    std::cout << "已发送指令：所有舵机转到360°" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // 等待转动

    // 第二次同步写：全部转回0°
    positions.fill(0);
    sms_sts.SyncWritePosEx(motor_ids.data(), motor_ids.size(),
                           reinterpret_cast<int16_t*>(positions.data()),
                           reinterpret_cast<uint16_t*>(speeds.data()),
                           accs.data());
    std::cout << "已发送指令：所有舵机转回0°" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    sms_sts.end();
    std::cout << "测试完成" << std::endl;
    return 0;
}



