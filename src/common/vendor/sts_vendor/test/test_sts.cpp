#include <gtest/gtest.h>
#include <sts_vendor/SCServo.hpp>

#include <array>
#include <iostream>
#include <thread>
#include <chrono>

// 测试1：验证库能否加载
TEST(STSVendorTest, library_loaded) {
    SMS_STS sms;
    SUCCEED();
}

// 测试2：完整硬件测试（打开串口 → Ping → 设置模式 → 转动电机）
TEST(STSVendorTest, device_full_test) {
    SMS_STS sms;

    const char* port = "/dev/ttyACM0"; // 根据实际设备修改
    int baud = 1000000;

    if (!sms.begin(baud, port)) {
        GTEST_SKIP() << "Cannot open serial port " << port;
    }

    // 要检测的 ID 列表
    std::array<uint8_t, 6> motor_ids = {11, 12, 13, 14, 15, 16};
    bool any_ok = false;

    // --- 1. Ping 所有电机 ---
    for (auto id : motor_ids) {
        uint8_t ping_id = sms.Ping(id);
        if (ping_id == id) {
            any_ok = true;
            std::cout << "[STS Test] Ping ID " << (int)id << " success." << std::endl;
        } else {
            std::cout << "[STS Test] Ping ID " << (int)id << " failed." << std::endl;
        }
    }

    if (!any_ok) {
        sms.end();
        GTEST_SKIP() << "No STS device responded on IDs 11-16.";
    }

    // --- 2. 切换到 Servo 模式（所有能 Ping 通的电机） ---
    for (auto id : motor_ids) {
        if (sms.Ping(id) == id) {
            sms.ServoMode(id);
            std::cout << "[STS Test] ID " << (int)id << " set to ServoMode." << std::endl;
        }
    }

    // 准备位置、速度、加速度数组
    std::array<uint16_t, 6> positions;
    std::array<uint16_t, 6> speeds;
    std::array<uint8_t, 6> accs;
    speeds.fill(0);
    accs.fill(0);

    // --- 3. 同步写入位置 4095（约 360°），让电机转动 ---
    positions.fill(4095);
    sms.SyncWritePosEx(
        motor_ids.data(),
        motor_ids.size(),
        reinterpret_cast<int16_t*>(positions.data()),
        reinterpret_cast<uint16_t*>(speeds.data()),
        accs.data()
    );
    std::cout << "[STS Test] Motor move to 360° (position 4095)." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // --- 4. 同步写入位置 0（回到 0°） ---
    positions.fill(0);
    sms.SyncWritePosEx(
        motor_ids.data(),
        motor_ids.size(),
        reinterpret_cast<int16_t*>(positions.data()),
        reinterpret_cast<uint16_t*>(speeds.data()),
        accs.data()
    );
    std::cout << "[STS Test] Motor back to 0° (position 0)." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    sms.end();

    std::cout << "[STS Test] All motors tested successfully." << std::endl;
    SUCCEED();
}

// 测试3：常量检查
TEST(STSVendorTest, constants_defined) {
    EXPECT_EQ(SMS_STS_ID, 5);
    EXPECT_EQ(SMS_STS_GOAL_POSITION_L, 42);
    EXPECT_EQ(SMS_STS_PRESENT_POSITION_L, 56);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
