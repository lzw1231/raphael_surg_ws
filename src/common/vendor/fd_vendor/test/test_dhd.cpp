#include <gtest/gtest.h>
#include <fd_vendor/dhd.hpp>

// 测试1：检查 SDK 版本是否能正确读取
TEST(FDVendorTest, sdk_version) {
    int major = 0, minor = 0, release = 0, revision = 0;
    dhdGetSDKVersion(&major, &minor, &release, &revision);
    // 只要版本号大于0，说明函数调用成功
    EXPECT_GT(major, 0);
}

// 测试2：检查是否有设备连接（物理设备或仿真）
TEST(FDVendorTest, device_open) {
    // 尝试打开第一个设备
    int id = dhdOpenID(0);
    if (id < 0) {
        GTEST_SKIP() << "No Force Dimension device connected, skipping test.";
    }
    // 如果有设备，检查是否能正常关闭
    int result = dhdClose(id);
    EXPECT_EQ(result, 0); // dhdClose 成功返回 0
}

// 测试3：简单的库加载测试（只调用函数，不检查错误）
TEST(FDVendorTest, library_loaded) {
    int major = 0, minor = 0, release = 0, revision = 0;
    dhdGetSDKVersion(&major, &minor, &release, &revision);
    // 能执行到这里就说明库加载正常
    EXPECT_TRUE(true);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
