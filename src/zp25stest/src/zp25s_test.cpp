
#include "zp25stest/zp25s_driver.hpp"
#include <thread>

int main() {
    ZP25SDriver zp25("/dev/ttyUSB0", 115200);
    zp25.init();

    std::string cmd_1 = "#004P0500T1000!";
    std::cout << " 发送指令: " << cmd_1 << std::endl;
    zp25.send_test(cmd_1);

    std::string cmd_2 = "#002PRAD!";
    std::cout << " 发送指令: " << cmd_2 << ", 等待接收..." << std::endl;
    zp25.send_receive_test(cmd_2);


    auto start = std::chrono::steady_clock::now(); // 发送前时间点

    // zp25.setTargetPositionRadian("001", 0);
}

// int main() {
//     std::string port = "/dev/ttyUSB0";
//     if (access(port.c_str(), R_OK | W_OK) == -1) {
//         std::cerr << "无串口权限... " << port << std::endl;
//         return -1;
//     }
//
//     // 1. 创建 IoContext
//     drivers::common::IoContext io_context(1);
//
//     // 2. 创建 SerialDriver（注意：构造函数只保存引用，不打开设备）
//     drivers::serial_driver::SerialDriver driver(io_context);
//
//     // 3. 配置参数（8N1，115200）
//     drivers::serial_driver::SerialPortConfig config(
//         115200,
//         drivers::serial_driver::FlowControl::NONE,
//         drivers::serial_driver::Parity::NONE,
//         drivers::serial_driver::StopBits::ONE
//     );
//
//     // 4. 初始化串口（内部创建 SerialPort 对象）
//     driver.init_port(port, config);
//
//     // 5. 获取 SerialPort 指针并打开设备
//     auto port_ptr = driver.port();
//     try {
//         port_ptr->open();
//         std::cout << "✅ 串口打开成功" << std::endl;
//     }
//     catch (const std::exception& e) {
//         std::cerr << "❌ 打开失败: " << e.what() << std::endl;
//         return -1;
//     }
//
//     // 6. 发送指令（例如查询 ID=0 位置）
//     std::string cmd = "#004P2500T1000!";
//     std::vector<uint8_t> data(cmd.begin(), cmd.end());
//
//     auto start = std::chrono::steady_clock::now(); // 发送前时间点
//
//     size_t sent = port_ptr->send(data);
//     std::cout << "发送 " << sent << " 字节" << std::endl;
//
//     // 7. 同步读取回复（阻塞，直到收到数据）
//     std::vector<uint8_t> buffer(256);
//     size_t received = port_ptr->receive(buffer);
//
//     auto end = std::chrono::steady_clock::now(); // 收到后时间点
//     auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
//
//     if (received > 0) {
//         std::string response(buffer.begin(), buffer.begin() + received);
//         std::cout << "收到回复: " << response << std::endl;
//         std::cout << "往返耗时: " << elapsed_ms << " ms" << std::endl;
//     } else {
//         std::cerr << "未收到数据" << std::endl;
//     }
//
//     // 8. 关闭
//     port_ptr->close();
//     return 0;
// }
