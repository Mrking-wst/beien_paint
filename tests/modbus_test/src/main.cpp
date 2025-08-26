#include <string>
#include <iostream>
#include <thread>
#include <csignal>
#include <atomic>
#include <ctime>
#include <iomanip>

#include "modbus/modbus.h"
#include "modbus/modbus-tcp.h"
#include "modbus_helper.hpp"

void PrintData(uint16_t *data)
{
    std::cout << "registers: ";
    for (int i = 0; i < sizeof(data) / sizeof(uint16_t); i++)
    {
        std::cout << data[i] << " ";
    }
    std::cout << "\n";
}

void PrintData(std::vector<uint16_t> data)
{
    auto now = std::chrono::system_clock::now();
    auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    std::cout << "[" << timestamp_ms << "]" << " registers: ";
    // std::time_t t = std::chrono::system_clock::to_time_t(now);
    // std::cout << "[" << std::put_time(std::localtime(&t), "%Y-%m-%d %H:%M:%S") << "]" << " " << std::this_thread::get_id() << " registers: ";
    for (int i = 0; i < data.size(); i++)
    {
        std::cout << data[i] << " ";
    }
    std::cout << "\n";
}

using modbus_wrapper::ModbusHelper;

std::atomic<bool> stop_flag(false);
ModbusHelper *g_modbus = nullptr; // 全局指针，信号处理时访问

// 捕获 Ctrl+C
void signalHandler(int signum)
{
    std::cout << "\n[INFO] Caught signal " << signum << " (Ctrl+C pressed)" << std::endl;
    stop_flag = true;

    if (g_modbus)
    {
        std::cout << "[INFO] Stopping Modbus connection..." << std::endl;
        g_modbus->StopReading(); // 优雅停止循环线程
    }
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    // 注册 Ctrl+C 处理函数
    std::signal(SIGINT, signalHandler);
    std::cout << "modbus tcp test with cpp" << std::endl;
    // modbus_t *cts;
    // cts = modbus_new_tcp("192.168.0.10", 502);
    // if (modbus_connect(cts) == -1)
    // {
    //     std::cout << "[plc_connect] plc connect error: " << modbus_strerror(errno) << std::endl;
    //     modbus_free(cts);
    //     return -1;
    // }

    // uint16_t registers[20];
    // int res = modbus_read_registers(cts, 0, 20, registers);
    // std::cout << "读取寄存器的结果：" << res << std::endl;
    // PrintData(registers);
    // modbus_close(cts);
    // modbus_free(cts);

    ModbusHelper modbus;
    g_modbus = &modbus;
    modbus.set_ip_address("192.168.0.10");
    modbus.set_port(502);
    modbus.set_reading_interval_ms(80);
    modbus.set_reconnect_interval_ms(3000);
    modbus.set_holding_register_num(20);
    timeval time_out;
    time_out.tv_sec = 0;
    time_out.tv_usec = 1000000;
    modbus.set_response_timeout(time_out);
    modbus.BindingDataReceived([](const std::vector<uint16_t> &datas)
                               { PrintData(datas); });
    modbus.StartReading();

    // 主线程等待 Ctrl+C
    while (!stop_flag)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::cout << "[INFO] Program exiting gracefully." << std::endl;

    return 0;
}