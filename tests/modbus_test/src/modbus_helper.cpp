#include <thread>
#include <string>
#include <functional>
#include <vector>
#include <sstream>
#include <cctype>
#include <iostream>

#include "modbus_helper.hpp"
#include "modbus/modbus.h"

namespace modbus_wrapper
{
    ModbusHelper::ModbusHelper()
    {
        this->is_reading_ = false;
    }

    ModbusHelper::~ModbusHelper()
    {
        this->StopReading();
        if (this->reading_thread_.joinable())
            this->reading_thread_.join();
    }

    std::string
    ModbusHelper::get_ip_address()
    {
        return this->ip_address_;
    }

    int
    ModbusHelper::get_port()
    {
        return this->port_;
    }

    int
    ModbusHelper::get_reading_interval_ms()
    {
        return this->reading_interval_ms_;
    }

    int
    ModbusHelper::get_reconnect_interval_ms()
    {
        return this->reconnect_interval_ms;
    }

    int
    ModbusHelper::get_holding_register_num()
    {
        return this->holding_register_num_;
    }

    timeval
    ModbusHelper::get_response_timeout()
    {
        return this->response_timeout_;
    }

    void
    ModbusHelper::set_response_timeout(const timeval response_timeout)
    {
        this->response_timeout_ = response_timeout;
    }

    void
    ModbusHelper::set_ip_address(const std::string &ip)
    {
        if (!this->IsValidIPv4(ip))
        {
            std::cout << "ip 地址格式错误";
        }

        this->ip_address_ = ip;
    }

    void
    ModbusHelper::set_port(const int port)
    {
        if (port > 65535 || port < 0)
        {
            std::cout << "port 端口号错误，不在0～65535范围内";
        }
        this->port_ = port;
    }

    void
    ModbusHelper::set_reading_interval_ms(const int interval)
    {
        this->reading_interval_ms_ = interval;
    }

    void
    ModbusHelper::set_reconnect_interval_ms(const int interval)
    {
        this->reconnect_interval_ms = interval;
    }

    void
    ModbusHelper::set_holding_register_num(const int registernum)
    {
        this->holding_register_num_ = registernum;
    }

    bool
    ModbusHelper::IsValidIPv4(const std::string &ip)
    {
        std::stringstream ss(ip);
        std::vector<std::string> segments;
        std::string segment;

        while (std::getline(ss, segment, '.'))
        {
            segments.push_back(segment);
        }

        if (segments.size() != 4)
            return false;

        for (const auto &seg : segments)
        {
            if (seg.empty() || seg.size() > 3)
                return false;

            for (char c : seg)
            {
                if (!std::isdigit(c))
                    return false;
            }

            int num = std::stoi(seg);
            if (num < 0 || num > 255)
                return false;

            if (seg.size() > 1 && seg[0] == '0')
                return false;
        }

        return true;
    }

    void
    ModbusHelper::StartReading()
    {
        if (this->is_reading_)
        {
            std::cout << "系统正在读取数据..." << std::endl;
            return;
        }

        this->is_reading_ = true;
        std::cout << "-读取任务启动" << std::endl;

        this->reading_thread_ = std::thread(std::bind(&ModbusHelper::ReadingLoop, this));
    }

    void
    ModbusHelper::StopReading()
    {
        this->is_reading_ = false;
        if (this->reading_thread_.joinable())
            this->reading_thread_.join();

        this->CloseContext();
    }

    void
    ModbusHelper::BindingDataReceived(DataReceived dr)
    {
        this->data_received_ = std::move(dr);
    }

    void
    ModbusHelper::RaiseDataReceived(const std::vector<uint16_t> &data)
    {
        if (this->data_received_ != nullptr)
            this->data_received_(data);
    }

    void
    ModbusHelper::ReadingLoop()
    {
        while (this->is_reading_)
        {
            this->ctx_ = modbus_new_tcp(this->ip_address_.c_str(), this->port_);
            if (ctx_ == nullptr)
            {
                std::cerr << "无法使用 " << this->ip_address_ << " " << this->port_ << " Tcp网络进行Modbus通信" << std::endl;
                return;
            }
            std::cout << "--等待建立连接" << std::endl;

            //  设置超时时间
            // timeval response_timeout;
            // response_timeout.tv_sec = 0;
            // response_timeout.tv_usec = 1000000; //  1000ms
            modbus_set_response_timeout(this->ctx_,response_timeout_.tv_sec,response_timeout_.tv_usec);

            if (modbus_connect(this->ctx_) == -1)
            {
                int err = errno;
                std::cerr << "[connect error] " << modbus_strerror(err) << "\n";
                modbus_free(this->ctx_);
                ctx_ = nullptr;
                std::this_thread::sleep_for(std::chrono::milliseconds(this->reconnect_interval_ms));
                continue;
            }

            uint16_t datas[this->holding_register_num_];
            while (this->is_reading_)
            {
                int count = modbus_read_registers(this->ctx_, 0, this->holding_register_num_, datas);
                // std::cout << "---读取寄存器数量：" << count << std::endl;
                if (count == -1)
                {
                    int err = errno;
                    std::cerr << "[reading error] " << modbus_strerror(err) << "\n";
                    this->CloseContext();
                    break;
                }

                if (count < this->holding_register_num_)
                {
                    std::cerr << "[reading error] " << "读取数据字节数量不正确" << "\n";
                }
                else
                {
                    std::vector<uint16_t> d(datas, datas + this->holding_register_num_);
                    this->RaiseDataReceived(d);
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(this->reading_interval_ms_));
            }
        }
    }

    void
    ModbusHelper::CloseContext()
    {
        if (this->ctx_)
        {
            modbus_close(this->ctx_);
            modbus_free(this->ctx_);
            this->ctx_ = nullptr;
        }
    }

} // namespace modbus_wrapper
