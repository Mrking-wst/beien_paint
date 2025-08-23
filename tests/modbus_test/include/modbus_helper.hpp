#ifndef MODBUS_HELPER_HPP
#define MODBUS_HELPER_HPP

#include <string>
#include <functional>
#include <vector>

#include "modbus/modbus.h"

namespace modbus_wrapper
{
    class ModbusHelper
    {
    public:
        using DataReceived = std::function<void(const std::vector<uint16_t> &)>;

    public:
        ModbusHelper();
        ~ModbusHelper();
        std::string get_ip_address();
        void set_ip_address(const std::string &ip);
        int get_port();
        void set_port(const int port);
        int get_reading_interval_ms();
        void set_reading_interval_ms(const int interval);
        int get_reconnect_interval_ms();
        void set_reconnect_interval_ms(const int interval);
        int get_holding_register_num();
        void set_holding_register_num(const int nums);
        void StartReading();
        void StopReading();
        void BindingDataReceived(DataReceived dr);

    private:
        void RaiseDataReceived(const std::vector<uint16_t> &data);
        bool IsValidIPv4(const std::string &ip);
        void ReadingLoop();
        void CloseContext();

    private:
        std::thread reading_thread_;
        modbus_t *ctx_;
        std::string ip_address_;
        int port_;
        int reading_interval_ms_;
        int reconnect_interval_ms;
        DataReceived data_received_;
        bool is_reading_;
        int holding_register_num_;
    };
} // namespace modbus_wrapper

#endif //  MODBUS_HELPER_HPP