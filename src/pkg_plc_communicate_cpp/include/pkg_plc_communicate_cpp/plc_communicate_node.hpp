#ifndef PLC_COMMUNICATE_NODE_HPP
#define PLC_COMMUNICATE_NODE_HPP

#include <string>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "modbus/modbus.h"

namespace PlcCommunicate
{
    using std_msgs::msg::UInt16MultiArray;

    class PlcCommunicateNode
        : public rclcpp::Node
    {
    public:
        PlcCommunicateNode(const std::string &nodeName);
        void Dispose();

    private:
        void ConnectModbus();
        void PollModbus();
        void TryReconnectModbus();
        void SourceDataReceivedCallback(const UInt16MultiArray &sourse_data);

    private:
        modbus_t *ctx_;
        rclcpp::Publisher<UInt16MultiArray>::SharedPtr source_data_pub_;
        rclcpp::Subscription<UInt16MultiArray>::SharedPtr source_data_sub_;
        rclcpp::TimerBase::SharedPtr poll_time_;
        rclcpp::TimerBase::SharedPtr reconnect_time_;
        std::thread reconnect_loop_thread_;
        std::string plc_ip_;
        int plc_port_;
        int read_interval_;
        int reconnect_interval_;
        int read_holding_register_start_address_;
        int read_holding_register_num_;
        int write_holding_register_start_address_;
        int write_holding_register_num_;
        int response_timeout_;
        bool is_connected_;
    };
} // namespace PlcCommunicate

#endif //  PLC_COMMUNICATE_NODE_HPP