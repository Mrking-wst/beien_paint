#ifndef PLC_COMMUNICATE_NODE_HPP
#define PLC_COMMUNICATE_NODE_HPP

#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "modbus/modbus.h"

namespace PlcCommunicate
{
    using std_msgs::msg::UInt16MultiArray;

    class PlcCommunicateNode
        : rclcpp::Node
    {
    public:
        PlcCommunicateNode(const std::string &nodeName);
        void Dispose();

    private:
        int Connect();
        void SourceDataReceivedCallback(const UInt16MultiArray &sourse_data);
        void ReconnectLoop();

    private:
        modbus_t *ctx_;
        rclcpp::Publisher<UInt16MultiArray>::SharedPtr source_data_pub_;
        rclcpp::Subscription<UInt16MultiArray>::SharedPtr source_data_sub_;
        rclcpp::TimerBase::SharedPtr start_time_;
        std::string plc_ip_;
        int plc_port_;
        int read_interval_;
        int reconnect_interval_;
        int holding_register_num_;
        int response_timeout_;
    };
} // namespace PlcCommunicate

#endif //  PLC_COMMUNICATE_NODE_HPP