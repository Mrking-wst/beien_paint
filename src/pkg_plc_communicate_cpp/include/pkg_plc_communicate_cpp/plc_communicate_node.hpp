#ifndef PLC_COMMUNICATE_NODE_HPP
#define PLC_COMMUNICATE_NODE_HPP

#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "modbus/modbus.h"
#include "modbus/modbus-tcp.h"

namespace PlcCommunicate
{
    using std_msgs::msg::UInt16MultiArray;

    class PlcCommunicateNode
        : rclcpp::Node
    {
    public:
        PlcCommunicateNode(const std::string &nodeName);

    private:
        int connect();

    private:
        std::shared_ptr<modbus_t> cts_;
        rclcpp::Publisher<UInt16MultiArray>::SharedPtr source_data_pub_;
        rclcpp::Subscription<UInt16MultiArray>::SharedPtr source_data_sub_;
        rclcpp::TimerBase::SharedPtr pub_interval_;
        std::string plcIp_;
        int plcPort_;
    };
} // namespace PlcCommunicate

#endif //  PLC_COMMUNICATE_NODE_HPP