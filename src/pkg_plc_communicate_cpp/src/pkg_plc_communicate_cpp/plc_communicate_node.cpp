#include <chrono>
#include <string>

#include "plc_communicate_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "modbus/modbus.h"
#include "modbus/modbus-tcp.h"

namespace PlcCommunicate
{
    PlcCommunicateNode::PlcCommunicateNode(const std::string &nodeName)
        : rclcpp::Node(nodeName)
    {
        this->declare_parameter<std::string>("plc_ip", "192.168.0.10");
        this->declare_parameter<int>("plc_port", 502);
        this->declare_parameter<int>("poll_rate", 10);

        this->get_parameter<std::string>("plc_ip", plc_ip_);
        this->get_parameter<int>("plc_port", plc_port_);
        this->get_parameter<int>("poll_rate", poll_rate_);
    }
} // namespace PlcCommunicate
