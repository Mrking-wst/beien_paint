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
        
    }
} // namespace PlcCommunicate
