#ifndef PLC_MANAGER_NODE_HPP
#define PLC_MANAGER_NODE_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pkg_beien_paint_msgs/msg/joycon_left.hpp"
#include "pkg_beien_paint_msgs/msg/joycon_right.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

namespace BeienPaint
{
    using std_msgs::msg::UInt16MultiArray;

    class PlcManagerNode : public rclcpp::Node
    {
    public:
        PlcManagerNode(const std::string &nodeName);

    private:
        rclcpp::Subscription<UInt16MultiArray>::SharedPtr plc_feedback_;
        rclcpp::Publisher<UInt16MultiArray>::SharedPtr plc_command_;
    };
} // namespace BeienPaint

#endif //  PLC_MANAGER_NODE_HPP