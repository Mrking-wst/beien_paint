#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "pkg_beien_paint_msgs/msg/joycon_left.hpp"
#include "pkg_beien_paint_msgs/msg/joycon_right.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "plc_manager_node.hpp"

namespace BeienPaint
{
    using pkg_beien_paint_msgs::msg::JoyconLeft;
    using pkg_beien_paint_msgs::msg::JoyconRight;
    using std_msgs::msg::UInt16MultiArray;

    PlcManagerNode::PlcManagerNode(const std::string &nodeName)
        : Node(nodeName)
    {
        RCLCPP_INFO(this->get_logger(), "PlcManagerNode started.");

        this->plc_feedback_ = this->create_subscription<UInt16MultiArray>(
            "/plc_feedback", 10,
            std::bind(&PlcManagerNode::plcFeedbackCallback, this, std::placeholders::_1));
    }

    void PlcManagerNode::plcFeedbackCallback(const UInt16MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received PLC feedback with data size: %zu", msg->data.size());
        //  解析PLC反馈数据，并发布给各个功能模块
    }

    void PlcManagerNode::joyconLeftCallback(const JoyconLeft::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Joycon Left data.");
        //  处理左手手柄数据
    }

    void PlcManagerNode::joyconRightCallback(const JoyconRight::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Joycon Right data.");
        //  处理右手手柄数据
    }
} // namespace BeienPaint