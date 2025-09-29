#ifndef BEIEN_PAINT_NODE_HPP
#define BEIEN_PAINT_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "pkg_beien_paint_msgs/msg/joycon_left.hpp"
#include "pkg_beien_paint_msgs/msg/joycon_right.hpp"
#include "pkg_beien_paint_msgs/msg/plc_command.hpp"

namespace BeienPaint
{
    using pkg_beien_paint_msgs::msg::JoyconLeft;
    using pkg_beien_paint_msgs::msg::JoyconRight;
    using pkg_beien_paint_msgs::msg::PlcCommand;

    class BeienPaintNode
        : public rclcpp::Node
    {
    public:
        BeienPaintNode(const std::string &node_name);

    private:
        void JoyconLeftCallback(const JoyconLeft &joycon_left_msg);
        void JoyconRightCallback(const JoyconRight &joycon_right_msg);
        float Map(float x, float in_min, float in_max, float out_min, float out_max);
        float Constrain(float x, float out_min, float out_max);
        void PushCommand();

    private:
        rclcpp::Subscription<JoyconLeft>::SharedPtr joycon_left_sub_;
        rclcpp::Subscription<JoyconRight>::SharedPtr joycon_right_sub_;
        rclcpp::Publisher<PlcCommand>::SharedPtr plc_command_pub_;
        rclcpp::TimerBase::SharedPtr push_command_timer_;
        float speed_;
        float angle_;
    };
}

#endif // BEIEN_PAINT_NODE_HPP