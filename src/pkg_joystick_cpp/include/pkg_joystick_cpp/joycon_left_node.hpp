#ifndef JOYCON_LEFT_NODE_HPP
#define JOYCON_LEFT_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "pkg_beien_paint_msgs/msg/joycon_left.hpp"
namespace JoyStick
{
    using pkg_beien_paint_msgs::msg::JoyconLeft;
    class JoyconLeftNode
        : public rclcpp::Node
    {

    private:
        rclcpp::Publisher<JoyconLeft>::SharedPtr left_joycon_;
        rclcpp::TimerBase::SharedPtr poll_timer_;
    };

} // namespace JoyStick

#endif //  JOYCON_LEFT_NODE_HPP