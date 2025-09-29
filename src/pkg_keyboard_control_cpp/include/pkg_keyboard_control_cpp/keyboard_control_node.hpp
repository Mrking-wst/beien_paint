#ifndef KEYBOARD_CONTROL_NODE_HPP
#define KEYBOARD_CONTROL_NODE_HPP

#include <ncurses.h>

#include "rclcpp/rclcpp.hpp"
#include "pkg_beien_paint_msgs/msg/plc_command.hpp"

namespace KeyboardControl
{
    using pkg_beien_paint_msgs::msg::PlcCommand;

    class KeyboardControlNode
        : public rclcpp::Node
    {
    public:
        KeyboardControlNode(const std::string &nodeName);
        ~KeyboardControlNode();

    private:
        void TimerCallback();

    private:
        rclcpp::Publisher<PlcCommand>::SharedPtr plc_command_pub_;
        rclcpp::TimerBase::SharedPtr poll_time_;
        int base_speed_;
        int base_angle_;
        int speed_;
        bool is_exit_;
        const int max_speed_ = 40;
        const int min_speed_ = 0;
        const int max_angle_ = 40;
        const int min_angle_ = -40;

    }; // namespace KeyboardControl
}
#endif // KEYBOARD_CONTROL_NODE_HPP