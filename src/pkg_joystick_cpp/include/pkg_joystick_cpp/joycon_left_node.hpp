#ifndef JOYCON_LEFT_NODE_HPP
#define JOYCON_LEFT_NODE_HPP

#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "pkg_beien_paint_msgs/msg/joycon_left.hpp"
#include "hidapi/hidapi.h"

namespace JoyStick
{
    using pkg_beien_paint_msgs::msg::JoyconLeft;
    class JoyconLeftNode
        : public rclcpp::Node
    {
    public:
        JoyconLeftNode(const std::string &nodeName);
        void Dispose();

    private:
        void PollData();
        void Reconnect();
        JoyconLeft ParseData2404(unsigned char *data);
        JoyconLeft ParseData2204(unsigned char *data);

    private:
        hid_device *joycon_;
        unsigned char data_[49];
        rclcpp::Publisher<JoyconLeft>::SharedPtr left_joycon_;
        std::thread reconnect_thread_;
        bool is_connected_;
        int poll_interval_ms_;
        int reconnect_interval_ms_;
    };

} // namespace JoyStick

#endif //  JOYCON_LEFT_NODE_HPP