#ifndef JOYCON_LEFT_NODE_HPP
#define JOYCON_LEFT_NODE_HPP

#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "pkg_beien_paint_msgs/msg/joycon_right.hpp"
#include "hidapi/hidapi.h"

namespace JoyStick
{
    using pkg_beien_paint_msgs::msg::JoyconRight;
    class JoyconRightNode
        : public rclcpp::Node
    {
    public:
        JoyconRightNode(const std::string &nodeName);
        void Dispose();

    private:
        void PollData();
        void Reconnect();
        JoyconRight ParseData2404(unsigned char *data);
        JoyconRight ParseData2204(unsigned char *data);

    private:
        hid_device *joycon_;
        unsigned char data_[49];
        rclcpp::Publisher<JoyconRight>::SharedPtr left_joycon_;
        std::thread reconnect_thread_;
        bool is_connected_;
        int poll_interval_ms_;
        int reconnect_interval_ms_;
    };

} // namespace JoyStick

#endif //  JOYCON_RIGHT_NODE_HPP