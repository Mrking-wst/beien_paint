#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "pkg_beien_paint_msgs/msg/joycon_left.hpp"
#include "joycon_left_node.hpp"

namespace JoyStick
{
    using pkg_beien_paint_msgs::msg::JoyconLeft;

    JoyconLeftNode::JoyconLeftNode(const std::string &nodeName)
        : rclcpp::Node(nodeName), is_connected_(false)
    {
        this->left_joycon_ = this->create_publisher<JoyconLeft>("joycon_left", 10);

        this->poll_timer_ = this->create_wall_timer(std::chrono::milliseconds(this->poll_interval_ms_),
                                                    std::bind(&JoyconLeftNode::PollTimerCallback, this));
    }

    void
    JoyconLeftNode::PollTimerCallback()
    {
    }

    void
    JoyconLeftNode::ReconnectTimerCallback()
    {
    }
} // namespace JoyStick
