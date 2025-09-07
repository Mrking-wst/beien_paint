#include <string>
#include <chrono>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "pkg_beien_paint_msgs/msg/joycon_left.hpp"
#include "joycon_left_node.hpp"
#include "hidapi/hidapi.h"

namespace JoyStick
{
    using pkg_beien_paint_msgs::msg::JoyconLeft;

    JoyconLeftNode::JoyconLeftNode(const std::string &nodeName)
        : rclcpp::Node(nodeName), is_connected_(false)
    {
        this->declare_parameter<int>("poll_interval_ms", 200);
        this->declare_parameter<int>("reconnect_interval_ms", 2000);

        this->get_parameter<int>("poll_interval_ms", this->poll_interval_ms_);
        this->get_parameter<int>("reconnect_interval_ms", this->reconnect_interval_ms_);

        this->left_joycon_ = this->create_publisher<JoyconLeft>("joycon_left", 10);

        this->reconnect_timer_ = this->create_wall_timer(std::chrono::milliseconds(this->reconnect_interval_ms_),
                                                         std::bind(&JoyconLeftNode::ReconnectTimerCallback, this));

        this->poll_timer_ = this->create_wall_timer(std::chrono::milliseconds(this->poll_interval_ms_),
                                                    std::bind(&JoyconLeftNode::PollTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "[%s] is initialized", this->get_name());
        this->poll_timer_->cancel();
        int res = hid_init();
        if (res == -1)
            RCLCPP_INFO(this->get_logger(), "初始化hidapi库失败，请确保系统中成功安装：libhidapi-dev库");
        else
            RCLCPP_INFO(this->get_logger(), "初始化hidapi库成功");
    }

    void
    JoyconLeftNode::PollTimerCallback()
    {
        int res = hid_read_timeout(this->joycon_, this->data_, sizeof(this->data_), 500);

        if (res <= 0)
        {
            RCLCPP_INFO(this->get_logger(), "收到数据长度： %d，手柄可能断开连接，详细信息：%ls",
                        res,
                        hid_error(this->joycon_));
            RCLCPP_INFO(this->get_logger(), "正在重新连接...");
            this->poll_timer_->cancel();
            this->reconnect_timer_->reset();
        }
        else
        {
            JoyconLeft left = this->ParseData(this->data_);
            std::ostringstream oss;

            for (size_t i = 0; i < left.raw_data.size(); i++)
            {
                oss << static_cast<int>(left.raw_data[i]);
                if (i != left.raw_data.size() - 1)
                    oss << " ";
            }

            RCLCPP_INFO(this->get_logger(), "原始数据：%s", oss.str().c_str());
            this->left_joycon_->publish(left);
        }
    }

    void
    JoyconLeftNode::ReconnectTimerCallback()
    {
        this->joycon_ = hid_open(0x057E, 0x2006, nullptr);

        if (this->joycon_ == nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "joycon左手柄，连接失败");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "joycon左手柄，连接成功");
            this->reconnect_timer_->cancel();
            this->poll_timer_->reset();
        }
    }

    JoyconLeft
    JoyconLeftNode::ParseData(unsigned char *data)
    {
        JoyconLeft joycon;
        joycon.up = false;
        joycon.down = false;
        joycon.left = false;
        joycon.right = false;
        joycon.l = false;
        joycon.zl = false;
        joycon.minus = false;
        joycon.stick_x = 128;
        joycon.stick_y = 128;

        if (data != nullptr)
        {
            joycon.zl = (data[1] & 0x40) != 0x00;
            joycon.l = (data[1] & 0x10) != 0x00;
            joycon.minus = (data[2] & 0x01) != 0x00;

            switch (data[3])
            {
            case 0x00:
                joycon.up = true;
                break;
            case 0x01:
                joycon.up = true;
                joycon.right = true;
                break;
            case 0x02:
                joycon.right = true;
                break;
            case 0x03:
                joycon.right = true;
                joycon.down = true;
                break;
            case 0x04:
                joycon.down = true;
                break;
            case 0x05:
                joycon.down = true;
                joycon.left = true;
                break;
            case 0x06:
                joycon.left = true;
                break;
            case 0x07:
                joycon.left = true;
                joycon.up = true;
                break;
            case 0x08:
                break;
            default:
                break;
            }
            joycon.stick_x = data[5];
            joycon.stick_y = data[7];
            for (int i = 0; i < 12; i++)
            {
                joycon.raw_data[i] = data[i];
            }
        }

        return joycon;
    }

    void
    JoyconLeftNode::Dispose()
    {
        RCLCPP_INFO(this->get_logger(), "释放节点资源...");
        if (!this->poll_timer_->is_canceled())
            this->poll_timer_->cancel();

        if (!this->reconnect_timer_->is_canceled())
            this->reconnect_timer_->cancel();

        hid_close(this->joycon_);
        hid_exit();
    }
} // namespace JoyStick
