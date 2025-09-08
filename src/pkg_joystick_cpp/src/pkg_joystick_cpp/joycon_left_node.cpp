#include <string>
#include <chrono>
#include <sstream>
#include <thread>

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

        RCLCPP_INFO(this->get_logger(), "[%s] is initialized", this->get_name());
        int res = hid_init();
        if (res == -1)
            RCLCPP_INFO(this->get_logger(), "初始化hidapi库失败，请确保系统中成功安装：libhidapi-dev库");
        else
            RCLCPP_INFO(this->get_logger(), "初始化hidapi库成功");

        this->reconnect_thread_ = std::thread(std::bind(&JoyconLeftNode::Reconnect, this));
        RCLCPP_INFO(this->get_logger(), "启动连接并读取任务");
    }

    void
    JoyconLeftNode::PollData()
    {
        int res = hid_read_timeout(this->joycon_, this->data_, sizeof(this->data_), 500);

        if (res < 0)
        {
            this->is_connected_ = false;
            RCLCPP_INFO(this->get_logger(), "收到数据长度： %d，手柄可能断开连接，详细信息：%ls",
                        res,
                        hid_error(this->joycon_));
            RCLCPP_INFO(this->get_logger(), "正在重新连接...");
        }
        else if (res == 0)
        {
            RCLCPP_INFO(this->get_logger(), "读取超时");
        }
        else
        {
            JoyconLeft left = this->ParseData2204(this->data_);
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
    JoyconLeftNode::Reconnect()
    {
        while (rclcpp::ok())
        {
            this->joycon_ = hid_open(0x057E, 0x2006, nullptr);

            if (this->joycon_ == nullptr)
            {
                this->is_connected_ = false;
                RCLCPP_INFO(this->get_logger(), "joycon左手柄，连接失败");
                std::this_thread::sleep_for(std::chrono::milliseconds(this->reconnect_interval_ms_));
                continue;
            }
            RCLCPP_INFO(this->get_logger(), "joycon左手柄，连接成功");
            this->is_connected_ = true;
            while (rclcpp::ok() && this->is_connected_)
            {
                this->PollData();
                // std::this_thread::sleep_for(std::chrono::milliseconds(this->poll_interval_ms_));
            }
        }
        RCLCPP_INFO(this->get_logger(), "用户强制退出左手柄交互");
    }

    JoyconLeft
    JoyconLeftNode::ParseData2404(unsigned char *data)
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

    JoyconLeft
    JoyconLeftNode::ParseData2204(unsigned char *data)
    {
        JoyconLeft joycon;
        joycon.up = false;
        joycon.down = false;
        joycon.left = false;
        joycon.right = false;
        joycon.l = false;
        joycon.zl = false;
        joycon.minus = false;
        joycon.function = false;
        joycon.sr = false;
        joycon.sl = false;
        joycon.stick_x = 2048;
        joycon.stick_y = 2047;

        if (data != nullptr)
        {
            joycon.minus = (data[4] & 0x01) != 0x00;
            joycon.function = (data[4] & 0x20) != 0x00;
            joycon.down = (data[5] & 0x01) != 0x00;
            joycon.up = (data[5] & 0x02) != 0x00;
            joycon.right = (data[5] & 0x04) != 0x00;
            joycon.left = (data[5] & 0x08) != 0x00;
            joycon.sr = (data[5] & 0x10) != 0x00;
            joycon.sl = (data[5] & 0x20) != 0x00;
            joycon.l = (data[5] & 0x40) != 0x00;
            joycon.zl = (data[5] & 0x80) != 0x00;

            joycon.stick_x = data[6] | ((data[7] & 0x0F) << 8);
            joycon.stick_y = (data[7] >> 4) | (data[8] << 4);
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
        if (this->reconnect_thread_.joinable())
            this->reconnect_thread_.join();
        hid_close(this->joycon_);
        hid_exit();
    }
} // namespace JoyStick
