#include <string>
#include <chrono>
#include <sstream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "pkg_beien_paint_msgs/msg/joycon_right.hpp"
#include "joycon_right_node.hpp"
#include "hidapi/hidapi.h"

namespace JoyStick
{
    using pkg_beien_paint_msgs::msg::JoyconRight;

    JoyconRightNode::JoyconRightNode(const std::string &nodeName)
        : rclcpp::Node(nodeName), is_connected_(false)
    {
        this->declare_parameter<int>("poll_interval_ms", 200);
        this->declare_parameter<int>("reconnect_interval_ms", 2000);

        this->get_parameter<int>("poll_interval_ms", this->poll_interval_ms_);
        this->get_parameter<int>("reconnect_interval_ms", this->reconnect_interval_ms_);

        this->left_joycon_ = this->create_publisher<JoyconRight>("joycon_right", 10);

        RCLCPP_INFO(this->get_logger(), "[%s] is initialized", this->get_name());
        int res = hid_init();
        if (res == -1)
            RCLCPP_INFO(this->get_logger(), "初始化hidapi库失败，请确保系统中成功安装：libhidapi-dev库");
        else
            RCLCPP_INFO(this->get_logger(), "初始化hidapi库成功");

        this->reconnect_thread_ = std::thread(std::bind(&JoyconRightNode::Reconnect, this));
        RCLCPP_INFO(this->get_logger(), "启动连接并读取任务");
    }

    void
    JoyconRightNode::PollData()
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
            JoyconRight left = this->ParseData2204(this->data_);
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
    JoyconRightNode::Reconnect()
    {
        while (rclcpp::ok())
        {
            this->joycon_ = hid_open(0x057E, 0x2007, nullptr);

            if (this->joycon_ == nullptr)
            {
                this->is_connected_ = false;
                RCLCPP_INFO(this->get_logger(), "joycon右手柄，连接失败");
                std::this_thread::sleep_for(std::chrono::milliseconds(this->reconnect_interval_ms_));
                continue;
            }
            RCLCPP_INFO(this->get_logger(), "joycon右手柄，连接成功");
            this->is_connected_ = true;
            while (rclcpp::ok() && this->is_connected_)
            {
                this->PollData();
                // std::this_thread::sleep_for(std::chrono::milliseconds(this->poll_interval_ms_));
            }
        }
        RCLCPP_INFO(this->get_logger(), "用户强制退出右手柄交互");
    }

    JoyconRight
    JoyconRightNode::ParseData2404(unsigned char *data)
    {
        JoyconRight joycon;
        joycon.a = false;
        joycon.b = false;
        joycon.x = false;
        joycon.y = false;
        joycon.r = false;
        joycon.zr = false;
        joycon.plus = false;
        joycon.sl = false;
        joycon.sr = false;
        joycon.stick_x = 2048;
        joycon.stick_y = 2047;

        if (data != nullptr)
        {
            joycon.a = (data[1] & 0x01) != 0x00;
            joycon.b = (data[1] & 0x02) != 0x00;
            joycon.x = (data[1] & 0x04) != 0x00;
            joycon.y = (data[1] & 0x08) != 0x00;
            joycon.r = (data[1] & 0x20) != 0x00;
            joycon.zr = (data[1] & 0x80) != 0x00;
            joycon.plus = (data[2] & 0x02) != 0x00;

            
            joycon.stick_x = data[9];
            joycon.stick_y = data[11];
            for (int i = 0; i < 12; i++)
            {
                joycon.raw_data[i] = data[i];
            }
        }

        return joycon;
    }

    JoyconRight
    JoyconRightNode::ParseData2204(unsigned char *data)
    {
        JoyconRight joycon;
        joycon.y = false;
        joycon.x = false;
        joycon.b = false;
        joycon.a = false;
        joycon.sr = false;
        joycon.sl = false;
        joycon.r = false;
        joycon.zr = false;
        joycon.plus = false;
        joycon.home = false;
        joycon.stick_x = 2048;
        joycon.stick_y = 2047;

        if (data != nullptr)
        {
            joycon.y = (data[3] & 0x01) != 0x00;
            joycon.x = (data[3] & 0x02) != 0x00;
            joycon.b = (data[3] & 0x04) != 0x00;
            joycon.a = (data[3] & 0x08) != 0x00;
            joycon.sr = (data[3] & 0x10) != 0x00;
            joycon.sl = (data[3] & 0x20) != 0x00;
            joycon.r = (data[3] & 0x40) != 0x00;
            joycon.zr = (data[3] & 0x80) != 0x00;
            joycon.plus = (data[4] & 0x02) != 0x00;
            joycon.home = (data[4] & 0x10) != 0x00;

            joycon.stick_x = data[9] | ((data[10] & 0x0F) << 8);
            joycon.stick_y = (data[10] >> 4) | (data[11] << 4);
            for (int i = 0; i < 12; i++)
            {
                joycon.raw_data[i] = data[i];
            }
        }

        return joycon;
    }

    void
    JoyconRightNode::Dispose()
    {
        RCLCPP_INFO(this->get_logger(), "释放节点资源...");
        if (this->reconnect_thread_.joinable())
            this->reconnect_thread_.join();
        hid_close(this->joycon_);
        hid_exit();
    }
} // namespace JoyStick
