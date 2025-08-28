#include <chrono>
#include <string>

#include "plc_communicate_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "modbus/modbus.h"

namespace PlcCommunicate
{
    using std_msgs::msg::UInt16MultiArray;

    PlcCommunicateNode::PlcCommunicateNode(const std::string &nodeName)
        : rclcpp::Node(nodeName)
    {
        this->declare_parameter<std::string>("plc_ip", "192.168.0.10");
        this->declare_parameter<int>("plc_port", 502);
        this->declare_parameter<int>("read_interval", 100);
        this->declare_parameter<int>("reconnect_interval", 2000);
        this->declare_parameter<int>("holding_register_num", 20);
        this->declare_parameter<int>("response_timeout", 1000);

        this->get_parameter<std::string>("plc_ip", plc_ip_);
        this->get_parameter<int>("plc_port", plc_port_);
        this->get_parameter<int>("read_interval", read_interval_);
        this->get_parameter<int>("reconnect_interval", reconnect_interval_);
        this->get_parameter<int>("holding_register_num", holding_register_num_);
        this->get_parameter<int>("response_timeout", response_timeout_);

        RCLCPP_INFO(this->get_logger(), "%s 节点初始化", this->get_name());

        this->source_data_pub_ = this->create_publisher<UInt16MultiArray>("source_data_pub", 10);
        this->source_data_sub_ = this->create_subscription<UInt16MultiArray>("source_data_sub",
                                                                             10,
                                                                             std::bind(&PlcCommunicateNode::SourceDataReceivedCallback, this, std::placeholders::_1));
        this->start_time_ = this->create_wall_timer(std::chrono::milliseconds(this->reconnect_interval_),
                                                    std::bind(&PlcCommunicateNode::ReconnectLoop, this));
    }

    void
    PlcCommunicateNode::ReconnectLoop()
    {
        this->ctx_ = modbus_new_tcp(this->plc_ip_.c_str(), this->plc_port_);
        if (this->ctx_ == nullptr)
        {
            RCLCPP_INFO(this->get_logger(),
                        " [%s] 无法使用 %s:%i", this->get_name(), this->plc_ip_, this->plc_port_);
            return;
        }
        modbus_set_response_timeout(this->ctx_, this->response_timeout_, 0);

        if (modbus_connect(this->ctx_) == -1)
        {
            int err = errno;
            RCLCPP_INFO(this->get_logger(),
                        " [%s] %s:%i，连接失败-%s",
                        this->get_name(),
                        this->plc_ip_,
                        this->plc_port_,
                        modbus_strerror);
            modbus_free(this->ctx_);
            this->ctx_ = nullptr;
            return;
        }

        
    }

    void
    PlcCommunicateNode::Dispose()
    {
        if (this->ctx_ == nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "modbus 已经被释放");
            return;
        }

        modbus_close(this->ctx_);
        modbus_free(this->ctx_);
        this->ctx_ = nullptr;
    }

    void
    PlcCommunicateNode::SourceDataReceivedCallback(const UInt16MultiArray &source_data)
    {
    }
} // namespace PlcCommunicate
