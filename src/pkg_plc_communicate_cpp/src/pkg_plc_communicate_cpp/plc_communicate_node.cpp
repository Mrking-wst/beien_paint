#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include <sstream>

#include "plc_communicate_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "modbus/modbus.h"

namespace PlcCommunicate
{
    using std_msgs::msg::UInt16MultiArray;

    PlcCommunicateNode::PlcCommunicateNode(const std::string &nodeName)
        : Node(nodeName), is_connected_(false)
    {
        this->declare_parameter<std::string>("plc_ip", "192.168.0.10");
        this->declare_parameter<int>("plc_port", 502);
        this->declare_parameter<int>("read_interval", 100);
        this->declare_parameter<int>("reconnect_interval", 2000);
        this->declare_parameter<int>("read_holding_register_start_address", 0);
        this->declare_parameter<int>("read_holding_register_num", 10);
        this->declare_parameter<int>("write_holding_register_start_address", 0);
        this->declare_parameter<int>("write_holding_register_num", 20);
        this->declare_parameter<int>("response_timeout", 1000);

        this->get_parameter<std::string>("plc_ip", plc_ip_);
        this->get_parameter<int>("plc_port", plc_port_);
        this->get_parameter<int>("read_interval", read_interval_);
        this->get_parameter<int>("reconnect_interval", reconnect_interval_);
        this->get_parameter<int>("read_holding_register_start_address", read_holding_register_start_address_);
        this->get_parameter<int>("read_holding_register_num", read_holding_register_num_);
        this->get_parameter<int>("write_holding_register_start_address", write_holding_register_start_address_);
        this->get_parameter<int>("write_holding_register_num", write_holding_register_num_);
        this->get_parameter<int>("response_timeout", response_timeout_);

        RCLCPP_INFO(this->get_logger(), "%s 节点初始化", this->get_name());

        this->source_data_pub_ = this->create_publisher<UInt16MultiArray>("source_data_pub", 10);
        this->source_data_sub_ = this->create_subscription<UInt16MultiArray>("source_data_sub",
                                                                             10,
                                                                             std::bind(&PlcCommunicateNode::SourceDataReceivedCallback, this, std::placeholders::_1));
        this->reconnect_time_ = this->create_wall_timer(std::chrono::milliseconds(this->reconnect_interval_),
                                                        std::bind(&PlcCommunicateNode::TryReconnectModbus, this));

        this->poll_time_ = this->create_wall_timer(std::chrono::milliseconds(this->read_interval_),
                                                   std::bind(&PlcCommunicate::PlcCommunicateNode::PollModbus, this));

        this->reconnect_time_->cancel();
    }

    void
    PlcCommunicateNode::Dispose()
    {
        if (this->ctx_ == nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "[%s] modbus 已经被释放", this->get_name());
            return;
        }

        this->is_connected_ = false;
        this->reconnect_time_->cancel();
        this->poll_time_->cancel();

        modbus_close(this->ctx_);
        modbus_free(this->ctx_);
        this->ctx_ = nullptr;
    }

    void
    PlcCommunicateNode::ConnectModbus()
    {
        if (this->ctx_ != nullptr)
        {
            modbus_close(this->ctx_);
            modbus_free(this->ctx_);
            this->ctx_ = nullptr;
            RCLCPP_INFO(this->get_logger(), "[%s] 当前连接不为空，连接之前需要将之前连接清空释放.", this->get_name());
        }

        this->ctx_ = modbus_new_tcp(this->plc_ip_.c_str(), this->plc_port_);
        if (this->ctx_ == nullptr)
        {
            RCLCPP_INFO(this->get_logger(),
                        " [%s] 无法使用 %s:%i", this->get_name(), this->plc_ip_.c_str(), this->plc_port_);
            return;
        }

        modbus_set_response_timeout(this->ctx_, this->response_timeout_, 0);

        if (modbus_connect(this->ctx_) == -1)
        {
            int err = errno;
            RCLCPP_INFO(this->get_logger(),
                        "[%s] %s:%i，连接失败-%s",
                        this->get_name(),
                        this->plc_ip_.c_str(),
                        this->plc_port_,
                        modbus_strerror(err));
            modbus_free(this->ctx_);
            this->ctx_ = nullptr;
            return;
        }

        this->is_connected_ = true;
        RCLCPP_INFO(this->get_logger(), "[%s] 成功连接modbus服务端：%s:%d", this->get_name(), this->plc_ip_.c_str(), this->plc_port_);
        this->reconnect_time_->cancel();
    }

    void
    PlcCommunicateNode::PollModbus()
    {
        // std::vector<uint16_t> sourceReadingData(this->read_holding_register_num_);
        uint16_t sourceReadingData[this->read_holding_register_num_];
        int count = modbus_read_registers(this->ctx_,
                                          this->read_holding_register_start_address_,
                                          this->read_holding_register_num_,
                                          sourceReadingData);
        if (count == -1)
        {
            int err = errno;
            RCLCPP_INFO(get_logger(), "[%s] 参数:%d %d", this->get_name(), this->read_holding_register_start_address_, this->read_holding_register_num_);
            RCLCPP_INFO(get_logger(), "[%s] 读取数据失败:%s", this->get_name(), modbus_strerror(err));
            this->is_connected_ = false;
            this->ConnectModbus(); //  重新连接一次

            if (!this->is_connected_)
            {
                this->reconnect_time_->reset(); //  启动周期性冲连
            }
            return;
        }

        UInt16MultiArray msg;
        std::ostringstream datas;
        for (int i = 0; i < this->read_holding_register_num_; i++)
        {
            msg.data.push_back(sourceReadingData[i]);
            datas << sourceReadingData[i];
            if (i != read_holding_register_num_ - 1)
            {
                datas << ' ';
            }
        }

        this->source_data_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
                    "[%s] 发布数据：%s",
                    this->get_name(),
                    datas.str().c_str());
    }

    void
    PlcCommunicateNode::TryReconnectModbus()
    {
        if (!this->is_connected_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "[%s] 正在重新连接：%s:%d...",
                        this->get_name(),
                        this->plc_ip_.c_str(),
                        this->plc_port_);
            this->ConnectModbus();
        }
    }

    void
    PlcCommunicateNode::SourceDataReceivedCallback(const UInt16MultiArray &source_data)
    {
        if (source_data.data.empty())
        {
            RCLCPP_INFO(this->get_logger(), "[%s] 待写入的数据为空！", this->get_name());
            return;
        }

        if (this->is_connected_)
        {
            RCLCPP_INFO(this->get_logger(), "[%s] 远程服务端未连接，数据写入失败！", this->get_name());
            return;
        }

        if (this->ctx_ != nullptr)
        {
            modbus_write_registers(this->ctx_,
                                   this->write_holding_register_start_address_,
                                   this->write_holding_register_num_,
                                   source_data.data.data());
        }
    }
} // namespace PlcCommunicate
