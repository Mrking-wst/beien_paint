#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "plc_communicate_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "modbus/modbus.h"
#include "yaml-cpp/yaml.h"
#include "pkg_beien_paint_msgs/msg/car_info.hpp"
#include "pkg_beien_paint_msgs/msg/joycon_left.hpp"
#include "pkg_beien_paint_msgs/msg/joycon_right.hpp"

namespace PlcCommunicate
{
    using pkg_beien_paint_msgs::msg::CarInfo;
    using std_msgs::msg::UInt16MultiArray;

    PlcCommunicateNode::PlcCommunicateNode(const std::string &nodeName)
        : Node(nodeName), is_connected_(false), plc_scheduler_(nullptr)
    {
        // this->declare_parameter<std::string>("plc_ip", "192.168.31.106");
        this->declare_parameter<std::string>("plc_ip", "192.168.100.5");
        this->declare_parameter<int>("plc_port", 502);
        this->declare_parameter<int>("read_interval", 100);
        this->declare_parameter<int>("reconnect_interval", 5000);
        this->declare_parameter<int>("read_holding_register_start_address", 0);
        this->declare_parameter<int>("read_holding_register_num", 10);
        this->declare_parameter<int>("write_holding_register_start_address", 0);
        this->declare_parameter<int>("write_holding_register_num", 20);
        this->declare_parameter<int>("response_timeout", 1);

        this->get_parameter<std::string>("plc_ip", plc_ip_);
        this->get_parameter<int>("plc_port", plc_port_);
        this->get_parameter<int>("read_interval", read_interval_);
        this->get_parameter<int>("reconnect_interval", reconnect_interval_);
        this->get_parameter<int>("read_holding_register_start_address", read_holding_register_start_address_);
        this->get_parameter<int>("read_holding_register_num", read_holding_register_num_);
        this->get_parameter<int>("write_holding_register_start_address", write_holding_register_start_address_);
        this->get_parameter<int>("write_holding_register_num", write_holding_register_num_);
        this->get_parameter<int>("response_timeout", response_timeout_);
        sourceReadingData_.resize(read_holding_register_num_);
        RCLCPP_INFO(this->get_logger(), "节点初始化");

        this->source_data_pub_ = this->create_publisher<UInt16MultiArray>("plc_feedback", 10);
        this->source_data_sub_ = this->create_subscription<UInt16MultiArray>("plc_command1",
                                                                             10,
                                                                             std::bind(&PlcCommunicateNode::SourceDataReceivedCallback, this, std::placeholders::_1));
        this->plc_command_sub_ = this->create_subscription<PlcCommand>("plc_command",
                                                                       10,
                                                                       std::bind(&PlcCommunicateNode::PlcCommandReceivedCallback, this, std::placeholders::_1));
        this->reconnect_time_ = this->create_wall_timer(std::chrono::milliseconds(this->reconnect_interval_),
                                                        std::bind(&PlcCommunicateNode::TryReconnectModbus, this));

        this->poll_time_ = this->create_wall_timer(std::chrono::milliseconds(this->read_interval_),
                                                   std::bind(&PlcCommunicateNode::PollModbus, this));
        this->scheduler_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                                         std::bind(&PlcCommunicateNode::PushCommand, this));
        this->car_info_pub_ = this->create_publisher<CarInfo>("car_info", 10);
        this->reconnect_time_->cancel();
        this->InitializeRegistersMapping();
    }

    void
    PlcCommunicateNode::Dispose()
    {
        if (this->ctx_ == nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "modbus 已经被释放");
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
            RCLCPP_INFO(this->get_logger(), "当前连接不为空，连接之前需要将之前连接清空释放.");
        }

        this->ctx_ = modbus_new_tcp(this->plc_ip_.c_str(), this->plc_port_);
        if (this->ctx_ == nullptr)
        {
            RCLCPP_INFO(this->get_logger(),
                        "无法使用 %s:%i", this->plc_ip_.c_str(), this->plc_port_);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "tcp 实例创建成功：%s:%d", this->plc_ip_.c_str(), this->plc_port_);
        modbus_set_response_timeout(this->ctx_, this->response_timeout_, 0);

        if (modbus_connect(this->ctx_) == -1)
        {
            int err = errno;
            RCLCPP_INFO(this->get_logger(),
                        "%s:%i，连接失败-%s",
                        this->plc_ip_.c_str(),
                        this->plc_port_,
                        modbus_strerror(err));
            modbus_free(this->ctx_);
            this->ctx_ = nullptr;
            this->is_connected_ = false;
            return;
        }

        this->is_connected_ = true;
        RCLCPP_INFO(this->get_logger(), "成功连接modbus服务端：%s:%d", this->plc_ip_.c_str(), this->plc_port_);
        this->reconnect_time_->cancel();
        this->poll_time_->reset();
    }

    void
    PlcCommunicateNode::PollModbus()
    {
        // std::vector<uint16_t> sourceReadingData(this->read_holding_register_num_);
        // uint16_t sourceReadingData[this->read_holding_register_num_];
        int count = modbus_read_registers(this->ctx_,
                                          this->read_holding_register_start_address_,
                                          this->read_holding_register_num_,
                                          sourceReadingData_.data());
        if (count == -1)
        {
            int err = errno;
            RCLCPP_INFO(get_logger(), "参数:%d %d", this->read_holding_register_start_address_, this->read_holding_register_num_);
            RCLCPP_INFO(get_logger(), "读取数据失败:%s", modbus_strerror(err));
            this->is_connected_ = false;
            this->ConnectModbus(); //  重新连接一次

            if (!this->is_connected_)
            {
                this->poll_time_->cancel();
                this->reconnect_time_->reset(); //  启动周期性冲连
                RCLCPP_INFO(this->get_logger(), "进入断线重连...");
            }
            return;
        }

        UInt16MultiArray msg;
        std::ostringstream datas;
        for (int i = 0; i < this->read_holding_register_num_; i++)
        {
            msg.data.push_back(sourceReadingData_[i]);
            datas << sourceReadingData_[i];
            if (i != read_holding_register_num_ - 1)
            {
                datas << ' ';
            }
        }

        this->source_data_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
                    "发布数据：%s",
                    datas.str().c_str());
    }

    void
    PlcCommunicateNode::TryReconnectModbus()
    {
        if (!this->is_connected_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "正在重新连接：%s:%d...",
                        this->plc_ip_.c_str(),
                        this->plc_port_);
            this->ConnectModbus();
        }
    }

    void
    PlcCommunicateNode::PushCommand()
    {
        plc_scheduler_.Dispatch();
    }

    void
    PlcCommunicateNode::SourceDataReceivedCallback(const UInt16MultiArray &source_data)
    {
        if (source_data.data.empty())
        {
            RCLCPP_INFO(this->get_logger(), "待写入的数据为空！");
            return;
        }

        if (this->is_connected_)
        {
            RCLCPP_INFO(this->get_logger(), "远程服务端未连接，数据写入失败！");
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

    void
    PlcCommunicateNode::PlcCommandReceivedCallback(const PlcCommand &plc_command)
    {
        if (plc_command.register_values.empty())
        {
            RCLCPP_INFO(this->get_logger(), "待写入的数据为空！");
            return;
        }

        if (plc_command.register_values.size() > static_cast<size_t>(this->write_holding_register_num_))
        {
            RCLCPP_INFO(this->get_logger(), "待写入的数据长度超过配置的最大写入长度！");
            return;
        }

        if (!this->is_connected_)
        {
            RCLCPP_INFO(this->get_logger(), "远程服务端未连接，数据写入失败！");
            return;
        }

        this->plc_scheduler_ = PlcScheduler(this->ctx_);
        PlcCommandEntry plc_command_entry;
        plc_command_entry.start_address = plc_command.start_address;
        plc_command_entry.register_values = plc_command.register_values;
        plc_command_entry.priority = static_cast<PlcPriority>(plc_command.priority);
        plc_command_entry.op_type = static_cast<PlcOpType>(plc_command.op_type);
        plc_command_entry.write_mode = static_cast<PlcWriteMode>(plc_command.write_mode);
        plc_command_entry.source_id = plc_command.source_id;
        auto ms = std::chrono::milliseconds(static_cast<uint64_t>(plc_command.header.stamp.sec) * 1000 + static_cast<uint64_t>(plc_command.header.stamp.nanosec) / 1000000);
        plc_command_entry.timestamp = std::chrono::steady_clock::time_point(ms);
        plc_command_entry.expire_duration = std::chrono::milliseconds(
            static_cast<int64_t>(plc_command.expire_duration.sec) * 1000 +
            static_cast<int64_t>(plc_command.expire_duration.nanosec) / 1000000);
        // RCLCPP_INFO(this->get_logger(), "向调度器写入指令");
        this->plc_scheduler_.PushCommand(plc_command_entry);
    }

    void
    PlcCommunicateNode::InitializeRegistersMapping()
    {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("pkg_plc_communicate_cpp");
        std::string yaml_file = pkg_path + "/config/plc_mapping.yaml";
        //  读取寄存器映射关系
        YAML::Node config = YAML::LoadFile(yaml_file);
        if (config.IsNull())
        {
            RCLCPP_ERROR(this->get_logger(), "读取寄存器映射关系失败，请检查文件是否存在！");
            return;
        }
        if (!config["plc_mapping"].IsDefined())
        {
            RCLCPP_ERROR(this->get_logger(), "配置文件中没有找到 plc_mapping 节点，请检查文件内容！");
            return;
        }
        auto mapping = config["plc_mapping"];
        for (auto it = mapping.begin(); it != mapping.end(); ++it)
        {
            std::string key = it->first.as<std::string>();
            int value = it->second.as<int>();
            this->registers_mapping_.emplace(key, value);
        }
    }

    float
    PlcCommunicateNode::Map(float x, float in_min, float in_max, float out_min, float out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    float
    PlcCommunicateNode::Constrain(float x, float out_min, float out_max)
    {
        if (x < out_min)
            return out_min;
        if (x > out_max)
            return out_max;
        return x;
    }

    uint16_t
    PlcCommunicateNode::GetUint16Fromfloat(float value)
    {

        return static_cast<uint16_t>(value);
    }
} // namespace PlcCommunicate
