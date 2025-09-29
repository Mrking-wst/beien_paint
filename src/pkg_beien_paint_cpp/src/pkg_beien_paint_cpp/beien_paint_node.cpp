#include <string>

#include "beien_paint_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace BeienPaint
{
    BeienPaintNode::BeienPaintNode(const std::string &node_name)
        : Node(node_name)
    {
        joycon_left_sub_ = this->create_subscription<JoyconLeft>(
            "joycon_left",
            10,
            std::bind(&BeienPaintNode::JoyconLeftCallback, this, std::placeholders::_1));

        joycon_right_sub_ = this->create_subscription<JoyconRight>(
            "joycon_right",
            10,
            std::bind(&BeienPaintNode::JoyconRightCallback, this, std::placeholders::_1));

        plc_command_pub_ = this->create_publisher<PlcCommand>("/plc/plc_command", 10);
    }

    void 
    BeienPaintNode::JoyconLeftCallback(const JoyconLeft &joycon_left_msg)
    {
        auto speed = Map(joycon_left_msg.stick_y, 0, 4096, -25.0, 25.0);
        PlcCommand plc_cmd;
        plc_cmd.source_id = "BeienPaintNode";
        plc_cmd.start_address = 4; // 假设起始地址为1
        plc_cmd.register_values = {static_cast<uint16_t>(speed)};
        plc_cmd.priority = 1;            // 优先级
        plc_cmd.op_type = 0;             // 假设1代表写操作
        plc_cmd.write_mode = 1;          // 假设1代表覆盖模式
        plc_cmd.expire_duration.sec = 1; // 1秒后过期
        plc_cmd.expire_duration.nanosec = 0;
        plc_cmd.header.stamp = this->now();
        RCLCPP_INFO(this->get_logger(),"写入速度");
        this->plc_command_pub_->publish(plc_cmd);
    }

    void 
    BeienPaintNode::JoyconRightCallback(const JoyconRight &joycon_right_msg)
    {
        auto angle = Map(joycon_right_msg.stick_x, 0, 4096, -25.0, 25.0);
        PlcCommand plc_cmd;
        plc_cmd.source_id = "BeienPaintNode";
        plc_cmd.start_address = 3;         // 假设起始地址为1
        plc_cmd.register_values = {static_cast<uint16_t>(angle)};
        plc_cmd.priority = 1;            // 优先级
        plc_cmd.op_type = 0;             // 假设1代表写操作
        plc_cmd.write_mode = 1;          // 假设1代表覆盖模式
        plc_cmd.expire_duration.sec = 1; // 1秒后过期
        plc_cmd.expire_duration.nanosec = 0;
        plc_cmd.header.stamp = this->now();
        this->plc_command_pub_->publish(plc_cmd);
    }

    float
    BeienPaintNode::Map(float x, float in_min, float in_max, float out_min, float out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    float
    BeienPaintNode::Constrain(float x, float out_min, float out_max)
    {
        if (x < out_min)
            return out_min;
        if (x > out_max)
            return out_max;
        return x;
    }
} // namespace BeienPaint