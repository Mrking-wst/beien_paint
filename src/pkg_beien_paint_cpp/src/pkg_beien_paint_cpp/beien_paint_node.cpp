#include <string>

#include "beien_paint_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace BeienPaint
{
    BeienPaintNode::BeienPaintNode(const std::string &node_name)
        : Node(node_name), speed_(0.0), angle_(0.0)
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
        push_command_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                                      std::bind(&BeienPaintNode::PushCommand, this));
    }

    void
    BeienPaintNode::JoyconLeftCallback(const JoyconLeft &joycon_left_msg)
    {
        speed_ = Map(joycon_left_msg.stick_y, 0, 4096, -40.0, 40.0);
    }

    void
    BeienPaintNode::JoyconRightCallback(const JoyconRight &joycon_right_msg)
    {
        angle_ = Map(joycon_right_msg.stick_x, 0, 4096, -40.0, 40.0);
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

    void
    BeienPaintNode::PushCommand()
    {
        PlcCommand plc_cmd;
        plc_cmd.source_id = "BeienPaintNode";
        plc_cmd.start_address = 3; // 假设起始地址为1
        plc_cmd.register_values = {static_cast<uint16_t>(angle_), static_cast<uint16_t>(speed_)};
        plc_cmd.priority = 1;            // 优先级
        plc_cmd.op_type = 0;             // 假设1代表写操作
        plc_cmd.write_mode = 1;          // 假设1代表覆盖模式
        plc_cmd.expire_duration.sec = 1; // 1秒后过期
        plc_cmd.expire_duration.nanosec = 0;
        plc_cmd.header.stamp = this->now();
        this->plc_command_pub_->publish(plc_cmd);
        RCLCPP_INFO(this->get_logger(), "数据写入--速度: %f  角度: %f", speed_, angle_);
    }
} // namespace BeienPaint