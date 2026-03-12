#include <string>

#include "beien_paint_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace BeienPaint
{
    BeienPaintNode::BeienPaintNode(const std::string &node_name)
        : Node(node_name), speed_(0.0), angle_(0.0),lifting_raise_(false),lifting_down_(false),
          heart_beat_(0),
          zl_(false),
          zr_(false),
          l_(false),
          r_(false),
          is_emergency_stop_(false),
          is_locking_(false),
          lock_(false)
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

        if(joycon_left_msg.zl){
            speed_ = Map(joycon_left_msg.stick_y, 0, 4096, -40.0, 40.0);
        }
        else{
            speed_ = 0.0;
        }

        if(joycon_left_msg.l){
            lifting_raise_ = joycon_left_msg.up;
            lifting_down_ = joycon_left_msg.down;
        }
        else{
            lifting_raise_ = false;
            lifting_down_ = false;
        }

        //  上锁 按下一次
        if(lock_ != joycon_left_msg.function){
            if(joycon_left_msg.function){
            is_locking_ = !is_locking_;
            }
            lock_ = joycon_left_msg.function;
        }

        if(is_locking_){
            speed_ = 0.0;
            angle_ = 0.0;
            lifting_raise_ = false;
            lifting_down_ = false;
        }
    }

    void
    BeienPaintNode::JoyconRightCallback(const JoyconRight &joycon_right_msg)
    {
        if(joycon_right_msg.zr && !is_locking_){
            angle_ = Map(joycon_right_msg.stick_x, 0, 4096, -40.0, 40.0);
        }
        else{
            angle_ = 0.0;
        }

        if(joycon_right_msg.b){
            is_emergency_stop_ = true;
        }
        else{
            is_emergency_stop_ = false;
        }

        if(is_emergency_stop_){
            lifting_raise_ = false;
            lifting_down_ = false;
            angle_ = 0.0;
            speed_ = 0.0;
        }
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
        if(heart_beat_>20) heart_beat_ = 0;
        PlcCommand plc_cmd;
        plc_cmd.source_id = "BeienPaintNode";
        //  功能     寄存器地址    VW
        //  心跳        30       60
        //  上升        31       62
        //  下降        32       64
        //  急停        33       66
        //  保留        34       68
        //  角度        35       70
        //  保留        36       72
        //  速度        37       74
        plc_cmd.start_address = 30; // 假设起始地址为1
        plc_cmd.register_values = {static_cast<uint16_t>(heart_beat_),static_cast<uint16_t>(lifting_raise_),static_cast<uint16_t>(lifting_down_),static_cast<uint16_t>(is_emergency_stop_),0,static_cast<uint16_t>(angle_), 0,static_cast<uint16_t>(speed_)};
        plc_cmd.priority = 1;            // 优先级
        plc_cmd.op_type = 0;             // 假设1代表写操作
        plc_cmd.write_mode = 1;          // 假设1代表覆盖模式
        plc_cmd.expire_duration.sec = 1; // 1秒后过期
        plc_cmd.expire_duration.nanosec = 0;
        plc_cmd.header.stamp = this->now();
        this->plc_command_pub_->publish(plc_cmd);
        RCLCPP_INFO(this->get_logger(), "数据写入--速度: %f  角度: %f 上升：%d 下降：%d 急停：%d 锁：%d\n\n", speed_, angle_, lifting_raise_, lifting_down_, is_emergency_stop_, is_locking_);
        heart_beat_ += 1;
    }
} // namespace BeienPaint
