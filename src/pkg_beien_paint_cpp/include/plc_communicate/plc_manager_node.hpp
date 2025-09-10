#ifndef PLC_MANAGER_NODE_HPP
#define PLC_MANAGER_NODE_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pkg_beien_paint_msgs/msg/joycon_left.hpp"
#include "pkg_beien_paint_msgs/msg/joycon_right.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

namespace BeienPaint
{
    using pkg_beien_paint_msgs::msg::JoyconLeft;
    using pkg_beien_paint_msgs::msg::JoyconRight;
    using std_msgs::msg::UInt16MultiArray;

    class PlcManagerNode : public rclcpp::Node
    {
    public:
        PlcManagerNode(const std::string &nodeName);

    private:
        void plcFeedbackCallback(const UInt16MultiArray::SharedPtr msg);
        void joyconLeftCallback(const JoyconLeft::SharedPtr msg);
        void joyconRightCallback(const JoyconRight::SharedPtr msg);

    private:
        rclcpp::Subscription<UInt16MultiArray>::SharedPtr plc_feedback_; //  订阅PLC数据，将获取的数据分块解析，然后发布，供各个功能模块使用
        rclcpp::Publisher<UInt16MultiArray>::SharedPtr plc_command_;     //  发布给PLC的命令数据，将各个功能模块的命令数据打包后发送给PLC
        rclcpp::Subscription<JoyconLeft>::SharedPtr joycon_left_;        //  订阅左手手柄数据
        rclcpp::Subscription<JoyconRight>::SharedPtr joycon_right_;      //  订阅右手手柄数据
    };
} // namespace BeienPaint

#endif //  PLC_MANAGER_NODE_HPP