#ifndef PLC_COMMUNICATE_NODE_HPP
#define PLC_COMMUNICATE_NODE_HPP

#include <string>
#include <chrono>
#include <thread>
#include <vector>
#include <unordered_map>
#include <memory>
#include <functional>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "pkg_beien_paint_msgs/msg/joycon_left.hpp"
#include "pkg_beien_paint_msgs/msg/joycon_right.hpp"
#include "pkg_beien_paint_msgs/msg/car_info.hpp"
#include "modbus/modbus.h"
#include "pkg_beien_paint_msgs/msg/plc_command.hpp"
#include "plc_scheduler.hpp"

namespace PlcCommunicate
{
    using pkg_beien_paint_msgs::msg::CarInfo;
    using pkg_beien_paint_msgs::msg::PlcCommand;
    using std_msgs::msg::UInt16MultiArray;

    class PlcCommunicateNode
        : public rclcpp::Node
    {
    public:
        PlcCommunicateNode(const std::string &nodeName);
        void Dispose();

    private:
        void ConnectModbus();
        void PollModbus();
        void TryReconnectModbus();
        void PushCommand();
        void SourceDataReceivedCallback(const UInt16MultiArray &sourse_data);
        void PlcCommandReceivedCallback(const PlcCommand &plc_command);
        void InitializeRegistersMapping();
        float Map(float x, float in_min, float in_max, float out_min, float out_max);
        float Constrain(float x, float out_min, float out_max);
        uint16_t GetUint16Fromfloat(float value);

    private:
        modbus_t *ctx_;
        rclcpp::Publisher<UInt16MultiArray>::SharedPtr source_data_pub_;
        rclcpp::Publisher<CarInfo>::SharedPtr car_info_pub_;
        rclcpp::Subscription<UInt16MultiArray>::SharedPtr source_data_sub_;
        rclcpp::Subscription<PlcCommand>::SharedPtr plc_command_sub_;
        rclcpp::TimerBase::SharedPtr poll_time_;
        rclcpp::TimerBase::SharedPtr reconnect_time_;
        std::thread reconnect_loop_thread_;
        std::string plc_ip_;
        int plc_port_;
        int read_interval_;
        int reconnect_interval_;
        int read_holding_register_start_address_;
        int read_holding_register_num_;
        int write_holding_register_start_address_;
        int write_holding_register_num_;
        int response_timeout_;
        bool is_connected_;
        std::unordered_map<std::string, int> registers_mapping_;
        PlcScheduler plc_scheduler_;
        rclcpp::TimerBase::SharedPtr scheduler_timer_;
        std::vector<uint16_t> sourceReadingData_;
    };
} // namespace PlcCommunicate

#endif //  PLC_COMMUNICATE_NODE_HPP