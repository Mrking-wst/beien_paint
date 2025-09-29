#include <locale.h>

#include "keyboard_control_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pkg_beien_paint_msgs/msg/plc_command.hpp"

namespace KeyboardControl
{
    using pkg_beien_paint_msgs::msg::PlcCommand;

    KeyboardControlNode::KeyboardControlNode(const std::string &nodeName)
        : Node(nodeName), speed_(0), is_exit_(false)
    {
        this->declare_parameter<int>("base_speed", 10);
        this->declare_parameter<int>("base_angle", 10);

        this->get_parameter<int>("base_speed", base_speed_);
        this->get_parameter<int>("base_angle", base_angle_);

        RCLCPP_INFO(this->get_logger(), "节点初始化");

        this->plc_command_pub_ = this->create_publisher<PlcCommand>("/plc/plc_command", 10);

        // 设置为用户环境的 locale，支持 UTF-8
        setlocale(LC_ALL, "");
        // 初始化 ncurses
        initscr();
        system("stty -icanon -echo"); // 关闭终端回显和缓冲
        cbreak();
        noecho();
        nodelay(stdscr, TRUE); // getch 非阻塞
        keypad(stdscr, TRUE);

        // 显示提示信息在顶部第一行
        // const char *info = "==键盘控制小车: W/S/A/D 前后左右,x 清零,shift+q/e 加/减速(w/s刷新), q 退出==";
        const wchar_t *info = L"==键盘控制小车: W/S/A/D 前后左右,x 清零,shift+q/e 加/减速(w/s刷新), q 退出==";
        // mvprintw(0, 0, "%s", info); // 第0行，第0列
        mvaddwstr(0, 0, info);
        refresh();

        this->poll_time_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                                   std::bind(&KeyboardControlNode::TimerCallback, this));
    }

    KeyboardControlNode::~KeyboardControlNode()
    {
        // 结束 ncurses
        endwin();
    }

    void
    KeyboardControlNode::TimerCallback()
    {
        // 读取键盘
        int ch = getch(); // 非阻塞

        switch (ch)
        {
        case 'w':
            speed_ = base_speed_;
            break;
        case 's':
            speed_ = base_speed_ * -1;
            break;
        case 'a':
            base_angle_ -= 5;
            if (base_angle_ < min_angle_)
                base_angle_ = min_angle_;
            break;
        case 'd':
            base_angle_ += 5;
            if (base_angle_ > max_angle_)
                base_angle_ = max_angle_;
            break;
        case 'Q': // Ctrl+q
            base_speed_ += 5;
            if (base_speed_ > max_speed_)
                base_speed_ = max_speed_;
            RCLCPP_INFO(this->get_logger(), "基础速度+5 -> %d", base_speed_);
            break;
        case 'E': // Ctrl+e
            base_speed_ -= 5;
            if (base_speed_ < min_speed_)
                base_speed_ = min_speed_;
            RCLCPP_INFO(this->get_logger(), "基础速度-5 -> %d", base_speed_);
            break;
        case 'q':
            base_speed_ = 0;
            base_angle_ = 0;
            speed_ = 0;
            RCLCPP_INFO(this->get_logger(), "速度和角度重置");
            RCLCPP_INFO(this->get_logger(), "退出");
            is_exit_ = true;
            break;
        case 'x': // 重置
            base_speed_ = 0;
            base_angle_ = 0;
            speed_ = 0;
            RCLCPP_INFO(this->get_logger(), "速度和角度重置");
            break;
        case ERR: // 没有按键
            break;
        }

        PlcCommand msg;
        msg.start_address = 3;
        msg.register_values = {static_cast<uint16_t>(base_angle_), static_cast<uint16_t>(speed_)};
        msg.priority = 1;
        msg.op_type = 0;
        msg.write_mode = 1;
        msg.expire_duration.sec = 1;
        msg.expire_duration.nanosec = 0;
        msg.header.stamp = this->now();
        msg.source_id = "keyboard_control";
        this->plc_command_pub_->publish(msg);

        // 在第2行显示当前状态
        mvprintw(2, 0, "当前速度: %d  当前角度: %d    ", speed_, base_angle_);
        mvprintw(3, 0, "=========================\n");
        refresh();

        if (is_exit_)
            rclcpp::shutdown();
    }
} // namespace KeyboardControl
