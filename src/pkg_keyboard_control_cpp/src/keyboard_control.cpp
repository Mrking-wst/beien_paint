#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "pkg_beien_paint_msgs/msg/plc_command.hpp"
#include "keyboard_control_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<KeyboardControl::KeyboardControlNode> keyboard_control = std::make_shared<KeyboardControl::KeyboardControlNode>("keyboard_control_node");
  rclcpp::spin(keyboard_control);
  rclcpp::shutdown();

  return 0;
}
