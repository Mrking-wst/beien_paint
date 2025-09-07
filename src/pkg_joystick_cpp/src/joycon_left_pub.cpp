#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "joycon_left_node.hpp"
#include "pkg_beien_paint_msgs/msg/joycon_left.hpp"

using pkg_beien_paint_msgs::msg::JoyconLeft;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<JoyStick::JoyconLeftNode> joyconLeftNode = std::make_shared<JoyStick::JoyconLeftNode>("joycon_left_node");

  rclcpp::spin(joyconLeftNode);

  rclcpp::shutdown();

  joyconLeftNode->Dispose();

  return 0;
}
