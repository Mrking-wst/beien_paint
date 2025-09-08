#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "joycon_right_node.hpp"
#include "pkg_beien_paint_msgs/msg/joycon_right.hpp"

using pkg_beien_paint_msgs::msg::JoyconRight;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<JoyStick::JoyconRightNode> joyconRightNode = std::make_shared<JoyStick::JoyconRightNode>("joycon_right_node");

  rclcpp::spin(joyconRightNode);

  rclcpp::shutdown();

  joyconRightNode->Dispose();

  return 0;
}
