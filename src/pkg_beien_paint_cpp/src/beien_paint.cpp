#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pkg_beien_paint_msgs/msg/plc_command.hpp"
#include "beien_paint_node.hpp"

using BeienPaint::BeienPaintNode;
using pkg_beien_paint_msgs::msg::PlcCommand;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<BeienPaintNode> beien_paint = std::make_shared<BeienPaintNode>("beien_paint_node");
    rclcpp::spin(beien_paint);
    rclcpp::shutdown();

    return 0;
}