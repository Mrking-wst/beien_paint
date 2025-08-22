#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::cout << "plc communicate" << std::endl;
    std::shared_ptr<rclcpp::Node> testNode = std::make_shared<rclcpp::Node>("test_node");
    rclcpp::spin(testNode);
    rclcpp::shutdown();

    return 0;
}