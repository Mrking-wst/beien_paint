#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "plc_communicate_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::cout << "plc communicate" << std::endl;
    std::shared_ptr<PlcCommunicate::PlcCommunicateNode> plc = std::make_shared<PlcCommunicate::PlcCommunicateNode>("plc_node");
    rclcpp::spin(plc);
    rclcpp::shutdown();
    plc->Dispose();
    return 0;
}