#include "pse_node.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Fusion::PSE::PseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
