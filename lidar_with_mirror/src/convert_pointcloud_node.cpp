#include "lidar_with_mirror/convert_pointcloud.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto filter_node = std::make_shared<ConvertPointCloud>();
    rclcpp::spin(filter_node);
    rclcpp::shutdown();
    return 0;
}
