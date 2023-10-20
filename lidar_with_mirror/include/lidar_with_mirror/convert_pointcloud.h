#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <laser_geometry/laser_geometry.hpp>

class ConvertPointCloud : public rclcpp::Node {
     public:
        ConvertPointCloud();
        void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

     private:
        laser_geometry::LaserProjection projector_;
        std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
        std::shared_ptr<tf2_ros::TransformListener> tfListener_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};
