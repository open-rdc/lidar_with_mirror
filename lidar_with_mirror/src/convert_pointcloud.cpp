#include "lidar_with_mirror/convert_pointcloud.h"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

ConvertPointCloud::ConvertPointCloud()
: Node("convert_pointcloud_node") // Initialize the Node with a name
{
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 100, std::bind(&ConvertPointCloud::scanCallback, this, std::placeholders::_1));

    point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud", 100);

    // For TF2, we will use a buffer and a listener
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto listener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
}

void ConvertPointCloud::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    sensor_msgs::msg::PointCloud2 cloud;
    laser_geometry::LaserProjection projector_;
    // We use the tf2_sensor_msgs::doTransform function to transform the scan into a point cloud
    projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, *tfBuffer_);
    point_cloud_publisher_->publish(cloud);
}
