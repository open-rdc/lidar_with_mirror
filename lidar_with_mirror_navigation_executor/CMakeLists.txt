cmake_minimum_required(VERSION 3.5)
project(lidar_with_mirror_navigation_executor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED) # Assuming you're using Nav2 for navigation
find_package(rclcpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
