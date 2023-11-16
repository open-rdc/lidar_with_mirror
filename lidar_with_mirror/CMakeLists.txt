cmake_minimum_required(VERSION 3.5)
project(lidar_with_mirror)

# Default to C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

# Find necessary dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
 include
 ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

# Declare a C++ executable
add_executable(convert_pointcloud_node src/convert_pointcloud_node.cpp src/convert_pointcloud.cpp)
ament_target_dependencies(convert_pointcloud_node rclcpp std_msgs tf2 tf2_ros laser_geometry)

# Specify libraries to link against
target_link_libraries(convert_pointcloud_node ${EIGEN3_LIBRARIES})

# Install targets
install(TARGETS
  convert_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
