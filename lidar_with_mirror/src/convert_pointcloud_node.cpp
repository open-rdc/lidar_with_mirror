#include "lidar_with_mirror/convert_pointcloud.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "convert_pointcloud");
    convert_pointcloud filter;
    ros::spin();
    return 0;
}

