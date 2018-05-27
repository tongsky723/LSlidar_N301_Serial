#include <ros/ros.h>

#include <n301n_lidar.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "n301n_publisher");

    n301_lidar_driver::n301n_lidar laser;
    
    return 0;
}
