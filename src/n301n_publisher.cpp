#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <std_msgs/UInt16.h>
#include <n301n_lidar.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "n301n_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    std::string port;
    int baud_rate;
    int angle_disable_min;
    int angle_disable_max;
    std::string frame_id;

    std_msgs::UInt16 rpms;

    priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
    priv_nh.param("baud_rate", baud_rate, 230400);
    priv_nh.param("frame_id", frame_id, std::string("laser"));
    priv_nh.param("angle_disable_min", angle_disable_min, -1);
    priv_nh.param("angle_disable_max", angle_disable_max, -1);
    priv_nh.param("baud_rate", baud_rate, 230400);

    boost::asio::io_service io;
    try{
        n301_lidar_driver::n301n_lidar laser(port, baud_rate, io);
        ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
        ros::Publisher motor_pub = nh.advertise<std_msgs::UInt16>("rpms", 1000);

        while(ros::ok()){
            sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
            if(scan == NULL)
                ROS_INFO("SCAN == NULL");
            scan->header.frame_id = frame_id;
            scan->header.stamp = ros::Time::now();
            laser.poll(scan, angle_disable_min, angle_disable_max);
            
            rpms.data = laser.rpms;
            laser_pub.publish(scan);
            motor_pub.publish(rpms);
            ROS_INFO("start SCAN");
            time_t tmNow = time(NULL);
                    char ctm[26];
                    tm *ptmNow = localtime(&tmNow);
                    std::strftime(ctm, 26,"%Y-%m-%d %H:%M:%S", ptmNow);
                    ROS_INFO("%s", ctm);
        }
        laser.close();
        return 0;
    }
    catch (boost::system::system_error ex){
        ROS_ERROR("Error instantiating laser object. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
        return -1;
    }
}
