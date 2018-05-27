#ifndef N301N_LIDAR_H
#define N301N_LIDAR_H

#include <sensor_msgs/LaserScan.h>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>
#include <ros/ros.h>

namespace n301_lidar_driver {
class n301n_lidar
{
public:

    uint16_t rpms;

    /**
     */
    n301n_lidar();

    /**
      * @brief Default destructor
      */
    ~n301n_lidar();

    /**
       * @brief Poll the laser to get a new scan. Blocks until a complete new scan is received or close is called.
       * @param scan LaserScan message pointer to fill in with the scan. The caller is responsible for filling in the ROS timestamp and frame_id
       */
    void poll(sensor_msgs::LaserScan::Ptr scan);

    /**
    * @brief check the laser recieved in 1s
    * */
    void checkLaserReceived(const ros::TimerEvent& event);
  
    /**
      * @brief Close the driver down and prevent the polling loop from advancing
      */
    void close();

private:
    /**
     * @brief checkSum
     * @param p_byte
     * @return
     */
    uint16_t checkSum(const uint8_t *p_byte);

    void run();

    std::string port_; ///< @brief The serial port the driver is attached to
    int baud_rate_; ///< @brief The baud rate for the serial connection
    double duration_;
    std::string frame_id_;
    double max_range_;
    double min_range_;


    bool shutting_down_; ///< @brief Flag for whether the driver is supposed to be shutting down or not
    boost::asio::serial_port *serial_; ///< @brief Actual serial port object for reading/writing to the n301n lidar Scanner
    uint16_t motor_speed_; ///< @brief current motor speed as reported by the n301n lidar.

    ros::Timer check_laser_timer_;
    ros::Time last_laser_received_ts_;
    ros::Duration laser_check_interval_;

    ros::Publisher laser_pub_;
    ros::Publisher motor_pub_;

    boost::thread * laser_thread_;
    
    

};

}

#endif // N301N_LIDAR_H
