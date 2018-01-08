#ifndef N301N_LIDAR_H
#define N301N_LIDAR_H

#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>

namespace n301_lidar_driver {
class n301n_lidar
{
public:

    uint16_t rpms;

    /**
      * @brief Constructs a new XV11Laser attached to the given serial port
      * @param port The string for the serial port device to attempt to connect to, e.g. "/dev/ttyUSB0"
      * @param baud_rate The baud rate to open the serial port at.
      * @param io Boost ASIO IO Service to use when creating the serial port object
      */
    n301n_lidar(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io);

    /**
      * @brief Default destructor
      */
    ~n301n_lidar();

    /**
       * @brief Poll the laser to get a new scan. Blocks until a complete new scan is received or close is called.
       * @param scan LaserScan message pointer to fill in with the scan. The caller is responsible for filling in the ROS timestamp and frame_id
       */
    void poll(sensor_msgs::LaserScan::Ptr scan, int angle_disable_min, int angle_disable_max);

    /**
      * @brief Close the driver down and prevent the polling loop from advancing
      */
    void close();
private:
    std::string port_; ///< @brief The serial port the driver is attached to
    uint32_t baud_rate_; ///< @brief The baud rate for the serial connection

    bool shutting_down_; ///< @brief Flag for whether the driver is supposed to be shutting down or not
    boost::asio::serial_port serial_; ///< @brief Actual serial port object for reading/writing to the n301n lidar Scanner
    uint16_t motor_speed_; ///< @brief current motor speed as reported by the n301n lidar.

    /**
     * @brief checkSum
     * @param p_byte
     * @return
     */
    uint16_t checkSum(const uint8_t *p_byte);

};

}

#endif // N301N_LIDAR_H
