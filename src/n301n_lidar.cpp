#include "n301n_lidar.h"

namespace n301_lidar_driver {
n301n_lidar::n301n_lidar(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io)
    :port_(port), baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_)
{
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
}

n301n_lidar::~n301n_lidar()
{
    serial_.close();
}

void n301n_lidar::poll(sensor_msgs::LaserScan::Ptr scan, int angle_disable_min, int angle_disable_max)
{
    uint8_t temp_char;
    uint8_t start_count = 0;
    bool got_scan = false;

    /*
     * The fomart of a packet:
     * <start> <index> <speed_L> <speed_H> [Data 0] [Data 1] [Data 2] [Data 3] <checksum_L> <checksum_H>
     * where:
     * start = 0xFA
     * index = [0xA0,0xF9](means packet 0 to 89 with 4 data in one packet)
     * speed = little endian
     * [Data x] = 4 bytes long data,each means:
     *   byte:  |    0    |        1        |     2    |    3
     *   bit :  |  7 - 0  |   7     6   5-0 |   7 - 0  |  7 - 0
     *          |   dis   |invalid,s.w.,dis |  s.s LSB |  s.s MSB
     *   s.w: signal warning
     *   s.s: signal strength
     * */

    { // This is for the newer driver that outputs packets 4 pings at a time
        boost::array<uint8_t, 1980> raw_bytes;
        uint8_t good_sets = 0;
        uint32_t motor_speed = 0;
        rpms=0;
        int index;
        while (!shutting_down_ && !got_scan) {
            // Wait until first data sync of frame: 0xFA, 0xA0
            boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count],1));
            if(start_count == 0) {
                if(raw_bytes[start_count] == 0xFA) {
                    start_count = 1;
                }
            } else if(start_count == 1) {
                if(raw_bytes[start_count] == 0xA0) {
                    start_count = 0;

                    // Now that entire start sequence has been found, read in the rest of the message
                    got_scan = true;

                    boost::asio::read(serial_,boost::asio::buffer(&raw_bytes[2], 1978));

                    scan->angle_min = 0.0;
                    scan->angle_max = 2.0*M_PI;
                    scan->angle_increment = (2.0*M_PI/360.0);
                    scan->range_min = 0.06;
                    scan->range_max = 10.0;
                    scan->ranges.resize(360);
                    scan->intensities.resize(360);

                    //read data in sets of 4
                    for(uint16_t i = 0; i < raw_bytes.size(); i=i+22) {

                        if(raw_bytes[i] == 0xFA && raw_bytes[i+1] == (0xA0+i/22)){

                            good_sets++;
                            motor_speed += (raw_bytes[i+3] << 8) + raw_bytes[i+2]; //accumulate count for avg. time increment
                            rpms=(raw_bytes[i+3]<<8|raw_bytes[i+2])/64;

                            for(uint16_t j = i+4; j < i+20; j=j+4) {
                                index = (4*i)/22 + (j-4-i)/4;
                                // Four bytes per reading
                                uint8_t byte0 = raw_bytes[j];
                                uint8_t byte1 = raw_bytes[j+1];
                                uint8_t byte2 = raw_bytes[j+2];
                                uint8_t byte3 = raw_bytes[j+3];
                                // First two bits of byte1 are status flags
                                // uint8_t flag1 = (byte1 & 0x80) >> 7;  // No return/max range/too low of reflectivity
                                // uint8_t flag2 = (byte1 & 0x40) >> 6;  // Object too close, possible poor reading due to proximity kicks in at < 0.6m
                                // Remaining bits are the range in mm
                                uint16_t range = ((byte1 & 0x3F)<< 8) + byte0;
                                // Last two bytes represent the uncertanty or intensity, might also be pixel area of target...
                                uint16_t intensity = (byte3 << 8) + byte2;
                                double dist = range / 1000.0;
                                if (((359 - index) < angle_disable_max) && ((359-index) > angle_disable_min)  ){
                                    scan->ranges[359 - index] = std::numeric_limits<float>::infinity();
                                }else{
                                    scan->ranges[359 - index] = ( dist < scan->range_min) ? scan->range_max: dist;
                                }
                                scan->intensities[359 - index] = intensity;
                            }


                        }
                    }
                    scan->time_increment = motor_speed/good_sets/1e8;
                }
            }
        }
    }
}

void n301n_lidar::close()
{
    shutting_down_ = true;
}

uint16_t n301n_lidar::checkSum(const uint8_t *p_byte)
{
    uint16_t checksum = 0;
    boost::array<uint16_t, 10> data;

    for(uint8_t i = 0; i < 10; i++){
        uint16_t p1 = p_byte[2*i];
        uint16_t p2 = p_byte[2*i+1];
        data[i] = p1 | (p2 << 8);
    }

    uint32_t check32 = 0;
    for(uint8_t i = 0; i < 10; i++){
        check32 = (check32 << 1) + data[i];
    }

    checksum = (check32 & 0x7FFF) + (check32 >> 15);
    checksum = checksum & 0x7FFF;
    return checksum;
}

}

