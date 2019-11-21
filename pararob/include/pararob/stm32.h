#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "pararob/encoder.h"

class STM32
{
public:
    STM32(ros::NodeHandle* nodehandle);

    void run();

private:
    /* data */
    ros::NodeHandle nh_;
    ros::Publisher stm_pub_;

    serial::Serial stm_ser_;

    pararob::encoder encoder_data_;

    void stm_init();
    int encoder_ser_init(serial::Serial* ser, int port);
    float fetch_data(serial::Serial* ser);
};
