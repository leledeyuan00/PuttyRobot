#pragma once

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "pararob/sensor.h"
#include <sstream>
#include <stdlib.h>
#include <iostream>
#include "pararob/parameter.h"

class Laser_talker
{
public:
    Laser_talker(ros::NodeHandle* nodehandle);
    // virtual ~laser_talker();

    /* 循环发送数据 */
    void run();
private:
    /* data */
    ros::NodeHandle nh_;
    ros::Publisher laser_pub_;

    serial::Serial laser1_ser_;
    serial::Serial laser2_ser_;
    serial::Serial laser3_ser_;

    pararob::sensor laser_data_;

    /* function */

    /* 初始化 */
    void laser_init();

    int laser_ser_init(serial::Serial* ser, int port);
    /* 获取数据 */
    double fetch_data(serial::Serial* ser);
    

};



