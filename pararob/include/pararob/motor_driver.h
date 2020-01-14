#pragma once

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "pararob/motor.h"
#include "pararob/parameter.h"

#define MOTOR_STROKE 27 //mm
#define MOTOR_RES 4095 //16bits

class Motor_driver
{
public:
    Motor_driver(ros::NodeHandle* nodehandle);
    // virtual ~Motor_driver();

    /* 循环发送数据 */
    void run();

private:
    /* data */
    ros::NodeHandle nh_;
    ros::Subscriber motor_sub_;

    serial::Serial motor_ser_;

    unsigned short motor_msg_[3];

    
    /* 初始化 */
    void motor_driver_init();
    int motor_ser_init(serial::Serial* ser, int port);
    /* 校验 */
    unsigned char motor_check_sum(unsigned char* data); 
    /* 发数据 */
    void motor_write(unsigned short* msg);
    
    void drivercallback(const pararob::motor& msg);
};
