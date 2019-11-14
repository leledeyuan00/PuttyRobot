#pragma once

#include "ros/ros.h"
#include "pararob/motor.h"
#include "pararob/sensor.h"
#include <iostream>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include "pararob/kinematic.h"
#include "pararob/parameter.h"


class Solution
{
public:
    Solution(ros::NodeHandle* nodehandle);
    // virtual ~Solution();

    void run();

private:
    /* data */
    ros::NodeHandle nh_;
    ros::Subscriber sensor_sub_;
    ros::Publisher motor_pub_;
    ros::Publisher data_monitor_;

    struct SENSOR sensor_;
    pararob::sensor sensor_msg_;


    struct MOTOR motor_;
    // float motor_current_[3];
    // float motor_cmd_[3];
    pararob::motor motor_msg_;

    struct PARA_SOLU para_solu_;

    MOTOR_KINE motor_kine_;
    
    /* function */
    /* 初始化 */
    void solution_init();
    /* 接口函数 */
    void sensorcall(const pararob::sensor& msg);
    void pub_msg(const MOTOR* motor);

    /* 算法 */
    // void motor_traj_generator(SENSOR* sensor, MOTOR* motor);
    void Motor_traj_gen(SENSOR* sensor, MOTOR* motor);
    void low_pass_init(FILTER* fil, float ratio, float init);
    void low_pass_update(FILTER* fil, float in);
};

