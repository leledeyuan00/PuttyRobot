#pragma once

/* public include */
#include "ros/ros.h"
#include <iostream>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <map>
#include <eigen3/Eigen/Dense>
#include <sstream>

#include <moveit_msgs/DisplayRobotState.h>

/* private include */
#include "pararob/motor.h"
#include "pararob/sensor.h"
#include "pararob/kinematic.h"
#include "pararob/parameter.h"
#include "pararob/tool_pos.h"


class Solution
{
public:
    Solution(ros::NodeHandle* nodehandle);
    // virtual ~Solution();

    void run();

private:
    /* system variable */
    ros::NodeHandle nh_;
    ros::Subscriber sensor_sub_;
    ros::Subscriber joint_state_sub_;
    ros::Publisher motor_pub_;
    ros::Publisher data_monitor_;
    ros::Publisher joint_pub_;
    ros::Publisher tool_pub_;

    std::map<std::string, double> mapJoint;

    /* data */
    struct SENSOR sensor_;
    pararob::sensor sensor_msg_;

    struct MOTOR motor_;
    pararob::motor motor_msg_;

    struct PARA_SOLU para_solu_;

    MOTOR_KINE motor_kine_;

    pararob::tool_pos tool_msg_;

    struct UR5_SOLU ur_;
    double joint_state_[6];
    
    /* function */
    /* 初始化 */
    void solution_init(void);
    void state_init(void);
    /* 接口函数 */
    void sensorcall(const pararob::sensor& msg);
    void pub_msg(const MOTOR* motor, const UR5_SOLU* ur, int mod);
    void JointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
    /* 算法 */
    // void motor_traj_generator(SENSOR* sensor, MOTOR* motor);
    void Motor_traj_gen(SENSOR* sensor, MOTOR* motor);
    void low_pass_init(FILTER* fil, float ratio, float init);
    void low_pass_update(FILTER* fil, float in);
};

