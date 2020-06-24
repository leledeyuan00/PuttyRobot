#pragma once

#define SIMULATE

#include "eigen3/Eigen/Dense"

#define PI 3.1415926
#define MOTOR_NUM 3
#define UR_JOINT_NUM 6
#define ALL_JOINT_NUM (MOTOR_NUM + UR_JOINT_NUM)

#define MOTORPLAT_RADIUS 0.06
#define LASERPLAT_RADIUS 0.120
#define MOTOR_LEN_INIT  0.12496
#define LASER_DIST_INIT 0.16840

#define MAX_UR_JOINT  2*PI
#define MIN_UR_JOINT -2*PI
#define MAX_PARA_LEN 0.147
#define MIN_PARA_LEN 0.12496

#define UR_JNT_MEAN   (MAX_UR_JOINT+MIN_UR_JOINT)/2
#define PARA_LEN_MEAN (MAX_PARA_LEN+MIN_PARA_LEN)/2

#define DIS_RANGE 1.5*(MAX_PARA_LEN - MIN_PARA_LEN)
enum PARREL_STATE
{
    LAST = 0,
    CURRENT,
    NEXT,
    ALL_STATE
};


struct FILTER
{
    float ratio;
    float out;
};

struct SENSOR
{
    struct FILTER fil[MOTOR_NUM];
    float data[MOTOR_NUM];
    unsigned short state;
};

struct MOTOR
{
    float current[MOTOR_NUM];
    struct FILTER fil[MOTOR_NUM];
    float tool_pos[MOTOR_NUM];
    float tool_error[MOTOR_NUM];
    float cmd[MOTOR_NUM];
};

struct PARA_SOLU
{
    Eigen::Matrix3f rot_matrix[ALL_STATE];
    Eigen::Vector3f laser_dist;
    Eigen::Vector3f motor_dist[ALL_STATE];
    float laser_cor_z [ALL_STATE];
    float motor_cor_z [ALL_STATE];
    float delta_mm    [MOTOR_NUM];
    Eigen::Matrix3f jacobian;
    Eigen::Matrix3f xyz;
    Eigen::Vector3f xyz_v;
    bool state;
};

struct UR5_SOLU
{
    double joint_cur[UR_JOINT_NUM];
    double joint_next[UR_JOINT_NUM];
    double transform_matrix[16];
    Eigen::Vector3f tool_xyz;
    bool state;
};

struct PID_EFF
{
    double error;
    double error_last;
    double error_last2;
    double kp;
    double ki;
    double kd;
};


