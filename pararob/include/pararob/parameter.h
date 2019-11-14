#pragma once

#include "eigen3/Eigen/Dense"

#define PI 3.1415926
#define MOTOR_NUM 3

#define MOTORPLAT_RADIUS 0.06
#define LASERPLAT_RADIUS 0.120
#define MOTOR_LEN_INIT  0.12496
#define LASER_DIST_INIT 0.16840

// #define LASER_DATA_PER 

enum PARREL_STATE
{
    LAST = 0,
    CURRENT,
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
    Eigen::Vector3f motor_dist;
    float laser_cor_z [ALL_STATE];
    float motor_cor_z [ALL_STATE];
    Eigen::Vector3f rot_vectors;
};


