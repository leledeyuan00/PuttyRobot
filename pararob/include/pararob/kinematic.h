#pragma once

#include <std_msgs/String.h> 
#include <std_msgs/Float32.h>//　ROS标准msg头文件
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "pararob/parameter.h"
#include "ros/ros.h"

class MOTOR_KINE
{
public:
    MOTOR_KINE(){};
    // virtual ~MOTOR_KINE();
    
    void motor_traj_gen(PARA_SOLU* para);

    void para_plat_init(PARA_SOLU* para);

private:
    /* data */

    /* function */
    /* 获取墙面法向量 */
    Eigen::Vector3f get_wall_plat_vec(Eigen::Vector3f laser_dist);

    /* 获取旋转矩阵 */
    Eigen::Matrix3f normal_vec_rotm(Eigen::Vector3f normal_vec);
    
    Eigen::Matrix<float, 2, 3> get_rot_element(Eigen::Vector3f vec);

    void eul2Rotm(Eigen::Vector3f& euler_ZYX, Eigen::Matrix3f& rotm);

    Eigen::Matrix3f rodrigues(float theta, Eigen::Vector3f vector);

    void inverse_solu(PARA_SOLU* para); 

    void para_plat_update(PARA_SOLU* para);
};

