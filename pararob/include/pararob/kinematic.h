#pragma once

/* public include */
#include <std_msgs/String.h> 
#include <std_msgs/Float32.h>//　ROS标准msg头文件
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "ros/ros.h"
#include <ros/spinner.h>
#include <ur_kinematics/ur_kin.h>
// #include <stdlib.h>

#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
// // MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <stdlib.h>
#include <time.h>
/* private include */
#include "pararob/parameter.h"

// #define DEBUG

using namespace ur_kinematics;

class MOTOR_KINE
{
public:
    MOTOR_KINE();
    // virtual ~MOTOR_KINE();
    
    void motor_traj_gen(PARA_SOLU* para, UR5_SOLU* ur);

    void robot_state_init(PARA_SOLU* para, UR5_SOLU* ur);

private:
    /* system state */
    ros::AsyncSpinner spinner;
    const std::string PLANNING_GROUP;
    moveit::planning_interface::MoveGroupInterface move_group_;
    const robot_state::JointModelGroup* joint_model_group_;
    moveit::core::RobotStatePtr current_state_;

    /*旋转矩阵转欧拉角后的角度为-π ～ π，当欧拉角超过pi时，欧拉角会发生突变，因此将欧拉角的范围扩展为-2π ～ 2π， 
    referenced_euler_zyx用于记录上次的欧拉角，每次将计算的欧拉角与上一次的欧拉角做比较，获取距离最近的欧拉角，避免突变*/
    Eigen::Vector3d referenced_euler_zyx;

    /* variable */
    Eigen::Matrix3f para_fram_base_;
    Eigen::VectorXd joint_mean_;
    Eigen::VectorXd joint_mini_;
    Eigen::Matrix3f rot_matrix_d_;
    int recored_z_;
    double current_z_,current_x_;
    /* function */
    /* UR function */
    Eigen::MatrixXd getJacobian(void);

    Eigen::VectorXd perform_func(Eigen::VectorXd joint, float k1, float k2);

    /* para plat function */
    /* 获取墙面法向量 */
    Eigen::Vector3f get_wall_plat_vec(Eigen::Vector3f laser_dist);

    /* 并联平台雅克比 */
    Eigen::Matrix3f para_jacobian(Eigen::Matrix3f rotm, Eigen::Vector3f length, Eigen::Matrix3f xyz);

    /* 并联平台正解 */
    Eigen::Matrix3f para_forward(PARA_SOLU* para);

    /* 伪逆 */
    Eigen::MatrixXd pinv(Eigen::MatrixXd A);

    /* 获取旋转矩阵 */
    Eigen::Matrix3f normal_vec_rotm(Eigen::Vector3f normal_vec);
    
    Eigen::Matrix<float, 2, 3> get_rot_element(Eigen::Vector3f vec);

    Eigen::Vector3f inverse_solu(Eigen::Matrix3f rotm, float top_z, Eigen::Matrix3f& xzy,Eigen::Vector3f& xyz_v); 

    void para_plat_update(PARA_SOLU* para);

    /* public function */
    
    void eul2Rotm(Eigen::Vector3f& euler_ZYX, Eigen::Matrix3f& rotm);
    
    Eigen::Vector3f rotm2Eul(Eigen::Vector3f vec_in);
    
    float max_error(Eigen::Vector3f vector);
    // void ur_transform_matrix(float *p, Eigen::Matrix4f& matrix);

    #ifdef DEBUG
    void test_func(void);
    #endif
};

