#ifndef __PARA_H__
#define __PARA_H__

// #define SIMULATE

#include <iostream>
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <boost/shared_ptr.hpp>
#include "sensor_msgs/LaserScan.h"
#ifndef SIMULATE
#include <ppr_msgs/laser.h>
#endif


#include "eigen3/Eigen/Dense"
#include <math.h>

#include <control_toolbox/pid.h>


#define PI 3.1415926
#define MOTOR_NUM 3

#define MOTORPLAT_RADIUS 0.06
#define LASERPLAT_RADIUS 0.1175
#define MOTOR_LEN_INIT  0.1049
#define LASER_DIST_UPPER 0.0245

#define MAX_PARA_LEN 0.1319
#define MIN_PARA_LEN 0.1049
#define PARA_LEN_MEAN (MAX_PARA_LEN+MIN_PARA_LEN)/2

enum PARREL_STATE
{
    LAST = 0,
    CURRENT,
    NEXT,
    ALL_STATE
};

struct Joint
{
    ros::Publisher pub;
    ros::Subscriber sub;
    float stat;
    float cmd;
    float laser;
};

struct para_info
{
    Eigen::Matrix3d rot_matrix[ALL_STATE];
    Eigen::Vector3d laser_dist;
    Eigen::Vector3d motor_dist[ALL_STATE];
    bool state;
};

struct inverse_info
{
    Eigen::Matrix3d n_vector3;
    Eigen::Vector3d target_dist;
};



class para
{
public:
    para(ros::NodeHandle &nh);

    Eigen::Matrix3d get_ppr_r(void);
    float get_wall_dist(void);
    float get_ppr_dist(void);
    uint16_t get_laser_state(void);
    void control_loop(void);


    // void run(void);
private:

    ros::NodeHandle nh_;
    std::vector<Joint> para_motor_;
    #ifndef SIMULATE
    // practice variable
    uint16_t laser_state_;
    #else
    // simulation variable
    uint16_t laser_state_;
    #endif
    para_info para_;
    float current_wall_dist_;
    float current_ppr_dist_;
    uint16_t current_laser_state_;

    Eigen::Matrix3d top_fram_, base_fram_;

    std::vector<control_toolbox::Pid> pid_controllers_;

    inverse_info para_inverse_[ALL_STATE];

    
    /* function */
    void state_init(void);
    void ros_init(void);
    void pub_msgs(void);
    #ifndef SIMULATE
    void laser_callback1(const ppr_msgs::laser &msg);
    #else
    void laser_callback1(const sensor_msgs::LaserScan &msg);
    void laser_callback2(const sensor_msgs::LaserScan &msg);
    void laser_callback3(const sensor_msgs::LaserScan &msg);
    #endif
    // Eigen::Vector3d inverse_solu(Eigen::Matrix3d rotm, float top_z, Eigen::Matrix3d& xzy,Eigen::Vector3d& xyz_v);
    inverse_info inverse_solu(Eigen::Matrix4d trans);
    Eigen::Vector3d rotm2Eul(Eigen::Vector3d vec_in);
    Eigen::Matrix3d normal_vec_rotm(Eigen::Vector3d normal_vec);
    Eigen::Matrix<double, 2, 3> get_rot_element(Eigen::Vector3d vec);
    Eigen::Vector3d get_wall_plat_vec(Eigen::Vector3d laser_dist);
    float max_error(Eigen::Vector3d vector);
    Eigen::Matrix3d eul2Rotm(Eigen::Vector3d& euler_ZYX);
    Eigen::Matrix3d Jacobian(Eigen::Matrix3d rotm, Eigen::Vector3d length, Eigen::Matrix3d xyz);

    Eigen::Matrix4d forward(Eigen::Matrix4d last_T, Eigen::Matrix3d last_nv3, Eigen::Vector3d last_length, Eigen::Vector3d current_length);

    Eigen::Matrix4d makeTrans(Eigen::Matrix3d R, Eigen::Vector3d V);

    Eigen::Matrix3d get_forward();

};


#endif