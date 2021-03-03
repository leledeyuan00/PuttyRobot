#ifndef __PARA_H__
#define __PARA_H__

// #define SIMULATE
//#define RDKINEMATIC


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

#include "para_solu/kinematic.h"


#define PI 3.1415926
#define MOTOR_NUM 3

#define MOTORPLAT_RADIUS 0.06
#define LASERPLAT_RADIUS 0.1175
#define LASER_DIST_UPPER 0.0245

// #define MAX_PARA_LEN 0.1319
// #define MIN_PARA_LEN 0.1049
#define MAX_PARA_LEN 0.1300
#define MIN_PARA_LEN 0.1065
#define MID_PARA_LEN (MAX_PARA_LEN+MIN_PARA_LEN)/2
#define PARA_CMD_INIT (MAX_PARA_LEN - MIN_PARA_LEN)/2

using namespace macmic_kinematic;

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

struct inverse_info
{
    Eigen::Matrix3d n_vector3;
    Eigen::Vector3d target_dist;
};

struct Jacobian_info
{
    Eigen::Matrix3d j3;
    Eigen::Matrix<double,6,3> trans;
};

struct para_info
{
    Eigen::Matrix4d trans_matrix;
    Eigen::Matrix3d rot_matrix[ALL_STATE];
    Eigen::Vector3d laser_dist;
    Eigen::Vector3d motor_dist[ALL_STATE];
    inverse_info inverse[ALL_STATE];
    Jacobian_info jacobian;
    bool state;
};


class para
{
public:
    para(ros::NodeHandle &nh);

    Eigen::Matrix3d get_ppr_r(void);
    float get_wall_dist(void);
    Eigen::Vector3d get_ppr_dist(void);
    uint16_t get_laser_state(void);
    Eigen::Vector3d get_laser_dist(void);
    bool get_error_detect(void);
    Eigen::Vector3d get_laser_raw(void);
    bool get_ppr_state(void);
    Eigen::Matrix<double,6,3> get_Jacobian(void); // unTransformmed Jacobian
    void control_loop(void);

    void set_ppr_incline(PPR_INCLINE incline);

    // for redundant kinematics
    void ppr_update(void);
    void pub_msgs(Eigen::Vector3d cmd);


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

    std::vector<std::shared_ptr<LowPassFilter>> laser_filters_; 

    Eigen::Matrix3d top_fram_, base_fram_;

    std::vector<control_toolbox::Pid> pid_controllers_;

    // control param
    
    PPR_INCLINE ppr_incline_;

    bool state_error_;
    bool ppr_init_;

    Eigen::Vector3d laser_sensors_raw_;
    
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
    Eigen::Matrix3d eul2Rotm(Eigen::Vector3d euler_ZYX);
    Jacobian_info Jacobian(Eigen::Matrix3d rotm, Eigen::Vector3d length, Eigen::Matrix3d xyz);

    Eigen::Matrix4d forward(Eigen::Matrix4d last_T, Eigen::Matrix3d last_nv3, Eigen::Vector3d last_length, Eigen::Vector3d current_length);

    Eigen::Matrix4d makeTrans(Eigen::Matrix3d R, Eigen::Vector3d V);

    Eigen::Matrix3d get_forward();

};


#endif