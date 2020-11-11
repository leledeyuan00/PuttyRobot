#ifndef __PARA_SOLU_H__
#define __PARA_SOLU_H__

//#define SIMULATE

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



class para_solu
{
public:
    para_solu(ros::NodeHandle &nh);

    Eigen::Vector3f get_eular(void);

    void run(void);
private:

    ros::NodeHandle nh_;
    std::vector<Joint> para_motor_;
    #ifndef SIMULATE
    void laser_callback1(const ppr_msgs::laser &msg);
    #else
    void laser_callback1(const sensor_msgs::LaserScan &msg);
    void laser_callback2(const sensor_msgs::LaserScan &msg);
    void laser_callback3(const sensor_msgs::LaserScan &msg);
    #endif
    para_info para_;
    Eigen::Vector3f current_eular_;

    std::vector<control_toolbox::Pid> pid_controllers_;

    
    /* function */
    void ros_init(void);
    void pub_msgs(void);
    Eigen::Vector3f inverse_solu(Eigen::Matrix3f rotm, float top_z, Eigen::Matrix3f& xzy,Eigen::Vector3f& xyz_v);
    Eigen::Vector3f rotm2Eul(Eigen::Vector3f vec_in);
    Eigen::Matrix3f normal_vec_rotm(Eigen::Vector3f normal_vec);
    Eigen::Matrix<float, 2, 3> get_rot_element(Eigen::Vector3f vec);
    Eigen::Vector3f get_wall_plat_vec(Eigen::Vector3f laser_dist);
    float max_error(Eigen::Vector3f vector);
    void calculate(void);
    void eul2Rotm(Eigen::Vector3f& euler_ZYX, Eigen::Matrix3f& rotm);

};


#endif