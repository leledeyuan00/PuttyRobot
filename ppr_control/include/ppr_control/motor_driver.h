#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_




#include <ros/ros.h>
#include <serial/serial.h>
#include <urdf/model.h>


/* for hardware interface */

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <controller_manager/controller_manager.h>
#include <boost/shared_ptr.hpp>
#include <limits>
/* private include */
#include "ppr_msgs/motor.h"

#define MOTOR_STROKE 27 //mm
#define MOTOR_RES 4095 //16bits

struct JOINT_INFO
{
    double pos;
    double vel;
    double eft;
};

struct JOINT
{
    JOINT_INFO state;
    JOINT_INFO cmd;
};


class Motor_driver : public hardware_interface::RobotHW
{
public:
    Motor_driver(ros::NodeHandle* nodehandle);
    // virtual ~Motor_driver();
    ~Motor_driver(){};
    void update(const ros::TimerEvent& e);
    void read();
    void write(ros::Duration elapsed_time);

    /* 循环发送数据 */
    void run();

private:
    /* const variables */
    size_t MOTOR_NUM;
    
    /* data */
    ros::NodeHandle nh_;
    ros::Subscriber motor_sub_;

    ros::Timer my_control_loop_;
    ros::Duration elapsed_time_;
    double loop_hz_;
    boost::shared_ptr<controller_manager::ControllerManager>controller_manager_;

    serial::Serial motor_ser_;

    
    std::vector<unsigned short> motor_msg_;
    std::vector<unsigned short> motor_msg2_;


    /* hardware interface */
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    hardware_interface::JointStateInterface jnt_st_int_;
    hardware_interface::PositionJointInterface pos_jnt_int_;
    hardware_interface::VelocityJointInterface vel_jnt_int_;
    hardware_interface::EffortJointInterface eft_jnt_int_;

    joint_limits_interface::JointLimits limits;
    joint_limits_interface::PositionJointSaturationInterface pos_jnt_limit_int_;
    joint_limits_interface::VelocityJointSaturationInterface vel_jnt_limit_int_;
    joint_limits_interface::EffortJointSaturationInterface eft_jnt_limit_int_;

    joint_limits_interface::PositionJointSoftLimitsInterface pos_jnt_soft_limits_;
    joint_limits_interface::VelocityJointSoftLimitsInterface vel_jnt_soft_limits_;
    joint_limits_interface::EffortJointSoftLimitsInterface eft_jnt_soft_limits_;

    boost::shared_ptr<urdf::Model> urdf_model_;

    std::vector<double> joint_position_lower_limits_;
    std::vector<double> joint_position_upper_limits_;
    std::vector<double> joint_velocity_limits_;
    std::vector<double> joint_effort_limits_;

    std::vector<JOINT> jnt_;
    // Modes
    std::string name_;
    bool use_rosparam_joint_limits_;
    bool use_soft_limits_if_available_;

    
    /* 初始化 */
    void motor_driver_init();
    int motor_ser_init(serial::Serial* ser, const int port);
    /* 校验 */
    unsigned char motor_check_sum(unsigned char* data); 
    /* 发数据 */
    void motor_write(std::vector<short unsigned int>& msg);
    
    void drivercallback(const ppr_msgs::motor& msg);

    void registerJointLimits(const hardware_interface::JointHandle &joint_handle_position,
                             const hardware_interface::JointHandle &joint_handle_velocity,
                             const hardware_interface::JointHandle &joint_handle_effort,
                             std::size_t joint_id);

    void loadURDF(ros::NodeHandle &nh, std::string param_name);
    std::vector<std::string> joint_names_;
    // std::vector<std::string> joint_names_{"motor1","motor2","motor3"};
};

#endif // !MOTOR_DRIVER_
