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
#include "motor_driver/motor.h"
#include "laser_sensor3/laser.h"
#include "pararob/kinematic.h"
#include "pararob/parameter.h"
#include "pararob/tool_pos.h"

class Test1
{
public:
    Test1(ros::NodeHandle* nodehandle);
    // virtual ~Test1();

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
    laser_sensor3::laser sensor_msg_;

    struct MOTOR motor_;
    motor_driver::motor motor_msg_;

    struct PARA_SOLU para_solu_;

    // MOTOR_KINE motor_kine_;

    pararob::tool_pos tool_msg_;

    struct UR5_SOLU ur_;
    double joint_state_[6];
    
    /* function */
    /* 初始化 */
    void test_init(void);
    void state_init(void);
    /* 接口函数 */
    void sensorcall(const laser_sensor3::laser& msg);
    void pub_msg(const MOTOR* motor);
    void JointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
    /* 算法 */
    // void motor_traj_generator(SENSOR* sensor, MOTOR* motor);
    void Motor_traj_gen(SENSOR* sensor, MOTOR* motor);
    void low_pass_init(FILTER* fil, float ratio, float init);
    void low_pass_update(FILTER* fil, float in);
};