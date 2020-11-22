#ifndef __UR_SOLU_H__
#define __UR_SOLU_H__

/* public include */
//std
#include <stdlib.h>
#include <sstream>
#include <map>
#include <time.h>
#include <boost/shared_ptr.hpp>
// ros
#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <control_toolbox/pid.h> // pid 
#include <control_toolbox/filters.h> // low_pass_filter


// algorithm
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ur_kinematics/ur_kin.h>
#include <kdl/frames_io.hpp>


/* private include */
#include "para_solu/para_solu.h"
#include "para_solu/go_ready_pos.h"
#include "para_solu/go_zero_pos.h"
#include "para_solu/go_cartisian.h"
#include "para_solu/kinematic.h"
#include "para_solu/go_feed.h"

// #define UR5_PARAMS

using namespace ur_kinematics;
using namespace macmic_kinematic;

class ur_solu
{

public:
    ur_solu(ros::NodeHandle &nh);
    // ~ur_solu();


    void control_loop(void);

private:
    /* data */
    // ros
    ros::NodeHandle nh_;
    ros::Publisher joint_pub_;
    ros::Subscriber joint_state_sub_;
    ros::ServiceServer go_ready_srv_;
    ros::ServiceServer go_zero_srv_;
    ros::ServiceServer go_cart_srv_;
    ros::ServiceServer go_feed_srv_;  

    // states
    std::map<std::string, double> mapJoint_;
    double joint_state_[6];
    static const double ready_pos_[6];
    bool ur_init_;
    Vector6d ur_pose_current_;
    Vector6d tool_pose_current_;
    float ppr_distance_;
    float wall_distance_;
    uint16_t laser_state_;
    Eigen::Matrix3d ppr_rotation_;
    Eigen::Matrix4d T_ppr_forward_;
    Eigen::Matrix4d T_tool_forward_;
    Eigen::Matrix4d T_ur_forward_;
    

    //service variable
    double start_pos_[6];
    Eigen::Matrix<double,6,1> start_cart_pos_;
    ros::Time start_time_;
    ros::Duration duration_time_;
    bool srv_start_;
    bool go_ready_pos_flag_;
    bool go_zero_pos_flag_;
    bool go_cartisian_flag_;
    bool go_feed_flag_;
    Eigen::Matrix<double,6,1> srv_cart_vel_;

    Eigen::Vector3d feed_d_cmd_;
    float feed_distance_;

    // cmd variable
    double joint_cmd_[6];

    Eigen::Matrix<double,6,6> ur_jacobian_;

    // kdl
    static const KDL::Rotation R_base;

    // parabot
    std::shared_ptr<para> para_;

    /* functions */
    void state_init(void);
    void ros_init(void);

    // ros function
    void JointStateCallback(const sensor_msgs::JointStateConstPtr& msg); // topic echo
    bool go_ready_pos(para_solu::go_ready_pos::Request &req, para_solu::go_ready_pos::Response &res); //srv
    bool go_zero_pos(para_solu::go_zero_pos::Request &req, para_solu::go_zero_pos::Response &res); //srv
    bool go_cartisian(para_solu::go_cartisian::Request &req, para_solu::go_cartisian::Response &res); //srv
    bool go_feed(para_solu::go_feed::Request &req, para_solu::go_feed::Response &res); //srv

    void pub_msg(void); // tipic pub
    bool srv_handle(void);

    // algorithm function
    void state_update(void);

};
#endif