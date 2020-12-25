#ifndef __UR_SOLU_H__
#define __UR_SOLU_H__

/* public include */
//std
#include <stdlib.h>
#include <sstream>
#include <map>
#include <boost/shared_ptr.hpp>
#include <ctime>
#include <fstream>
// ros
#include "ros/ros.h"
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <control_toolbox/pid.h> // pid 
#include <control_toolbox/filters.h> // low_pass_filter


// algorithm
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/src/SVD/JacobiSVD.h>
#include <ur_kinematics/ur_kin.h>
#include <kdl/frames_io.hpp>


/* private include */
#include "para_solu/para_solu.h"
#include "para_solu/go_ready_pos.h"
#include "para_solu/go_zero_pos.h"
#include "para_solu/go_cartisian.h"
#include "para_solu/kinematic.h"
#include "para_solu/go_feed.h"
#include "ppr_msgs/setStmPosition.h"

// #define UR5_PARAMS

using namespace ur_kinematics;


typedef enum
{
    PUTTY_INIT =0,
    PUTTY_PUSH,
    PUTTY_START,
    PUTTY_BACK,
    PUTTY_DOWN,
    PUTTY_ERROR,
    PUTTY_FEED,
    PUTTY_ALL
}PUTTY_TASK_SMC;

typedef enum
{
    USE_NONE = 0,
    USE_WEIGHTED,
    USE_PROJECT,
    USE_ALL,
}RDKINE_OPTION;

#define X_FREE      (1 << 0)
#define Y_FREE      (1 << 1)
#define Z_FREE      (1 << 2)
#define ALPHA_FREE  (1 << 3)
#define BETA_FREE   (1 << 4)
#define GAMMA_FREE  (1 << 5)

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
    ros::Publisher monitor_rqt_pub_;
    ros::Subscriber joint_state_sub_;
    ros::ServiceServer go_ready_srv_;
    ros::ServiceServer go_zero_srv_;
    ros::ServiceServer go_cart_srv_;
    ros::ServiceServer go_feed_srv_;
    ros::ServiceClient set_stm_des_pos_;    

    // states
    std::map<std::string, double> mapJoint_;
    double joint_state_[6];
    static const double ready_pos_[6];
    bool ur_init_;
    Vector6d current_ur_pose_;
    Vector6d current_tool_pose_;
    Vector6d last_tool_pose_;
    Eigen::Vector3d ppr_distance_;
    float wall_distance_[ALL_STATE];
    uint16_t laser_state_;
    Eigen::Matrix3d ppr_rotation_;
    Eigen::Matrix4d T_ppr_forward_;
    Eigen::Matrix4d T_tool_forward_;
    Eigen::Matrix4d T_ur_forward_;
    Eigen::Matrix<double,6,6> R_tool_forward_;
    double start_task_beta_delta_;
    double start_last_run_dist_;
     

    PUTTY_TASK_SMC putty_smc_;

    // monitor data
    std::vector<float> monitor_rqt_;
    

    //service variable
    double start_pos_[6];
    Eigen::Vector3d start_ppr_pos_;
    Vector6d start_cart_pos_;
    Vector6d start_tool_pos_;
    ros::Time start_time_;
    ros::Duration duration_time_;
    bool srv_start_;
    bool go_ready_pos_flag_;
    bool go_zero_pos_flag_;
    bool go_cartisian_flag_;
    bool go_feed_flag_;
    Vector6d srv_cart_vel_;

    Eigen::Vector3d feed_d_cmd_;
    float feed_distance_;

    // cmd variable
    double joint_cmd_[6];
    Eigen::Vector3d ppr_cmd_;

    Eigen::Matrix<double,6,6> ur_jacobian_;

    std::vector<control_toolbox::Pid> cmd_pids_;
    std::vector<ros::Time> pid_time_;
    ros::Duration task_duration_;

#ifdef RDKINEMATIC
    Eigen::Matrix<double,6,9> rd_jacobian_;
#endif
    // kdl
    static const KDL::Rotation R_base;

    // parabot
    std::shared_ptr<para> para_;

    // log
    std::ofstream outfile_;

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
    
    // -- task handles
    void task_handle(void);

    bool task_reset(void);
    bool task_error_detect(void);
    void task_init(void);
    void task_push(void);
    void task_feed(void);
    void task_back(void);
    void task_start(void);
    void task_down(void);

    void error_handle(void);

    void cmdGen(double *cmd, Vector6d v);

    Vector6d keepPose(Vector6d startV, Vector6d currentV, uint8_t option = 1);
    Vector6d comPPr(Vector6d tool_pose, Vector6d ur_pose);

    // algorithm function
    void state_update(void);

    /**
     * @breif: Redundant kinmatics
     * @params: catisan delta X, projection phi, option whther use weighted or projection
     * @return: desired delta joint 9 (ur + ppr)
    */
    Vector9d WetProjK(Eigen::Matrix<double,6,9> jacobian, Vector6d vel_c, RDKINE_OPTION option = USE_ALL);

};
#endif