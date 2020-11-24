#include "para_solu/ur_solu.h"

// const double ur_solu::ready_pos_[6] = {2.3561,-0.5313,-2.0961,1.0409,1.5780,0};
// const double ur_solu::ready_pos_[6] = {2.3630071714692953, -0.3361866517154537,-2.0500701271791417 , 0.777506008855223, 1.5781260694960224, 0.003906303596900251};
const double ur_solu::ready_pos_[6] = {3.1538477188384766, 0.012988474539133321, -1.9388980514208418, 0.3013533012388496, 1.5421069907327087, -0.7460776516580543};
// singularity position
// const double ur_solu::ready_pos_[6] = { 1.2517773551034281, -0.8450580025012533,  -2.0178354557875986, 1.3312313417301347, 1.5946120510301638, 1.2499124779019208};

const KDL::Rotation ur_solu::R_base =  KDL::Rotation::EulerZYZ(0,PI/2,-PI*3/4);

ur_solu::ur_solu(ros::NodeHandle &nh):nh_(nh),ur_init_(false)
{
    state_init();
    ros_init();
    para_.reset(new para(nh));
}

void ur_solu::state_init(void)
{
    /* joint_state initial */
    mapJoint_.insert(std::pair<std::string, double>("poineer::gtrobot_arm::arm_shoulder_pan_joint", 2.359));
    mapJoint_.insert(std::pair<std::string, double>("poineer::gtrobot_arm::arm_shoulder_lift_joint", -0.534));
    mapJoint_.insert(std::pair<std::string, double>("poineer::gtrobot_arm::arm_elbow_joint", -2.096));
    mapJoint_.insert(std::pair<std::string, double>("poineer::gtrobot_arm::arm_wrist_1_joint", 1.05));
    mapJoint_.insert(std::pair<std::string, double>("poineer::gtrobot_arm::arm_wrist_2_joint", 1.567));
    mapJoint_.insert(std::pair<std::string, double>("poineer::gtrobot_arm::arm_wrist_3_joint", -3.14));

    T_ur_forward_ = Eigen::Matrix4d::Identity();
    // some flags
    srv_start_ = false;
    go_cartisian_flag_ = false;

    // smc init
    putty_smc_ = PUTTY_INIT;
}

/* ros */

void ur_solu::ros_init(void)
{
    // sub
    joint_state_sub_ = nh_.subscribe("/gtrobot_arm/joint_states", 10, &ur_solu::JointStateCallback, this);

    // pub
    joint_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/gtrobot_arm/arm_group_controller/command", 1);
    cmd_vel_monitor_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/gtrobot_arm/arm_group_controller/cmd_vel_monitor", 1);

    //srv
    go_ready_srv_ = nh_.advertiseService("/gtrobot_arm/go_ready_pos", &ur_solu::go_ready_pos,this);
    go_zero_srv_ = nh_.advertiseService("/gtrobot_arm/go_zero_pos", &ur_solu::go_zero_pos,this);
    go_cart_srv_ = nh_.advertiseService("/gtrobot_arm/go_cartisian", &ur_solu::go_cartisian,this);
    go_feed_srv_ = nh_.advertiseService("/gtrobot_arm/go_feed",&ur_solu::go_feed,this);

}

// sub call back functions
void ur_solu::JointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    // lock.lock();
    for (int i = 0; i < 6; i++)
    {
        mapJoint_[msg->name[i]] = msg->position[i];
    }

    joint_state_[0] = mapJoint_["poineer::gtrobot_arm::arm_shoulder_pan_joint"];
    joint_state_[1] = mapJoint_["poineer::gtrobot_arm::arm_shoulder_lift_joint"];
    joint_state_[2] = mapJoint_["poineer::gtrobot_arm::arm_elbow_joint"];
    joint_state_[3] = mapJoint_["poineer::gtrobot_arm::arm_wrist_1_joint"];
    joint_state_[4] = mapJoint_["poineer::gtrobot_arm::arm_wrist_2_joint"];
    joint_state_[5] = mapJoint_["poineer::gtrobot_arm::arm_wrist_3_joint"];
    
    if (!ur_init_)
    {
        memcpy(joint_cmd_,joint_state_,sizeof(joint_state_));
        ur_init_ = true;
    }
    
    // ROS_INFO("joint1: [%f], joint2: [%f], joint3: [%f], joint4: [%f], joint5: [%f], joint6:[%f]",joint_state_[0],joint_state_[1],joint_state_[2],joint_state_[3],joint_state_[4],joint_state_[5]);
    // lock.unlock();
}

// srv call back functions
bool ur_solu::go_ready_pos(para_solu::go_ready_pos::Request &req, 
                           para_solu::go_ready_pos::Response &res)
{
    duration_time_ = ros::Duration(req.duration);
    
    srv_start_ = true;
    go_ready_pos_flag_ = true;
    start_time_ = ros::Time::now();

    memcpy(start_pos_,joint_state_,sizeof(joint_state_));
    // start_pos_ = joint_state_;
    
    res.success = true;
    return true;
}

bool ur_solu::go_zero_pos(para_solu::go_zero_pos::Request &req, 
                          para_solu::go_zero_pos::Response &res)
{
    duration_time_ = ros::Duration(req.duration);
    
    srv_start_ = true;
    go_zero_pos_flag_ = true;
    start_time_ = ros::Time::now();

    memcpy(start_pos_,joint_state_,sizeof(joint_state_));
    // start_pos_ = joint_state_;
    
    res.success = true;
    return true;
}

bool ur_solu::go_cartisian(para_solu::go_cartisian::Request &req, 
                           para_solu::go_cartisian::Response &res)
{
    duration_time_ = ros::Duration(req.duration);
    for (size_t i = 0; i < 6; i++)
    {
        srv_cart_vel_(i) = req.cmd_vel[i];
    }
    
    srv_start_ = true;
    go_cartisian_flag_ = true;
    start_time_ = ros::Time::now();

    memcpy(start_pos_,joint_state_,sizeof(joint_state_));
    start_cart_pos_ = EigenT2Pos(T_ur_forward_);

    ROS_INFO("current record pos is [%f, %f, %f, %f, %f, %f",start_cart_pos_ [0],start_cart_pos_ [1],start_cart_pos_ [2],start_cart_pos_ [3],start_cart_pos_ [4],start_cart_pos_ [5]);    
    res.success = true;
    return true;
}

bool ur_solu::go_feed(para_solu::go_feed::Request &req, 
                      para_solu::go_feed::Response &res)
{
    Eigen::Vector3d feed_temp;
    feed_distance_ = req.distance;
    putty_smc_ = (PUTTY_TASK_SMC)req.cmd;
    feed_temp << req.x_vel, 0, 0;
    feed_d_cmd_ = EigenT2EigenR(T_ur_forward_) * feed_temp;

    go_feed_flag_ = true;
    start_time_ = ros::Time::now();

    memcpy(start_pos_,joint_state_,sizeof(joint_state_));
    start_cart_pos_ = ur_pose_current_;
    start_tool_pos_ = tool_pose_current_;

    // std::cout << "feed cmd is \r\n" << feed_d_cmd_ << std::endl;

    res.success = true;
    return true;
}

bool ur_solu::srv_handle(void)
{
    ros::Duration current_duration((ros::Time::now() - start_time_));
    double finish_time = duration_time_.toSec();
    double time = current_duration.toSec();
    if (current_duration <= duration_time_)
    {
        for(size_t i =0; i<6; i++)
        {
            if(go_ready_pos_flag_)
            {
                joint_cmd_[i] = (ready_pos_[i] - start_pos_[i]) * (time/finish_time ) + start_pos_[i];
            }
            else if(go_zero_pos_flag_)
            {
                joint_cmd_[i] = (0.0 - start_pos_[i]) * (time/finish_time ) + start_pos_[i];
            }
            
        }
        if(go_cartisian_flag_)
        {
            ur_jacobian_ = Jacobian(joint_state_,KDLR2EigenT(R_base));
            Eigen::Matrix<double,6,1> cmd_vel;
            Eigen::Matrix<double,6,1> srv_cart_vel_act;
            srv_cart_vel_act = srv_cart_vel_ * time + start_cart_pos_ - EigenT2Pos(T_ur_forward_);

            cmd_vel = ur_jacobian_.inverse()* srv_cart_vel_;
            for (size_t i = 0; i < 6; i++)
            {
                joint_cmd_[i] = cmd_vel(i) + joint_state_[i];
            }
        }
    }
    else{
        srv_start_ = false; // reset flag
        go_ready_pos_flag_ = false;
        go_zero_pos_flag_ = false;
        go_cartisian_flag_=false;
        ROS_INFO("Already go ready position");
    }
    putty_smc_ = PUTTY_INIT;
}

// publish function
void ur_solu::pub_msg(void)
{
    std_msgs::Float64MultiArray ur_cmd_msg;
    std_msgs::Float64MultiArray ur_cmd_vel_monitor_msg;
    for(size_t i = 0; i<6 ; i++)
    {
        ur_cmd_msg.data.push_back(joint_cmd_[i]);
        ur_cmd_vel_monitor_msg.data.push_back(cmd_vel_monitor_[i]);
    }
    joint_pub_.publish(ur_cmd_msg);
    
    //monitor
    cmd_vel_monitor_pub_.publish(ur_cmd_vel_monitor_msg);
}


// state update function
void ur_solu::state_update(void)
{
    // ur state first
    T_ur_forward_ = ur_forward(joint_state_,KDLR2EigenT(R_base));
    ur_jacobian_ = Jacobian(joint_state_,KDLR2EigenT(R_base));
    ur_pose_current_ = EigenT2Pos(T_ur_forward_);

    // parse ppr state second
    para_->control_loop();
    ppr_distance_  = para_->get_ppr_dist();
    wall_distance_ = para_->get_wall_dist();
    laser_state_   = para_->get_laser_state();
    ppr_rotation_  = para_->get_ppr_r();
    
    Eigen::Vector3d ppr_v;
    ppr_v << 0,0,ppr_distance_;
    T_ppr_forward_ << ppr_rotation_ , ppr_v , 0,0,0,1;

    T_tool_forward_ = T_ur_forward_ * T_ppr_forward_;
    tool_pose_current_ = EigenT2Pos(T_tool_forward_);

    // std::cout << "ur pos current \r\n" << ur_pose_current_ << std::endl;
    
    // std::cout<< " tool ur pose error \r\n" << tool_pose_current_ - ur_pose_current_ << std::endl;
}

/* tasks */

// error detect
bool ur_solu::task_error_detect(void)
{
    if (laser_state_ != 0)
    {
        return false;
    }
    if ( fabs( wall_distance_ + ur_pose_current_(0) - 0.581463) > 0.3) // ready pos x axis
    {
        return false;
    }
    if(wall_distance_ < 0.050)
    {
        return false;
    }
    return true;
    
}

void ur_solu::error_handle(void)
{
    // error handle TODO
}

// task init
void ur_solu::task_init()
{
    // stop UR
    for (size_t i = 0; i < 6; i++)
    {
        joint_cmd_[i] = joint_state_[i] ;
    }
}

// task push
void ur_solu::task_push(void)
{
    // float push_vel = 0.1;
    float wall_distance_d = 0.2; // has a bug , should to change to tool frame
    float y_distance_d = 0.05;
    float pid_e = (wall_distance_ - wall_distance_d) * 0.01;

    Vector6d wall_distance_e  ,vel_cmdj,vel_cmdj1,vel_cmdj2,vel_cmdj3;
    Vector6d error_pose;

    error_pose = (start_cart_pos_ - ur_pose_current_)*0.01;
    error_pose(0) = 0;
    error_pose(5) = 0;

    if (wall_distance_ - wall_distance_d < 0.05)
    {
        error_pose(1) = (y_distance_d + start_cart_pos_(1) - ur_pose_current_(1))*0.01;
    }
    
    vel_cmdj1 = ur_jacobian_.inverse() * error_pose;

    wall_distance_e << pid_e,0,0,0,0,0;

    vel_cmdj2 = ur_jacobian_.inverse() * wall_distance_e;

    Vector6d ur_tool_e; // gamma
    ur_tool_e = - ((tool_pose_current_ - ur_pose_current_) * 0.01);
    ur_tool_e(0) = 0; ur_tool_e(1) = 0; ur_tool_e(2) = 0; ur_tool_e(3) = 0; ur_tool_e(4) = 0;

    vel_cmdj3 = ur_jacobian_.inverse() * ur_tool_e;


    vel_cmdj = vel_cmdj1 + vel_cmdj2 + vel_cmdj3;

    for (size_t i = 0; i < 6; i++)
    {
        cmd_vel_monitor_[i] = vel_cmdj(i);
        if (fabs(vel_cmdj(i)) <= 0.5 )
        {
            joint_cmd_[i] = joint_state_[i]  + vel_cmdj(i);
        }
        else{
            ROS_ERROR("joint [%zu] vel is too large %f",i,vel_cmdj(i));
        }
        
        // joint_cmd_[i] = joint_state_[i]  + vel_cmdj3(i);
    }
}


// task start 
void ur_solu::task_start(void)
{
    float wall_distance_d = 0.2;
    float y_distance_d = 0.8;
    float finish_time = 20;
    float pid_e = (wall_distance_ - wall_distance_d) * 0.01; // keep a fixed distance
    Vector6d wall_distance_e  ,vel_cmdj,vel_cmdj1,vel_cmdj2,vel_cmdj3;
    Vector6d error_pose;

    /* cmdj1 keep ur a right pose */
    error_pose = (start_cart_pos_ - ur_pose_current_)*0.01;
    error_pose(0) = 0; // open x and y and gamma to adjust ppr
    error_pose(1) = 0;
    error_pose(2) = 0;
    error_pose(5) = 0;

    vel_cmdj1 = ur_jacobian_.inverse() * error_pose;

    /* cmdj2 keep ur and wall distance */
    wall_distance_e << pid_e,0,0,0,0,0;

    vel_cmdj2 = ur_jacobian_.inverse() * wall_distance_e;

    /* cmdj3 keep ur trace ppr rotation to keep ppr has a Identity rot matrix */
    Vector6d ur_tool_e; // gamma
    ur_tool_e = - ((tool_pose_current_ - ur_pose_current_) * 0.01);
    ur_tool_e(0) = 0; ur_tool_e(1) = 0; ur_tool_e(2) = 0; ur_tool_e(3) = 0; ur_tool_e(4) = 0;

    vel_cmdj3 = ur_jacobian_.inverse() * ur_tool_e;

    /* cmdj4 start putty */
    // time
    ros::Duration current_duration((ros::Time::now() - start_time_));
    float start_e, time_e;
    Vector6d ur_start_e,vel_cmdj4;
    Eigen::Vector3d tool_d2base;
    time_e = (current_duration.toSec()/finish_time) <= 1? (current_duration.toSec()/finish_time):1;
    tool_d2base = EigenT2EigenR(T_ur_forward_) * Eigen::Vector3d(y_distance_d*time_e,0,0);
    for (size_t i = 0; i < 3; i++)
    {
        ur_start_e(i) = (start_cart_pos_(i) - tool_d2base(i)  - ur_pose_current_(i));
        ur_start_e(i+3) =0;
    }
    std::cout<< "ur start feed e \r\n" << ur_start_e << std::endl;
    // vel_cmdj4 << 0,0,0,0,0,0;
    vel_cmdj4 = ur_jacobian_.inverse()*ur_start_e;

    vel_cmdj = vel_cmdj1 + vel_cmdj2 + vel_cmdj3 + vel_cmdj4;

    for (size_t i = 0; i < 6; i++)
    {
        cmd_vel_monitor_[i] = vel_cmdj(i);
        if (fabs(vel_cmdj(i)) <= 0.5 )
        {
            joint_cmd_[i] = joint_state_[i]  + vel_cmdj(i);
        }
        else{
            ROS_ERROR("joint [%zu] vel is too large %f",i,vel_cmdj(i));
        }
        // joint_cmd_[i] = joint_state_[i]  + vel_cmdj3(i);
    }
} 

// task back
void ur_solu::task_back(void)
{
    // float push_vel = 0.1;
    float wall_distance_d = 0.3; // has a bug , should to change to tool frame
    float pid_e = (wall_distance_ - wall_distance_d) * 0.01;

    Vector6d wall_distance_e  ,vel_cmdj,vel_cmdj1,vel_cmdj2,vel_cmdj3;
    Vector6d error_pose;

    error_pose = (start_cart_pos_ - ur_pose_current_)*0.01;
    error_pose(0) = 0;
    error_pose(5) = 0;
    
    vel_cmdj1 = ur_jacobian_.inverse() * error_pose;

    wall_distance_e << pid_e,0,0,0,0,0;

    vel_cmdj2 = ur_jacobian_.inverse() * wall_distance_e;

    Vector6d ur_tool_e; // gamma
    ur_tool_e = - ((tool_pose_current_ - ur_pose_current_) * 0.01);
    ur_tool_e(0) = 0; ur_tool_e(1) = 0; ur_tool_e(2) = 0; ur_tool_e(3) = 0; ur_tool_e(4) = 0;

    vel_cmdj3 = ur_jacobian_.inverse() * ur_tool_e;


    vel_cmdj = vel_cmdj1 + vel_cmdj2 + vel_cmdj3;

    for (size_t i = 0; i < 6; i++)
    {
        cmd_vel_monitor_[i] = vel_cmdj(i);
        if (fabs(vel_cmdj(i)) <= 0.5 )
        {
            joint_cmd_[i] = joint_state_[i]  + vel_cmdj(i);
        }
        else{
            ROS_ERROR("joint [%zu] vel is too large %f",i,vel_cmdj(i));
        }
        
        // joint_cmd_[i] = joint_state_[i]  + vel_cmdj3(i);
    }
}
void ur_solu::task_feed()
{
    static Vector6d record_temp_start_pos = ur_pose_current_;
    record_temp_start_pos(5) = ur_pose_current_(5);

    if (go_feed_flag_)
    {
        record_temp_start_pos(0) -= feed_d_cmd_(0);
        record_temp_start_pos(1) -= feed_d_cmd_(1);
        record_temp_start_pos(2) -= feed_d_cmd_(2);
        go_feed_flag_ = false;
    }

    float wall_distance_d = 0.2;
    float pid_e = (wall_distance_ - wall_distance_d) * 0.01;
    Vector6d wall_distance_e  ,vel_cmdj1,vel_cmdj2 ,vel_cmdj3;
    wall_distance_e << pid_e,0,0,0,0,0;
    vel_cmdj1 = ur_jacobian_.inverse() * wall_distance_e;

    Vector6d ur_temp_e = (record_temp_start_pos - ur_pose_current_)*0.001;
    Vector6d ur_tool_e;
    ur_tool_e = - ((tool_pose_current_ - ur_pose_current_) * 0.01);
    ur_tool_e(0) = 0; ur_tool_e(1) = 0; ur_tool_e(2) = 0; ur_tool_e(3) = 0; ur_tool_e(4) = 0;

    vel_cmdj2 = ur_jacobian_.inverse() * ur_tool_e;
    vel_cmdj3 = ur_jacobian_.inverse() * ur_temp_e;
    // std::cout << "record pos is \r\n" << ur_temp_e << std::endl; 
    // std::cout << "current pos is \r\n" <<ur_pose_current_ << std::endl;
    // printf("rotation e is %f \r\n", ur_tool_e(5));
    
    for (size_t i = 0; i < 6; i++)
    {
        cmd_vel_monitor_[i] = vel_cmdj1(i) + vel_cmdj2(i) + vel_cmdj3(i);
        joint_cmd_[i] = joint_state_[i]  + vel_cmdj1(i) + vel_cmdj2(i) + vel_cmdj3(i);
        // joint_cmd_[i] = joint_state_[i]  + vel_cmdj3(i);
    }
    // printf("wall distance is [%f]\r\n",wall_distance_);
    // printf("wall distance e is [%f]\r\n", pid_e);
    // printf("vel cmd is [%f %f %f %f %f %f]", vel_cmdj(0), vel_cmdj(1), vel_cmdj(2), vel_cmdj(3), vel_cmdj(4), vel_cmdj(5));
}

// main task
void ur_solu::task_handle(void)
{
    if (!task_error_detect())
    {
        /* some error */
        // putty_smc_ = PUTTY_ERROR;
    }
    else
    {
        switch (putty_smc_)
        {
        case PUTTY_INIT:
        {
            task_init();
            break;
        }
        case PUTTY_PUSH:
        {
            task_push();
            break;
        }
        case PUTTY_START:
        {
            task_start();
            break;
        }
        case PUTTY_BACK:
        {
            task_back();
            break;
        }
        case PUTTY_FEED:
        {
            task_feed();
            break;
        }
        case PUTTY_ERROR:
        {
            error_handle();
            break;
        }
        default:
            break;
        }
    }    
}

void ur_solu::control_loop(void)
{
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        // ur initializing
        if(ur_init_)
        {
            // update state
            state_update(); 
            // response srv
            if(srv_start_)
            {
                srv_handle();
            }
            else{
                task_handle();   
            }
            pub_msg();
        }
        // spin
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ur_solu");
    ros::NodeHandle nh;

    ur_solu ur_solu_(nh);
    ur_solu_.control_loop();

    ROS_INFO("END PROGRAME");

    return 0;
}