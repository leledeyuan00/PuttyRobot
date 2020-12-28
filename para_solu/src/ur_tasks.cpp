#include "para_solu/ur_solu.h"
/* tasks */
#define TEST_RECORD_PPR_FORWARD


// error detect
bool ur_solu::task_error_detect(void)
{
    if (laser_state_ != 0)
    {
        return false;
    }
    if ( fabs( wall_distance_[CURRENT] + current_ur_pose_(0) - 0.581463) > 0.3) // ready pos x axis
    {
        return false;
    }
    if(wall_distance_[CURRENT] < 0.050)
    {
        ROS_ERROR("It's too close to wall !!!");
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
        #ifndef SIMULATE
        joint_cmd_[i] = joint_state_[i] ;
        #endif
    }
}

// task push
void ur_solu::task_push(void)
{
    // float push_vel = 0.1;
    float wall_distance_d = 0.096; // z of tool 
    float y_distance_d = 0.01;
    ros::Duration finish_duration(2);
    ros::Duration start_duration = ros::Time::now() - start_time_;
    float time;
    if (start_duration <= finish_duration)
    {
        time = start_duration.toSec() / finish_duration.toSec();
    }
    else{
        time = 1;
    }
    
    // float y_distance_d = 0.00;
    // float pid_e = (wall_distance_[CURRENT] - wall_distance_d) * 0.5;
    
    float wall_distance_e = wall_distance_[CURRENT] - wall_distance_d;

    Vector6d wall_distance_v;
    wall_distance_v << 0,0,wall_distance_e,0,0,0;
    // wall_distance_v << wall_distance_e,0,0,0,0,0;
    Eigen::Matrix4d wall_distance_t;

    Vector6d target_pose = (R_tool_forward_ * wall_distance_v ) - start_tool_pos_ + current_tool_pose_ ;

    Vector6d target_pose_vel = target_pose * time - (current_tool_pose_ - start_tool_pos_);

    // composition 
    target_pose_vel += comPPr(current_tool_pose_,current_ur_pose_);
    // X free and beta free and gamma free
    Vector6d cmd_keep = keepPose(start_tool_pos_,current_tool_pose_, X_FREE | BETA_FREE | GAMMA_FREE);
    target_pose_vel+= cmd_keep;
    // pid
    for (size_t i = 0; i < cmd_pids_.size(); i++)
    {
        target_pose_vel[i] = cmd_pids_[i].computeCommand(target_pose_vel[i],task_duration_);
    }
    

    std::cout << "target pose is \r\n" << target_pose_vel << std::endl;

    cmdGen(joint_cmd_,JacoInvSVD(ur_jacobian_) * target_pose_vel);
    
    monitor_rqt_[7] = wall_distance_e;


    if (fabs(wall_distance_[CURRENT] - wall_distance_d) <= 1e-5)
    {
        putty_smc_ = PUTTY_INIT;
        ppr_msgs::setStmPosition stmsrv;
        stmsrv.request.data = 45;
        set_stm_des_pos_.call(stmsrv);
        ROS_INFO("task push is finished!");
    }
}


// task start 
void ur_solu::task_start(void)
{
    float wall_distance_d = 0.096;
    float y_distance_d = 0.8; // actual sign error

    ros::Duration finish_duration(20);
    ros::Duration start_duration = ros::Time::now() - start_time_;
    float time;
    if (start_duration <= finish_duration)
    {
        time = start_duration.toSec() / finish_duration.toSec();
    }
    else{
        time = 1;
    }
    
    double wall_distance_e = wall_distance_[CURRENT] - wall_distance_d;
    double wall_distance_e_dot = wall_distance_[CURRENT] - wall_distance_[LAST];
    double comp_wall_distance = 0;
    
    double current_run_dist = sqrt(pow((current_tool_pose_(0)-start_tool_pos_(0)),2) + pow((current_tool_pose_(1) - start_tool_pos_(1)),2) );
    static double wall_tan_gamma[ALL_STATE];
    wall_tan_gamma[NEXT] = wall_tan_gamma[LAST];
    wall_tan_gamma[LAST] = wall_tan_gamma[CURRENT];
    wall_tan_gamma[CURRENT] = atan2(wall_distance_e_dot,(current_run_dist-start_last_run_dist_));
    
    static int count =0;
    static bool init =false;

    // test record ppr forward instead of tool forward 2020 12 25
    #ifdef TEST_RECORD_PPR_FORWARD

    static Eigen::Matrix4d record_ppr_forward = T_ppr_forward_;
    #endif

    if (count <=10)
    {
        if(fabs(wall_distance_e_dot)>=1e-6 )
        {
            if (fabs(current_run_dist-start_last_run_dist_)>=1e-5)
            {
                // start_task_beta_delta_ += wall_tan_gamma[CURRENT] * 0.01 + (wall_tan_gamma[CURRENT] - wall_tan_gamma[LAST])*0.0 + ((wall_tan_gamma[CURRENT] - wall_tan_gamma[LAST]) - (wall_tan_gamma[LAST] - wall_tan_gamma[NEXT]))*0.0;
                start_task_beta_delta_ += wall_tan_gamma[CURRENT] * 0.01;
            }
            else{
                start_task_beta_delta_ += 0;
            }
            count =0;
        }
        else{
            count++;
        }
    }
    else if(!init){
        // comp_wall_distance = wall_distance_e;
        // init = true;
    }
    // test a const beta;
    start_task_beta_delta_ = -0.07;
    
    start_last_run_dist_ = current_run_dist;

    Vector6d feed;
    feed << (y_distance_d * time) - current_run_dist,0,comp_wall_distance,0,0,0;
    // feed << 0,y_distance_d,0,0,0,0;
    Vector6d beta_deltav;
    beta_deltav << 0,0,0,0,-start_task_beta_delta_,0;
    // EigenT2M6(V2EigenT(beta_deltav)) *
    // ROS_INFO("\r\n current_run_dist is %f \r\n wall_distance_e %f \r\n current_run_dist-last_run_dist %f\r\n start_task_beta_delta_ %f",current_run_dist,wall_distance_e,current_run_dist-start_last_run_dist_,start_task_beta_delta_);
    #ifdef TEST_RECORD_PPR_FORWARD
    Vector6d target_pose_vel = (EigenT2M6(T_ur_forward_ * record_ppr_forward) * EigenT2M6(V2EigenT(beta_deltav)) * feed ) ;
    #else
    Vector6d target_pose_vel = (R_tool_forward_ * EigenT2M6(V2EigenT(beta_deltav)) * feed ) ;
    #endif
    // Vector6d target_pose_vel = ( feed * time) + start_tool_pos_ - current_tool_pose_ ;
    // target_pose_vel(0) =0;

    // composition 
    // Vector6d wall_v;
    // wall_v << 0,0,wall_distance_e,0,0,0;
    // target_pose_vel += R_tool_forward_ * wall_v;

    target_pose_vel += comPPr(current_tool_pose_,current_ur_pose_);
    // X free and beta free and gamma free
    Vector6d cmd_keep = keepPose(start_tool_pos_,current_tool_pose_, X_FREE | Y_FREE | BETA_FREE | GAMMA_FREE);
    target_pose_vel += cmd_keep;
    // pid
    for (size_t i = 0; i < cmd_pids_.size(); i++)
    {
        target_pose_vel[i] = cmd_pids_[i].computeCommand(target_pose_vel[i],task_duration_);
    }
    
    // std::cout << "target pose is \r\n" << target_pose_vel << std::endl;

    cmdGen(joint_cmd_,JacoInvSVD(ur_jacobian_) * target_pose_vel);

    monitor_rqt_[7] = wall_distance_e;

    outfile_ << start_duration.toSec() << " " << wall_distance_e << " " << start_task_beta_delta_ << std::endl;

    if (fabs(y_distance_d - current_run_dist) <= 1e-3) // 1mm
    {
        putty_smc_ = PUTTY_INIT;
        outfile_.close();
        ROS_INFO("task start is finished!");
    }
}

// task back
void ur_solu::task_back(void)
{
    // float push_vel = 0.1;
    float wall_distance_d = 0.25; // has a bug , should to change to tool frame
    float pid_e = (wall_distance_[CURRENT] - wall_distance_d) * 0.01;

    Vector6d wall_distance_e  ,vel_cmdj,vel_cmdj1,vel_cmdj2,vel_cmdj3;
    Vector6d error_pose;

    error_pose = (start_cart_pos_ - current_ur_pose_)*0.01;
    error_pose(0) = 0;
    error_pose(5) = 0;
    
    vel_cmdj1 = ur_jacobian_.inverse() * error_pose;

    wall_distance_e << pid_e,0,0,0,0,0;

    vel_cmdj2 = ur_jacobian_.inverse() * wall_distance_e;

    Vector6d ur_tool_e; // gamma
    ur_tool_e = - ((current_tool_pose_ - current_ur_pose_) * 0.01);
    ur_tool_e(0) = 0; ur_tool_e(1) = 0; ur_tool_e(2) = 0; ur_tool_e(3) = 0; ur_tool_e(4) = 0;

    vel_cmdj3 = ur_jacobian_.inverse() * ur_tool_e;


    vel_cmdj = vel_cmdj1 + vel_cmdj2 + vel_cmdj3;

    for (size_t i = 0; i < 6; i++)
    {
        if (fabs(vel_cmdj(i)) <= 0.5 )
        {
            joint_cmd_[i] = joint_state_[i]  + vel_cmdj(i);
        }
        else{
            ROS_ERROR("joint [%zu] vel is too large %f",i,vel_cmdj(i));
        }
        // joint_cmd_[i] = joint_state_[i]  + vel_cmdj3(i);
    }

    if (fabs(wall_distance_[CURRENT] - wall_distance_d) <= 1e-5)
    {
        putty_smc_ = PUTTY_INIT;
        ppr_msgs::setStmPosition stmsrv;
        stmsrv.request.data = 0;
        set_stm_des_pos_.call(stmsrv);
        ROS_INFO("task push is finished!");
    }
}
void ur_solu::task_feed()
{
    static Vector6d record_temp_start_pos = current_ur_pose_;
    record_temp_start_pos(5) = current_ur_pose_(5);

    if (go_feed_flag_)
    {
        record_temp_start_pos(0) -= feed_d_cmd_(0);
        record_temp_start_pos(1) -= feed_d_cmd_(1);
        record_temp_start_pos(2) -= feed_d_cmd_(2);
        go_feed_flag_ = false;
    }

    float wall_distance_d = 0.2;
    float pid_e = (wall_distance_[CURRENT] - wall_distance_d) * 0.01;
    Vector6d wall_distance_e  ,vel_cmdj1,vel_cmdj2 ,vel_cmdj3;
    wall_distance_e << pid_e,0,0,0,0,0;
    vel_cmdj1 = ur_jacobian_.inverse() * wall_distance_e;

    Vector6d ur_temp_e = (record_temp_start_pos - current_ur_pose_)*0.001;
    Vector6d ur_tool_e;
    ur_tool_e = - ((current_tool_pose_ - current_ur_pose_) * 0.01);
    ur_tool_e(0) = 0; ur_tool_e(1) = 0; ur_tool_e(2) = 0; ur_tool_e(3) = 0; ur_tool_e(4) = 0;

    vel_cmdj2 = ur_jacobian_.inverse() * ur_tool_e;
    vel_cmdj3 = ur_jacobian_.inverse() * ur_temp_e;
    // std::cout << "record pos is \r\n" << ur_temp_e << std::endl; 
    // std::cout << "current pos is \r\n" <<current_ur_pose_ << std::endl;
    // printf("rotation e is %f \r\n", ur_tool_e(5));
    
    for (size_t i = 0; i < 6; i++)
    {
        joint_cmd_[i] = joint_state_[i]  + vel_cmdj1(i) + vel_cmdj2(i) + vel_cmdj3(i);
        // joint_cmd_[i] = joint_state_[i]  + vel_cmdj3(i);
    }
    // printf("wall distance is [%f]\r\n",wall_distance_[CURRENT]);
    // printf("wall distance e is [%f]\r\n", pid_e);
    // printf("vel cmd is [%f %f %f %f %f %f]", vel_cmdj(0), vel_cmdj(1), vel_cmdj(2), vel_cmdj(3), vel_cmdj(4), vel_cmdj(5));
}

void ur_solu::task_down(void)
{
    Eigen::Matrix<double,6,1> cmd_vel;
    Eigen::Matrix<double,6,1> srv_cart_vel_act;
    float vel;
    if (feed_distance_ >0)
    {
        vel = 0.003;
    }
    else{
        vel = -0.003;
    }
    if ( fabs(start_cart_pos_(2) - current_ur_pose_(2)) <= fabs(feed_distance_))
    {
        // srv_cart_vel_act = srv_cart_vel_ *  + start_cart_pos_ - EigenT2Pos(T_ur_forward_);
        srv_cart_vel_act << 0,0,vel,0,0,0;

        cmd_vel = ur_jacobian_.inverse()* srv_cart_vel_act;
    }
    else{
        cmd_vel << 0,0,0,0,0,0;
    }
    
    
    
    for (size_t i = 0; i < 6; i++)
    {
        joint_cmd_[i] = cmd_vel(i) + joint_state_[i];
    }
}