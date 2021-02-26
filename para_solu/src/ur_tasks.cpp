#include "para_solu/ur_solu.h"
/* tasks */
#define TEST_RECORD_PPR_FORWARD


// error detect
bool ur_solu::task_error_detect(void)
{
    if (para_->get_ppr_state() != true)
    {
        ROS_INFO("Para Robot is initializing");
        return false;
    }
    if (laser_error_count_ >= 10)
    {
        // 10 times error continous, there be must some errors!
        ROS_ERROR("Lasers state have some error! May be it is out of the range");
        return false;
    }
    if ( fabs( wall_distance_[CURRENT] + current_ur_pose_(0) - 0.581463) > 0.3) // ready pos x axis
    {
        ROS_ERROR("UR maybe over motion in its Space!");
        return false;
    }
    if(wall_distance_[CURRENT] < 0.050)
    {
        ROS_ERROR("It's too close to wall !!!");
        return false;
    }
    if(stm_state_.error_code != 0)
    {
        ROS_ERROR("It's some error in STM32. Please check it!");
        return false;
    }
    if (para_->get_error_detect())
    {
        ROS_ERROR("It's some error in Para robot!");
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
    float wall_distance_d = 0.094; // z of tool 
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
    

    // std::cout << "target pose is \r\n" << target_pose_vel << std::endl;

    cmdGen(joint_cmd_,JacoInvSVD(ur_jacobian_) * target_pose_vel);
    
    monitor_rqt_[7] = wall_distance_e;


    if (fabs(wall_distance_[CURRENT] - wall_distance_d) <= 1e-5)
    {
        ROS_INFO("task push is finished");
        putty_smc_ = PUTTY_INIT;
        set_stm_pos(30);
    }
}


// task start 
void ur_solu::task_start(void)
{
    float wall_distance_d = 0.094;
    float y_distance_d = 0.6; // actual sign error

    ros::Duration finish_duration(10);
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
    double comp_wall_distance = -(stm_state_.force - start_force_ ) * 0.001;
    
    double current_run_dist = sqrt(pow((current_tool_pose_(0)-start_tool_pos_(0)),2) + pow((current_tool_pose_(1) - start_tool_pos_(1)),2) );
    // static double wall_tan_gamma[ALL_STATE];
    // wall_tan_gamma[NEXT] = wall_tan_gamma[LAST];
    // wall_tan_gamma[LAST] = wall_tan_gamma[CURRENT];
    // wall_tan_gamma[CURRENT] = atan2(wall_distance_e_dot,(current_run_dist-start_last_run_dist_));
    
    static int count =0;
    static bool init =false;

    start_last_run_dist_ = current_run_dist;

    Vector6d feed;
    feed << (y_distance_d * time) - current_run_dist,0,comp_wall_distance,0,0,0;
    // feed << 0,y_distance_d,0,0,0,0;
    double force_beta = -(stm_state_.force - start_force_ ) * 0.1;
    std::cout << " force beta : [" << force_beta << "] comp_wall_distance : ["<< comp_wall_distance << "]"<<  std::endl;
    Vector6d beta_deltav;
    beta_deltav << 0,0,0,0,-(task_record_beta_ + force_beta ),0;
    // EigenT2M6(V2EigenT(beta_deltav)) *
    // ROS_INFO("\r\n current_run_dist is %f \r\n wall_distance_e %f \r\n current_run_dist-last_run_dist %f\r\n start_task_beta_delta_ %f",current_run_dist,wall_distance_e,current_run_dist-start_last_run_dist_,start_task_beta_delta_);
    #ifdef TEST_RECORD_PPR_FORWARD
    
    Vector6d target_pose_vel = (EigenT2M6(T_ur_forward_ * task_record_ppr_forward_) * EigenT2M6(V2EigenT(beta_deltav)) * feed ) ;
    #else
    Vector6d target_pose_vel = (R_tool_forward_ * EigenT2M6(V2EigenT(beta_deltav)) * feed ) ;
    #endif

    target_pose_vel += comPPr(current_tool_pose_,current_ur_pose_);
    // X free and beta free and gamma free
    Vector6d cmd_keep = keepPose(start_tool_pos_,current_tool_pose_, X_FREE | Y_FREE | BETA_FREE | GAMMA_FREE);
    target_pose_vel += cmd_keep;
    // pid
    for (size_t i = 0; i < cmd_pids_.size(); i++)
    {
        target_pose_vel[i] = cmd_pids_[i].computeCommand(target_pose_vel[i],task_duration_);
    }
    
    cmdGen(joint_cmd_,JacoInvSVD(ur_jacobian_) * target_pose_vel);

    monitor_rqt_[7] = wall_distance_e;
    // std::cout << stm_state_.force << std::endl;

    outfile_ << start_duration.toSec() << " " << wall_distance_e << " "<< stm_state_.force 
             << " " << stm_state_.angle
             << " " << current_tool_pose_(0) 
             << " " << current_tool_pose_(1)
             << " " << current_tool_pose_(2)
             << " " << current_tool_pose_(3)
             << " " << current_tool_pose_(4)
             << " " << current_tool_pose_(5)<<std::endl;

    if (fabs(y_distance_d - current_run_dist) <= 1e-3) // 1mm
    {
        putty_smc_ = PUTTY_BACK;
        outfile_.close();
        ROS_INFO("task start is finished!");
        // set_stm_im(5,0.8); // for stable ending
        // set_stm_pos(40);
        memcpy(start_pos_,joint_state_,sizeof(joint_state_));
        start_cart_pos_ = current_ur_pose_;
        start_tool_pos_ = current_tool_pose_;
        start_time_ = ros::Time::now();
    }
}

// task back
void ur_solu::task_back(void)
{

    float wall_distance_d = 0.13; // z of tool 
    ros::Duration finish_duration(2);
    ros::Duration start_duration = ros::Time::now() - start_time_;
    float time,time2;
    if (start_duration <= finish_duration)
    {
        time = start_duration.toSec() / finish_duration.toSec();
    }
    else{
        time = 1;
    }
    
    time2 = time * 4 > 1 ? 1 : time * 4;
    if (time2 < 1)
    {
        set_stm_pos(30 + 20 * time2);
    }
    
    
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
    

    // std::cout << "target pose is \r\n" << target_pose_vel << std::endl;

    cmdGen(joint_cmd_,JacoInvSVD(ur_jacobian_) * target_pose_vel);
    
    monitor_rqt_[7] = wall_distance_e;


    if(fabs(wall_distance_[CURRENT] - wall_distance_d) <= 0.02 && fabs(wall_distance_[LAST] - wall_distance_d) > 0.02)
    {
        ROS_INFO("ML parameter changed in back task");
        set_ML_para();
    }

    if (fabs(wall_distance_[CURRENT] - wall_distance_d) <= 1e-3 && fabs(wall_distance_[LAST] - wall_distance_d) > 1e-3)
    {
        ROS_INFO("stm32 pos changed in back task");
        set_stm_pos(0);
    }
    if(fabs(wall_distance_[CURRENT] - wall_distance_d) <= 1e-3 && fabs(stm_state_.angle) <= 5.0 )
    {
        ROS_INFO("task back is finished!");
        putty_smc_ = PUTTY_CAMERA;
        memcpy(start_pos_,joint_state_,sizeof(joint_state_));
        start_cart_pos_ = current_ur_pose_;
        start_tool_pos_ = current_tool_pose_;
        start_time_ = ros::Time::now();
    }
    // ensure stm pos is go to 0 already
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
        vel = 0.006;
    }
    else{
        vel = -0.006;
    }
    if ( fabs(start_cart_pos_(2) - current_ur_pose_(2)) <= fabs(feed_distance_))
    {
        // srv_cart_vel_act = srv_cart_vel_ *  + start_cart_pos_ - EigenT2Pos(T_ur_forward_);
        srv_cart_vel_act << 0,0,vel,0,0,0;

        cmd_vel = ur_jacobian_.inverse()* srv_cart_vel_act;
    }
    else{
        cmd_vel << 0,0,0,0,0,0;
        putty_smc_ = PUTTY_INIT;
        ROS_INFO("task up/down is finished!");
    }
    
    
    
    for (size_t i = 0; i < 6; i++)
    {
        joint_cmd_[i] = cmd_vel(i) + joint_state_[i];
    }
}

void ur_solu::task_last_pos(void)
{
    set_stm_pos(0);

    ros::Duration duration_time(10);

    ros::Duration current_duration((ros::Time::now() - start_time_));
    double finish_time = duration_time.toSec();
    double time = current_duration.toSec();
    if (current_duration <= duration_time)
    {
        for(size_t i =0; i<6; i++)
        {
            joint_cmd_[i] = (last_start_pos_[i] - start_pos_[i]) * (time/finish_time ) + start_pos_[i];
        }
    }
    else{
        ROS_INFO("Already go last start position");
        putty_smc_ = PUTTY_INIT;
    }
}

void ur_solu::task_calibrate(void)
{
    float wall_distance_d = 0.096;
    float y_distance_d = 0.6; // actual sign error

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
                start_task_beta_delta_ += wall_tan_gamma[CURRENT] * 0.001;
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
        ROS_INFO("Compensation is over, gamma is [%f]",start_task_beta_delta_);
        // comp_wall_distance = wall_distance_e;
        init = true;
    }
    // test a const beta;
    // start_task_beta_delta_ = -0.0108;
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
    std::cout << stm_state_.force << std::endl;

    outfile_ << start_duration.toSec() << " " << wall_distance_e << " " << start_task_beta_delta_ << " "<< stm_state_.force << std::endl;

    if (fabs(y_distance_d - current_run_dist) <= 1e-3) // 1mm
    {
        putty_smc_ = PUTTY_INIT;
        outfile_.close();
        ROS_INFO("task calibrate is finished!");
        task_record_beta_ = start_task_beta_delta_;
        task_record_ppr_forward_ = record_ppr_forward;

        // saved start beta
        std::ofstream outfile_start_beta;
        std::stringstream filename;
        std::time_t now = std::time(NULL);
        std::tm *lt = std::localtime(&now);
        char* home_dir = getenv("HOME");
        filename << home_dir <<"/lg/calibrate/" << "start_beta" << ".txt";
        outfile_start_beta.open(filename.str());
        if(!outfile_start_beta) std::cout<<"error"<<std::endl;
        outfile_start_beta << start_task_beta_delta_ << std::endl;
        outfile_start_beta << record_ppr_forward << std::endl;
        outfile_start_beta.close();
    }
}

void ur_solu::task_camera(void)
{
    ros::Duration go_camera_time(10);

    ros::Duration current_duration((ros::Time::now() - start_time_));
    double finish_time = go_camera_time.toSec();
    double time = current_duration.toSec();
    if (current_duration <= go_camera_time)
    {
        for(size_t i =0; i<6; i++)
        {
            joint_cmd_[i] = (camera_pos_[i] - start_pos_[i]) * (time/finish_time ) + start_pos_[i];
        }
    }
    else{
        ROS_INFO("Already go camera position");
        putty_smc_ = PUTTY_INIT;
        // if (!camera_stated_)
        // {
        //     start_video_record();
        // }
        // else{
            // camera_stated_ = false;
            stop_video_record();
            // start_video_record();
        // }
        
    }
}

void ur_solu::task_record(void)
{
    if (record_count_ < record_count_max_)
    {
        record_count_++;

        ros::Duration start_duration = ros::Time::now() - start_time_;
        Eigen::Vector3d laser,laser_raw;
        laser = para_->get_laser_dist();
        laser_raw = para_->get_laser_raw();
        outfile_ << start_duration.toSec() << " " << laser(0) << " " << laser_raw(0) <<std::endl;
    }else{
        putty_smc_ = PUTTY_INIT;
        outfile_.close();
        ROS_INFO("Data record task is done!");
    }
}