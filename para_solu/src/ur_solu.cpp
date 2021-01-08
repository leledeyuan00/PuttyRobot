#include "para_solu/ur_solu.h"

// const double ur_solu::ready_pos_[6] = {2.3561,-0.5313,-2.0961,1.0409,1.5780,PI};
// const double ur_solu::ready_pos_[6] = {2.3630071714692953, -0.3361866517154537,-2.0500701271791417 , 0.777506008855223, 1.5781260694960224, 0.003906303596900251};

//right position
// const double ur_solu::ready_pos_[6] = {3.1538477188384766, 0.012988474539133321, -1.9388980514208418, 0.3013533012388496, 1.5421069907327087, -0.7460776516580543+PI};
// const double ur_solu::ready_pos_[6] = {3.1356465511484553, 0.017377816650038902, -1.9296997347895655, 0.28723617156258374, 1.5430918346293536, 2.4137534246624694};
const double ur_solu::ready_pos_[6] = {2.9986374856312388, -0.07798873279574714, -1.7902944337701348, 0.280497818416781, 1.5671034304284603, 2.4992552297940165};

// singularity position
// const double ur_solu::ready_pos_[6] = { 1.2517773551034281, -0.8450580025012533,  -2.0178354557875986, 1.3312313417301347, 1.5946120510301638, 1.2499124779019208+ PI};

// const KDL::Rotation ur_solu::R_base =  KDL::Rotation::EulerZYZ(0,PI/2,PI/4);
const KDL::Rotation ur_solu::R_base =  KDL::Rotation::EulerZYZ(0,PI/2,-PI*3/4);
// const KDL::Rotation ur_solu::R_base =   KDL::Rotation::RotZ(0);



ur_solu::ur_solu(ros::NodeHandle &nh):nh_(nh),ur_init_(false)
{
    state_init();
    ros_init();
    para_.reset(new para(nh));
}

//TODO:: Transform UR -> tool  axis x pi

void ur_solu::state_init(void)
{
    /* joint_state initial */
    #ifndef SIMULATE
    mapJoint_.insert(std::pair<std::string, double>("arm_shoulder_pan_joint", 2.359));
    mapJoint_.insert(std::pair<std::string, double>("arm_shoulder_lift_joint", -0.534));
    mapJoint_.insert(std::pair<std::string, double>("arm_elbow_joint", -2.096));
    mapJoint_.insert(std::pair<std::string, double>("arm_wrist_1_joint", 1.05));
    mapJoint_.insert(std::pair<std::string, double>("arm_wrist_2_joint", 1.567));
    mapJoint_.insert(std::pair<std::string, double>("arm_wrist_3_joint", -3.14));
    #else
    mapJoint_.insert(std::pair<std::string, double>("poineer::gtrobot_arm::arm_shoulder_pan_joint", 2.359));
    mapJoint_.insert(std::pair<std::string, double>("poineer::gtrobot_arm::arm_shoulder_lift_joint", -0.534));
    mapJoint_.insert(std::pair<std::string, double>("poineer::gtrobot_arm::arm_elbow_joint", -2.096));
    mapJoint_.insert(std::pair<std::string, double>("poineer::gtrobot_arm::arm_wrist_1_joint", 1.05));
    mapJoint_.insert(std::pair<std::string, double>("poineer::gtrobot_arm::arm_wrist_2_joint", 1.567));
    mapJoint_.insert(std::pair<std::string, double>("poineer::gtrobot_arm::arm_wrist_3_joint", -3.14));
    #endif
    T_ur_forward_ = Eigen::Matrix4d::Identity();

    ppr_cmd_ = Eigen::Vector3d(MID_PARA_LEN,MID_PARA_LEN,MID_PARA_LEN);

    // pid initial
    monitor_rqt_.resize(8);
    for (size_t i = 0; i < mapJoint_.size(); i++)
    {
        control_toolbox::Pid PID(1e-1,0,0,1e-5,-1e-5,true); // just p i d
        cmd_pids_.push_back(PID);
    }
    pid_time_.resize(2);
    pid_time_[1] = ros::Time::now();
    
    // some flags
    srv_start_ = false;
    go_cartisian_flag_ = false;
    go_ready_pos_flag_ = false;
    go_zero_pos_flag_  = false;

    // smc init
    putty_smc_ = PUTTY_INIT;
}

/* ros */

void ur_solu::ros_init(void)
{
    // sub
    #ifndef SIMULATE
    joint_state_sub_ = nh_.subscribe("/joint_states", 10, &ur_solu::JointStateCallback, this);
    #else
    joint_state_sub_ = nh_.subscribe("/gtrobot_arm/joint_states", 10, &ur_solu::JointStateCallback, this);
    #endif

    stm32_sub_ = nh_.subscribe("/stm32_topic",10,&ur_solu::Stm32Callback,this);

    // pub
    #ifndef SIMULATE
    joint_pub_ = nh_.advertise<std_msgs::String>("ur_driver/URScript", 1);
    #else
    joint_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/gtrobot_arm/arm_group_controller/command", 1);
    #endif
    monitor_rqt_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/gtrobot/monitor", 1);


    //srv
    go_ready_srv_ = nh_.advertiseService("/gtrobot_arm/go_ready_pos", &ur_solu::go_ready_pos,this);
    go_zero_srv_ = nh_.advertiseService("/gtrobot_arm/go_zero_pos", &ur_solu::go_zero_pos,this);
    go_cart_srv_ = nh_.advertiseService("/gtrobot_arm/go_cartisian", &ur_solu::go_cartisian,this);
    go_feed_srv_ = nh_.advertiseService("/gtrobot_arm/go_feed",&ur_solu::go_feed,this);
    set_stm_des_pos_ = nh_.serviceClient<ppr_msgs::setStmPosition>("/setStmPosition");
}

// sub call back functions
void ur_solu::JointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    // lock.lock();
    for (int i = 0; i < 6; i++)
    {
        mapJoint_[msg->name[i]] = msg->position[i];
    }

    #ifndef SIMULATE
    joint_state_[0] = mapJoint_["arm_shoulder_pan_joint"];
    joint_state_[1] = mapJoint_["arm_shoulder_lift_joint"];
    joint_state_[2] = mapJoint_["arm_elbow_joint"];
    joint_state_[3] = mapJoint_["arm_wrist_1_joint"];
    joint_state_[4] = mapJoint_["arm_wrist_2_joint"];
    joint_state_[5] = mapJoint_["arm_wrist_3_joint"];
    #else
    joint_state_[0] = mapJoint_["poineer::gtrobot_arm::arm_shoulder_pan_joint"];
    joint_state_[1] = mapJoint_["poineer::gtrobot_arm::arm_shoulder_lift_joint"];
    joint_state_[2] = mapJoint_["poineer::gtrobot_arm::arm_elbow_joint"];
    joint_state_[3] = mapJoint_["poineer::gtrobot_arm::arm_wrist_1_joint"];
    joint_state_[4] = mapJoint_["poineer::gtrobot_arm::arm_wrist_2_joint"];
    joint_state_[5] = mapJoint_["poineer::gtrobot_arm::arm_wrist_3_joint"];
    #endif
    if (!ur_init_)
    {
        memcpy(joint_cmd_,joint_state_,sizeof(joint_state_));
        memcpy(last_start_pos_,joint_state_,sizeof(joint_state_));
        ur_init_ = true;
    }
    
    // ROS_INFO("joint1: [%f], joint2: [%f], joint3: [%f], joint4: [%f], joint5: [%f], joint6:[%f]\r\n",joint_state_[0],joint_state_[1],joint_state_[2],joint_state_[3],joint_state_[4],joint_state_[5]);
    // lock.unlock();
}

void ur_solu::Stm32Callback(const ppr_msgs::encoderConstPtr& msg)
{
    stm_state_.angle = msg->angle;
    stm_state_.error_code = msg->error_code;
    stm_state_.force = msg->force;
    stm_state_.status = msg->status;
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
    start_ppr_pos_ = ppr_distance_;

    ppr_msgs::setStmPosition stmsrv;
    stmsrv.request.data = 0;
    set_stm_des_pos_.call(stmsrv);

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
    start_ppr_pos_ = ppr_distance_;
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
    start_cart_pos_ = current_ur_pose_;
    start_tool_pos_ = current_tool_pose_;

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
    start_cart_pos_ = current_ur_pose_;
    start_tool_pos_ = current_tool_pose_;

    for (size_t i = 0; i < cmd_pids_.size(); i++)
    {
        cmd_pids_[i].reset();
    }
    
    start_task_beta_delta_ = 0;
    start_last_run_dist_ = 0;
    // std::cout << "feed cmd is \r\n" << feed_d_cmd_ << std::endl;
    // record states for variable smc
    switch (putty_smc_)
    {
    case PUTTY_START:
    {
        memcpy(last_start_pos_,joint_state_,sizeof(joint_state_));
        std::stringstream filename;
        std::time_t now = std::time(NULL);
        std::tm *lt = std::localtime(&now);
        filename <<"./lg/" << lt->tm_mon << "-" << lt->tm_mday << "-" << lt->tm_hour << "-" << lt->tm_min << "-" << lt->tm_sec << ".txt";
        outfile_.open(filename.str());
        if(!outfile_) std::cout<<"error"<<std::endl;
        break;
    }
    case PUTTY_RECORD:
    {
        record_count_max_ = req.distance;
        record_count_ = 0;
        std::stringstream filename;
        std::time_t now = std::time(NULL);
        std::tm *lt = std::localtime(&now);
        filename <<"./lg/laser-" << lt->tm_mon << "-" << lt->tm_mday << "-" << lt->tm_hour << "-" << lt->tm_min << "-" << lt->tm_sec << ".txt";
        outfile_.open(filename.str());
        if(!outfile_) std::cout<<"error"<<std::endl;
        break;
    }
    
    default:
        break;
    }
    // for (size_t i = 0; i < cmd_pids_.size(); i++)
    // {
    //     cmd_pids_[i].reset();
    // }
    

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
        if(go_ready_pos_flag_)
        {
            for(size_t i =0; i<6; i++)
            {
                joint_cmd_[i] = (ready_pos_[i] - start_pos_[i]) * (time/finish_time ) + start_pos_[i];
            }
            ppr_cmd_ = (Eigen::Vector3d(MID_PARA_LEN,MID_PARA_LEN,MID_PARA_LEN) - start_ppr_pos_) * (time/finish_time ) + start_ppr_pos_ ; 
        }
        else if(go_zero_pos_flag_)
        {
            for(size_t i =0; i<6; i++)
            {
                joint_cmd_[i] = (0.0 - start_pos_[i]) * (time/finish_time ) + start_pos_[i];       
            }
            // test for forward ppr
            ppr_cmd_ = (Eigen::Vector3d(MIN_PARA_LEN,MAX_PARA_LEN,MAX_PARA_LEN) - start_ppr_pos_) * (time/finish_time ) + start_ppr_pos_ ;
        }    
        else if(go_cartisian_flag_)
        {
            // ur_jacobian_ = Jacobian(joint_state_,KDLR2EigenT(R_base));
            Vector6d cmd_vel;
            Vector6d srv_cart_vel_act;
            Vector9d cmd_all;
            // start_cart_pos_+=srv_cart_vel_;
            // srv_cart_vel_act =  start_cart_pos_ - current_ur_pose_;
            srv_cart_vel_act = srv_cart_vel_ * time*100 + start_tool_pos_ - current_tool_pose_;

            std::cout << " srv_cart_vel_act is \r\n" << srv_cart_vel_act << std::endl;
        #ifndef RDKINEMATIC
            cmd_vel = ur_jacobian_.inverse()* srv_cart_vel_act;
            for (size_t i = 0; i < 6; i++)
            {
                joint_cmd_[i] = cmd_vel(i) + joint_state_[i];
            }
        #else
            cmd_all = WetProjK(rd_jacobian_,srv_cart_vel_act,USE_ALL);
            for (size_t i = 0; i < 6; i++)
            {
                joint_cmd_[i] = cmd_all(i) + joint_state_[i];
            }
            for (size_t i = 0; i < 3; i++)
            {
                ppr_cmd_(i) = cmd_all(i+6) + ppr_distance_(i);
            }      
        #endif
            
        }
        // std::cout << "ppr_cmd is \r\n" << ppr_cmd_ << std::endl;

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
    #ifndef SIMULATE
    double ace,vel,urt;
    std_msgs::String msg;
    std::stringstream ss;
    ace = 0.0; vel = 0.0;  urt = 0.0125;
    ss << "servoj([" << joint_cmd_[0] << "," << joint_cmd_[1] << "," << joint_cmd_[2] << ","    
                    << joint_cmd_[3] << "," << joint_cmd_[4] << "," << joint_cmd_[5] <<"]," 
                    << "a=" << ace << ",v=" << vel << ", t=" << urt << ")";
    msg.data = ss.str();

    // printf("ur joint cmd is [");
    // for(size_t i = 0; i<6 ; i++)
    // {
    //     printf("%f, ",joint_cmd_[i]);
    // }
    // printf("]\r\n");
    joint_pub_.publish(msg);

    #else
    std_msgs::Float64MultiArray ur_cmd_msg;
    std_msgs::Float64MultiArray monitor_rqt_msg;
    for(size_t i = 0; i<6 ; i++)
    {
        ur_cmd_msg.data.push_back(joint_cmd_[i]);
    }
    for (size_t i = 0; i < monitor_rqt_.size(); i++)
    {
        monitor_rqt_msg.data.push_back(monitor_rqt_[i]);
    }
    
    joint_pub_.publish(ur_cmd_msg);
    //monitor
    monitor_rqt_pub_.publish(monitor_rqt_msg);
    #endif    
}


// state update function
void ur_solu::state_update(void)
{
    // time 
    pid_time_[0] = pid_time_[1];
    pid_time_[1] = ros::Time::now();
    task_duration_ = pid_time_[1] - pid_time_[0];
    // ur state first
    T_ur_forward_ = KDLR2EigenT(R_base) * ur_forward(joint_state_,KDLR2EigenT(R_base));
    current_ur_pose_ = EigenT2Pos( T_ur_forward_);
    /* TODO some error in transfer */
    double temp;
    temp = current_ur_pose_(3);
    current_ur_pose_(3) = -current_ur_pose_(4);
    current_ur_pose_(4) = temp;
    // std::cout << "ur pose \r\n" << current_ur_pose_ << std::endl;

    /* quaternion */
    Eigen::Vector4d ur_quaternion = EigenT2Qua(T_ur_forward_);
    // std::cout << "ur quaternion is \r\n" << ur_quaternion << std::endl;


    // parse ppr state second
    ppr_distance_  = para_->get_ppr_dist();

    // std::cout << "ppr_distance_\r\n "<< ppr_distance_ << std::endl; 

    wall_distance_[LAST]    = wall_distance_[CURRENT];
    wall_distance_[CURRENT] = para_->get_wall_dist();
    laser_state_   = para_->get_laser_state();
    ppr_rotation_  = para_->get_ppr_r();

    
    Eigen::Vector3d ppr_v;
    ppr_v << 0,0,ppr_distance_.sum()/3;
    // ppr_v << 0,0,1;
    T_ppr_forward_ << ppr_rotation_ , ppr_v , 0,0,0,1;
    // std::cout << "PPR pose \r\n" <<  EigenT2Pos(T_ur_forward_* KDLR2EigenT(KDL::Rotation::EulerZYX(PI/2,0,PI/2)) * T_ppr_forward_) << std::endl;
    // std::cout << "PPR pose \r\n" <<  EigenT2Pos(T_ur_forward_* T_ppr_forward_) << std::endl;

    T_tool_forward_ = T_ur_forward_ *  T_ppr_forward_;

    last_tool_pose_ = current_tool_pose_;
    current_tool_pose_ = EigenT2Pos(T_tool_forward_);
    /* TODO some error in transfer */
    temp = current_tool_pose_(3);
    current_tool_pose_(3) = -current_tool_pose_(4);
    current_tool_pose_(4) = temp;
    // std::cout << "tool pose \r\n" << current_tool_pose_ << std::endl;

    R_tool_forward_ = EigenT2M6(T_tool_forward_);


    Eigen::Vector4d tool_quaternion = EigenT2Qua(T_tool_forward_);

    ur_jacobian_ = Jacobian(joint_state_,KDLR2EigenT(R_base),(T_ppr_forward_).col(3).head(3));

#ifdef RDKINEMATIC
    Eigen::Matrix<double,6,6> R_ur_forward = EigenT2M6(T_ur_forward_);
    rd_jacobian_ << ur_jacobian_ , R_ur_forward * para_->get_Jacobian();
#endif

    for (size_t i = 0; i < 3; i++)
    {
        monitor_rqt_[i] = current_tool_pose_(i+3);
    }
    for (size_t i = 0; i < 4; i++)
    {
        monitor_rqt_[i+3] = tool_quaternion(i);
    }
    
    
    
    // std::cout << "wall distance is " << wall_distance_ << std::endl;    
}



// main task
void ur_solu::task_handle(void)
{
    if (!task_error_detect())
    {
        /* some error */
        putty_smc_ = PUTTY_ERROR;
    }
    
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
    case PUTTY_DOWN:
    {
        task_down();
        break;
    }
    case PUTTY_LAST_POS:
    {
        task_last_pos();
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
    case PUTTY_RECORD:
    {
        task_record();
        break;
    }
    default:
        break;
    }
    
}

/**
 * @brief: ur cmd generate
*/
void ur_solu::cmdGen(double *cmd, Vector6d v)
{
    double threshold = 0.5;
    for (size_t i = 0; i < 6; i++)
    {
        if (fabs(v(i)) <= threshold )
        {
            cmd[i] = joint_state_[i]  + v(i);
        }
        else{
            cmd[i] = joint_state_[i]  + clamp(v(i),-threshold,threshold);
            ROS_ERROR("joint [%zu] vel is too large %f",i,cmd[i]);
        }
    }
}

Vector6d ur_solu::keepPose(Vector6d startV, Vector6d currentV, uint8_t option)
{
    Vector6d cmd;
    for (size_t i = 0; i < 6; i++)
    {
        if ((option >> i) & 0x01 )
        {
            cmd(i) = 0;
        }
        else
        {
            cmd(i) = startV(i) - currentV(i);
        }
    }
    return cmd;
}

Vector6d ur_solu::comPPr(Vector6d tool_pose, Vector6d ur_pose)
{
    Vector6d v = Vector6d::Zero();
    v[4] = tool_pose[4] - ur_pose[4];
    v[5] = tool_pose[5] - ur_pose[5];
    return v;
}


/* Not Used -- cdy 2020.12.18 */
Vector9d ur_solu::WetProjK(Eigen::Matrix<double,6,9> jacobian, Vector6d vel_c, RDKINE_OPTION option)
{
    Vector9d delta_j;
    Eigen::Matrix<double,9,9> WeightM;
    WeightM << 0.01 * Eigen::Matrix<double,6,6>::Identity(), Eigen::Matrix<double,6,3>::Zero(),
               Eigen::Matrix<double,3,6>::Zero(), 10 * Eigen::Matrix3d::Identity();

    Eigen::Matrix<double,9,6> j_pseudo = jacobian.transpose()*((jacobian * jacobian.transpose()).inverse());

    switch (option)
    {
    case USE_NONE:
    {
        delta_j = j_pseudo * vel_c;
        break;
    }
    case USE_WEIGHTED:
    {
        delta_j = WeightM.inverse()* jacobian.transpose() * ((jacobian * WeightM.inverse() * jacobian.transpose()).inverse()) * vel_c;
        break;
    }
    case USE_PROJECT:
    {
        Vector9d pro_phi = graProjVector(ppr_distance_,MID_PARA_LEN, MAX_PARA_LEN);
        delta_j = j_pseudo * vel_c + (Eigen::Matrix<double,9,9>::Identity() - j_pseudo*jacobian) * pro_phi;
        break;
    }
    case USE_ALL:
    {
        Vector9d pro_phi = graProjVector(ppr_distance_,MID_PARA_LEN, MAX_PARA_LEN);
        delta_j = WeightM.inverse()* jacobian.transpose() * ((jacobian * WeightM.inverse() * jacobian.transpose()).inverse()) * vel_c
                  + (Eigen::Matrix<double,9,9>::Identity() - j_pseudo*jacobian) * pro_phi;
        break;
    }
    
    default:
        break;
    }
    return delta_j;
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
        #ifndef RDKINEMATIC
            para_->control_loop();
        #else
            para_->ppr_update();
        #endif

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

        #ifndef RDKINEMATIC
        #else
            para_->pub_msgs(ppr_cmd_);
        #endif
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