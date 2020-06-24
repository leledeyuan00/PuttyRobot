#include "pararob/solution.h"


Solution::Solution(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    state_init();
    solution_init();
}

void Solution::state_init(void)
{
    /* joint_state initial */
    mapJoint.insert(std::pair<std::string, double>("shoulder_pan_joint", 2.359));
    mapJoint.insert(std::pair<std::string, double>("shoulder_lift_joint", -0.534));
    mapJoint.insert(std::pair<std::string, double>("elbow_joint", -2.096));
    mapJoint.insert(std::pair<std::string, double>("wrist_1_joint", 1.05));
    mapJoint.insert(std::pair<std::string, double>("wrist_2_joint", 1.567));
    mapJoint.insert(std::pair<std::string, double>("wrist_3_joint", -3.14));
    
    for (size_t i = 0; i < MOTOR_NUM; i++)
    {
        sensor_.data[i] = 0;
    }
    
}

void Solution::solution_init(void)
{
    sensor_sub_ = nh_.subscribe("laser_topic", 10, &Solution::sensorcall,this);
    joint_state_sub_ = nh_.subscribe("joint_states", 10, &Solution::JointStateCallback, this);
    motor_pub_  = nh_.advertise<motor_driver::motor>("motor_topic", 10);
    data_monitor_  = nh_.advertise<motor_driver::motor>("data_monitor", 10);
    joint_pub_ = nh_.advertise<std_msgs::String>("ur_driver/URScript", 1);
    tool_pub_ = nh_.advertise<pararob::tool_pos>("dayuan_tool", 1);
    sensor_.state   = 255;
}

void Solution::sensorcall(const laser_sensor3::laser& msg)
{
    if(msg.sensor1>0 && msg.sensor2>0 && msg.sensor3>0)
    {
        sensor_.data[0] = (msg.sensor1- 24.37 )/ 1000.0;
        sensor_.data[1] = (msg.sensor2- 24.37 )/ 1000.0;
        sensor_.data[2] = (msg.sensor3- 24.37 )/ 1000.0;
    }
     sensor_.state   = msg.state;
}

void Solution::JointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    // lock.lock();
    for (int i = 0; i < 6; i++)
    {
        mapJoint[msg->name[i]] = msg->position[i];
    }

    joint_state_[0] = mapJoint["shoulder_pan_joint"];
    joint_state_[1] = mapJoint["shoulder_lift_joint"];
    joint_state_[2] = mapJoint["elbow_joint"];
    joint_state_[3] = mapJoint["wrist_1_joint"];
    joint_state_[4] = mapJoint["wrist_2_joint"];
    joint_state_[5] = mapJoint["wrist_3_joint"];
    // ROS_INFO("joint1: [%f], joint2: [%f], joint3: [%f], joint4: [%f], joint5: [%f], joint6:[%f]",joint_state_[0],joint_state_[1],joint_state_[2],joint_state_[3],joint_state_[4],joint_state_[5]);
    // lock.unlock();
}

void Solution::pub_msg(const MOTOR* motor, const UR5_SOLU* ur, int mod)
{
    double ace,vel,urt;
    std_msgs::String msg;
    std::stringstream ss;
    ace = 0.0; vel = 0.0;  urt = 0.0125;

    motor_msg_.motor1 = motor->fil[0].out;
    motor_msg_.motor2 = motor->fil[1].out;
    motor_msg_.motor3 = motor->fil[2].out;

    tool_msg_.x = ur->tool_xyz(0);
    tool_msg_.y = ur->tool_xyz(1);
    tool_msg_.z = ur->tool_xyz(2);
    
    ss << "servoj([" << ur->joint_next[0] << "," << ur->joint_next[1] << "," << ur->joint_next[2] << ","    
                     << ur->joint_next[3] << "," << ur->joint_next[4] << "," << ur->joint_next[5] <<"]," 
                     << "a=" << ace << ",v=" << vel << ", t=" << urt << ")";
    msg.data = ss.str();
    // ROS_INFO("%s", msg.data.c_str());
    /* insuarance */
    if(ur->state!=0 && (mod&0x02)){
        // joint_pub_.publish(msg);
    }
    if(mod&0x01){
        motor_pub_.publish(motor_msg_);
    }
    tool_pub_.publish(tool_msg_);
    // data_monitor_.publish(motor_msg_);
}

/* 算法 */
void Solution::low_pass_init(FILTER* fil, float ratio, float init)
{
    fil->ratio = ratio;
    fil->out = init;
 }

void Solution::low_pass_update(FILTER* fil, float in)
{
    fil->out = in * fil->ratio + fil->out * (1 - fil->ratio);
}


void Solution::run()
{
    ros::Rate loop_rate(100);
    static int loop_num = 0;
    for (size_t i = 0; i < MOTOR_NUM; i++)
    {
        /* filter init loop */
        low_pass_init(&motor_.fil[i],1,PARA_LEN_MEAN-MOTOR_LEN_INIT);
        low_pass_init(&sensor_.fil[i],0.8,LASER_DIST_INIT);
    }
    motor_kine_.robot_state_init(&para_solu_, &ur_);
    motor_.cmd[0] = PARA_LEN_MEAN-MOTOR_LEN_INIT;
    motor_.cmd[1] = PARA_LEN_MEAN-MOTOR_LEN_INIT;
    motor_.cmd[2] = PARA_LEN_MEAN-MOTOR_LEN_INIT;
    // pub_msg(&motor_, &ur_,1);
    //delay()
    /* forever loop */
    while (ros::ok())
    {
        /* filter update */
        for (size_t i = 0; i < MOTOR_NUM; i++){
            low_pass_update(&sensor_.fil[i],sensor_.data[i]);
        }
        /* parse data */
        para_solu_.laser_dist << sensor_.fil[0].out, sensor_.fil[1].out, sensor_.fil[2].out;

        #ifndef SIMULATE
        for (size_t i = 0; i < UR_JOINT_NUM; i++){
            ur_.joint_cur[i] = joint_state_[i];
        }
        #else
        for (size_t i = 0; i < UR_JOINT_NUM; i++){
            ur_.joint_cur[i] = 0;
        }
        #endif
        /* algorithm start */
    
        if(sensor_.state!=0)
        {
            pub_msg(&motor_, &ur_,1);
        }
        else{
            motor_kine_.motor_traj_gen(&para_solu_, &ur_);   
            motor_.cmd[0] = (para_solu_.motor_dist[NEXT](0) - (MOTOR_LEN_INIT));
            motor_.cmd[1] = (para_solu_.motor_dist[NEXT](1) - (MOTOR_LEN_INIT));
            motor_.cmd[2] = (para_solu_.motor_dist[NEXT](2) - (MOTOR_LEN_INIT));                            
            for (size_t i = 0; i < MOTOR_NUM; i++){
                low_pass_update(&motor_.fil[i],motor_.cmd[i]);
            }
            /* algorithm end & pub msg */
            if(loop_num <= 200){
                loop_num++;
                pub_msg(&motor_, &ur_,1);
            }
            else{
                pub_msg(&motor_, &ur_,3);
            }
        }
        ROS_INFO("Motor data 1: %f 2: %f 3: %f",motor_.cmd[0],motor_.cmd[1],motor_.cmd[2]);
        /* loop */
        ros::spinOnce(); 
        loop_rate.sleep(); 
    }
    
}

int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "solution");
    ros::NodeHandle nh;

    Solution solution(&nh);
    solution.run();
    // ros::shutdown();
    return 0;
}