#include "para_solu/ur_solu.h"

const double ur_solu::ready_pos_[6] = {2.3561,-0.5313,-2.0961,1.0409,1.5780,0};

const KDL::Rotation ur_solu::R_base =  KDL::Rotation::RotY(PI/2) * KDL::Rotation::RotZ(PI/4);

ur_solu::ur_solu(ros::NodeHandle &nh):nh_(nh),ur_init_(false)
{
    state_init();
    para_.reset(new para(nh));
    ros_init();
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

    // some flags
    srv_start_ = false;
}

void ur_solu::ros_init(void)
{
    joint_state_sub_ = nh_.subscribe("/gtrobot_arm/joint_states", 10, &ur_solu::JointStateCallback, this);
    joint_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/gtrobot_arm/arm_group_controller/command", 1);
    go_ready_srv_ = nh_.advertiseService("/gtrobot_arm/go_ready_pos", &ur_solu::go_ready_pos,this);
    go_zero_srv_ = nh_.advertiseService("/gtrobot_arm/go_zero_pos", &ur_solu::go_zero_pos,this);
    go_cart_srv_ = nh_.advertiseService("/gtrobot_arm/go_cartisian", &ur_solu::go_cartisian,this);

}

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
    
    res.success = true;
    return true;
}

bool ur_solu::srv_handle(void)
{
    ros::Duration current_duration((ros::Time::now() - start_time_));
    double finish_time = duration_time_.toSec();
    if (current_duration <= duration_time_)
    {
        double time = current_duration.toSec();
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
            else if(go_cartisian_flag_)
            {
                ur_jacobian_ = Jacobian(joint_state_,KDLR2EigenT(R_base));
                Eigen::Matrix<double,6,1> cmd_vel;
                cmd_vel = ur_jacobian_.inverse()* srv_cart_vel_;
                for (size_t i = 0; i < 6; i++)
                {
                    joint_cmd_[i] = cmd_vel(i) + joint_state_[i];
                }
            }
        }
        // ROS_INFO("joint1: [%f], joint2: [%f], joint3: [%f], joint4: [%f], joint5: [%f], joint6:[%f]",joint_cmd_[0],joint_cmd_[1],joint_cmd_[2],joint_cmd_[3],joint_cmd_[4],joint_cmd_[5]);
    }
    else{
        srv_start_ = false; // reset flag
        go_ready_pos_flag_ = false;
        go_zero_pos_flag_ = false;
        go_cartisian_flag_=false;
        ROS_INFO("Already go ready position");
    }
}

void ur_solu::pub_msg(void)
{
    std_msgs::Float64MultiArray ur_cmd_msg;
    for(size_t i = 0; i<6 ; i++)
    {
        ur_cmd_msg.data.push_back(joint_cmd_[i]);
    }
    joint_pub_.publish(ur_cmd_msg);
}


void ur_solu::control_loop(void)
{
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        if(ur_init_)
        {
            para_->control_loop();

            // response srv
            if(srv_start_)
            {
                srv_handle();
            }

            double T[16];
            forward(joint_state_,T);
            Eigen::Matrix4d T_forward = Eigen::Matrix4d::Identity();

            for (int i = 0; i < 4; i ++)
            {
                for (int j = 0; j < 4; j ++)
                {
                    (T_forward)(i,j) = T[i*4+j];
                }
            }

            T_forward = KDLR2EigenT(R_base)*T_forward;

            // ur_jacobian_ = Jacobian(joint_state_,KDLR2EigenT(R_base));
            
            

            // std::cout << "base is : \r\n" << T_forward << std::endl;
            // std::cout << "Jacobian is : \r\n "<< ur_jacobian_<<std::endl;
            
            std::cout << "Current position is [ " << T_forward(0,3) << " "<< T_forward(1,3) << " "<< T_forward(2,3) << " ]" << std::endl;


            // publish msgs
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