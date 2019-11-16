#include "pararob/solution.h"


Solution::Solution(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    solution_init();
}

void Solution::solution_init()
{
    sensor_sub_ = nh_.subscribe("laser_topic", 10, &Solution::sensorcall,this);
    motor_pub_  = nh_.advertise<pararob::motor>("motor_topic", 10);
    data_monitor_  = nh_.advertise<pararob::motor>("data_monitor", 10);

    sensor_.state   = 255;
}

void Solution::sensorcall(const pararob::sensor& msg)
{
    if(msg.sensor1>0 && msg.sensor2>0 && msg.sensor3>0)
    {
        sensor_.data[0] = (msg.sensor1- 24.37 )/ 1000.0;
        sensor_.data[1] = (msg.sensor2- 24.37 )/ 1000.0;
        sensor_.data[2] = (msg.sensor3- 24.37 )/ 1000.0;
    }
     sensor_.state   = msg.state;
}

void Solution::pub_msg(const MOTOR* motor)
{
    motor_msg_.motor1 = motor->cmd[0];
    motor_msg_.motor2 = motor->cmd[1];
    motor_msg_.motor3 = motor->cmd[2];
    motor_pub_.publish(motor_msg_);
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
    static size_t run_state = 3;
    ros::Rate loop_rate(100);

    for (size_t i = 0; i < MOTOR_NUM; i++)
    {
        /* code for loop body */
        // low_pass_init(&motor_.fil[i],0.5,0);
        low_pass_init(&sensor_.fil[i],0.8,80);
    }
    motor_.cmd[0] = 0.12;
    motor_.cmd[1] = 0.12;
    motor_.cmd[2] = 0.12;
    motor_kine_.para_plat_init(&para_solu_);
    while (ros::ok)
    {
        for (size_t i = 0; i < MOTOR_NUM; i++)
        {
            /* code for loop body */
            low_pass_update(&sensor_.fil[i],sensor_.data[i]);
        }
        
        para_solu_.laser_dist << sensor_.fil[0].out, sensor_.fil[1].out, sensor_.fil[2].out;
        // if(sensor_.state!=0)
        // {

        // }
        // else{
            // run_state--;
            motor_kine_.motor_traj_gen(&para_solu_);   
            motor_.cmd[0] = para_solu_.motor_dist(0) - MOTOR_LEN_INIT;
            motor_.cmd[1] = para_solu_.motor_dist(1) - MOTOR_LEN_INIT;
            motor_.cmd[2] = para_solu_.motor_dist(2) - MOTOR_LEN_INIT;
            ROS_INFO("Motor data 1: %f 2: %f 3: %f",motor_.cmd[0],motor_.cmd[1],motor_.cmd[2]);
        // }

        pub_msg(&motor_);

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
    ros::shutdown();
    return 0;
}