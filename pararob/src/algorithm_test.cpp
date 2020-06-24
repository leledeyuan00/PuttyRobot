#include "pararob/algorithm_test.h"

// #define FIRST_DEBUG 1
#define SECOND_DEBUG 1
// #define THIRD_DEBUG 1
// #define FORTH_DEBUG 1

Test1::Test1(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    state_init();
    test_init();
}

void Test1::state_init()
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
        motor_.cmd[i] = 0.0125;
    }
}

void Test1::test_init()
{
    motor_pub_ = nh_.advertise<motor_driver::motor>("motor_topic",10);
    sensor_sub_ = nh_.subscribe("laser_topic", 10, &Test1::sensorcall,this);
    sensor_.state   = 255;
}

void Test1::sensorcall(const laser_sensor3::laser& msg)
{
    if(msg.sensor1>0 && msg.sensor2>0 && msg.sensor3>0)
    {
        sensor_.data[0] = (msg.sensor1- 24.37 )/ 1000.0;
        sensor_.data[1] = (msg.sensor2- 24.37 )/ 1000.0;
        sensor_.data[2] = (msg.sensor3- 24.37 )/ 1000.0;
    }
     sensor_.state   = msg.state;
}

void Test1::pub_msg(const MOTOR* motor)
{
    motor_msg_.motor1 = motor->cmd[0];
    motor_msg_.motor2 = motor->cmd[1];
    motor_msg_.motor3 = motor->cmd[2];

    motor_pub_.publish(motor_msg_);
}

void Test1::run()
{
    ros::Rate loop_rate(100);
    /* Debug First variables */
    static int dir;
    static float dist;
    dir = 0;
    dist = 0.015;

    MOTOR_KINE motor_kine_;

    while (ros::ok())
    {
        /* code for loop body */
        para_solu_.laser_dist << sensor_.data[0], sensor_.data[1], sensor_.data[2];
        for (size_t i = 0; i < UR_JOINT_NUM; i++){
            ur_.joint_cur[i] = 0;
        }

        for (size_t i = 0; i < MOTOR_NUM; i++)
        {
            motor_.current[i] = motor_.cmd[i];
        }
        

        /* Debug First for motor loop */
        #if FIRST_DEBUG
        if (dist>= 0.025)
        {
            dir = 1;
        }
        else if(dist <= 0.015)
        {
            dir = 0;
        }
        if (dir != 1)
        {
            dist += 0.0001;
        }
        else{
            dist -=0.0001;
        }      

        for (size_t i = 0; i < MOTOR_NUM; i++)
        {
            /* code for loop body */
            // motor_.cmd[i] = dist;
            motor_.cmd[i] = 0.015;
        }
        pub_msg(&motor_);
        ROS_INFO("Motor data 1: %f 2: %f 3: %f",motor_.cmd[0],motor_.cmd[1],motor_.cmd[2]);
        #elif SECOND_DEBUG
        if(sensor_.state!=0)
        {
            pub_msg(&motor_);
        }
        else{
            Eigen::Vector3f wall_plat_vector,wall_eular;
            Eigen::Matrix3f rot_matrix;
            wall_plat_vector = motor_kine_.get_wall_plat_vec(para_solu_.laser_dist);
            ROS_INFO("Wall platform vector is : [%f, %f, %f]", wall_plat_vector(0), wall_plat_vector(1),wall_plat_vector(2));
            if(motor_kine_.max_error(para_solu_.laser_dist)>0.001){
                rot_matrix = motor_kine_.normal_vec_rotm(wall_plat_vector);
            }
            else{
                rot_matrix = Eigen::Matrix3f::Identity(3,3);
            }  
            wall_eular = motor_kine_.rotm2Eul(rot_matrix.col(2));
            ROS_INFO("Wall Eular is : [%f, %f, %f]", wall_eular(0), wall_eular(1),wall_eular(2));
        }
        #elif THIRD_DEBUG

        #elif FORTH_DEBUG
        #endif
        
        /* loop */
        ros::spinOnce(); 
        loop_rate.sleep(); 
    }   
}




int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "algorithm_test");
    ros::NodeHandle nh;

    Test1 algorithm(&nh);
    algorithm.run();

    return 0;
}