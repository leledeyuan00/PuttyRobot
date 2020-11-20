#include "para_solu/para_solu.h"

para::para(ros::NodeHandle &nh):nh_(nh),laser_state_(0)
{
    ros_init();
}

void para::ros_init()
{
    // pid_controllers_.resize(3);
    pid_controllers_.push_back(control_toolbox::Pid(0.020,0,0.01)); //x
    pid_controllers_.push_back(control_toolbox::Pid(0.020,0,0.01)); //y
    pid_controllers_.push_back(control_toolbox::Pid(0.020,0,0.01)); //y
    // 1
    Joint joint_info1;
    #ifndef SIMULATE
    joint_info1.pub = nh_.advertise<std_msgs::Float64>("/motors/joint1_position_controller/command", 10);
    joint_info1.sub = nh_.subscribe("/laser_topic", 10, &para::laser_callback1,this);
    #else
    joint_info1.pub = nh_.advertise<std_msgs::Float64>("/parabot/joint1_position_controller/command", 10);
    joint_info1.sub = nh_.subscribe("/parabot/laser1", 10, &para::laser_callback1,this);
    #endif
    joint_info1.stat = 0;
    joint_info1.laser = 0;
    joint_info1.cmd = 0;
    para_motor_.push_back(joint_info1);
    // 2
    Joint joint_info2;
    #ifndef SIMULATE
    joint_info2.pub = nh_.advertise<std_msgs::Float64>("/motors/joint2_position_controller/command", 10);
    #else
    joint_info2.pub = nh_.advertise<std_msgs::Float64>("/parabot/joint2_position_controller/command", 10);
    joint_info2.sub = nh_.subscribe("/parabot/laser2", 10, &para::laser_callback2,this);
    #endif
    joint_info2.stat = 0;
    joint_info2.laser = 0;
    joint_info2.cmd = 0;
    para_motor_.push_back(joint_info2);
    // 3
    Joint joint_info3;
    #ifndef SIMULATE
    joint_info3.pub = nh_.advertise<std_msgs::Float64>("/motors/joint3_position_controller/command", 10);
    #else
    joint_info3.pub = nh_.advertise<std_msgs::Float64>("/parabot/joint3_position_controller/command", 10);
    joint_info3.sub = nh_.subscribe("/parabot/laser3", 10, &para::laser_callback3,this);
    #endif
    joint_info3.stat = 0;
    joint_info3.laser = 0;
    joint_info3.cmd = 0;
    para_motor_.push_back(joint_info3);

    para_.rot_matrix[CURRENT] = Eigen::Matrix3f::Identity(3,3);
    para_.rot_matrix[NEXT] = Eigen::Matrix3f::Identity(3,3);

    pub_msgs();
}

#ifndef SIMULATE
void para::laser_callback1(const ppr_msgs::laser &msg)
{
  para_motor_[0].laser = msg.sensor1 / 1000.0;
  para_motor_[1].laser = msg.sensor2 / 1000.0;
  para_motor_[2].laser = msg.sensor3 / 1000.0;
  laser_state_ = msg.state;
}
#else
void para::laser_callback1(const sensor_msgs::LaserScan &msg)
{
    para_motor_[0].laser = msg.ranges[0];
}
void para::laser_callback2(const sensor_msgs::LaserScan &msg)
{
    para_motor_[1].laser = msg.ranges[0];
}
void para::laser_callback3(const sensor_msgs::LaserScan &msg)
{
    para_motor_[2].laser = msg.ranges[0];
}
#endif
void para::pub_msgs(void)
{
    std_msgs::Float64 cmd;
    for (size_t i = 0; i < para_motor_.size(); i++)
    {
        cmd.data = para_motor_[i].cmd;
        para_motor_[i].pub.publish(cmd);
    }
    
}

Eigen::Vector3f para::inverse_solu(Eigen::Matrix3f rotm, float top_z, Eigen::Matrix3f& xzy, Eigen::Vector3f& xyz_v)
{
    /* variable */
    Eigen::Matrix3f top_fram, base_fram, link_matrix;
    Eigen::Vector3f eular;
    float s1,s2,s3,c1,c2,c3,arfa,beta,gama;
    Eigen::Vector3f desired_eula, rot_vectors, target_dist;

    eular = rotm2Eul(rotm.col(2));
    c1 = cos(eular(0));
    c2 = cos(eular(1));
    c3 = cos(eular(2));
    s1 = sin(eular(0));
    s2 = sin(eular(1));
    s3 = sin(eular(2));
   
    top_fram << MOTORPLAT_RADIUS,               -MOTORPLAT_RADIUS/2,          -MOTORPLAT_RADIUS/2,
                               0,        sqrt(3)*MOTORPLAT_RADIUS/2,  -sqrt(3)*MOTORPLAT_RADIUS/2,
                               0,                                 0,                            0;
    base_fram = top_fram;

    rot_vectors << 0.5*MOTORPLAT_RADIUS*(c2*c3+s1*s2*s3-c1*c3), MOTORPLAT_RADIUS*(c2*s3),top_z;
    link_matrix << rot_vectors,rot_vectors,rot_vectors;

    xyz_v = rot_vectors;
    xzy = rotm * top_fram + link_matrix - base_fram;
    
    target_dist << sqrt( pow(xzy(0,0),2) + pow(xzy(1,0),2) + pow(xzy(2,0),2)), 
                   sqrt( pow(xzy(0,1),2) + pow(xzy(1,1),2) + pow(xzy(2,1),2)), 
                   sqrt( pow(xzy(0,2),2) + pow(xzy(1,2),2) + pow(xzy(2,2),2));
    return target_dist;
}

void para::eul2Rotm(Eigen::Vector3f& euler_ZYX, Eigen::Matrix3f& rotm)
{
    //将欧拉角转变为旋转矩阵
    float tan3;
    Eigen::Vector3d c = Eigen::Vector3d(cos(euler_ZYX(0)), cos(euler_ZYX(1)), cos(euler_ZYX(2)));
    Eigen::Vector3d s = Eigen::Vector3d(sin(euler_ZYX(0)), sin(euler_ZYX(1)), sin(euler_ZYX(2)));

    tan3 = -s(0)*s(1)/(c(0)*c(1));
    s(2) = tan3/(sqrt(pow(tan3,2) + 1));
    c(2) = 1/(sqrt(pow(tan3,2) + 1));

    rotm <<                c(1)*c(2),                 -c(1)*s(2),        s(1),
            c(0)*s(2)+c(2)*s(0)*s(1),   c(0)*c(2)-s(0)*s(1)*s(2),  -c(1)*s(0),
            s(0)*s(2)-c(0)*c(2)*s(1),   c(2)*s(0)+c(0)*s(1)*s(2),   c(0)*c(1);
}

Eigen::Vector3f para::rotm2Eul(Eigen::Vector3f vec_in)
{
    Eigen::Vector3f vec;
    float a,b,c;
    float s1,s2,c1,c2,tan1,tan3;

    a = vec_in(0);
    b = vec_in(1);
    c = vec_in(2);

    s2 = a;
    c2 = sqrt(1-pow(s2,2));
    tan1 = -b/c;
    c1 = 1/(sqrt(pow(tan1,2) + 1));
    s1 = tan1/(sqrt(pow(tan1,2) + 1));
    tan3 = -s1*s2/(c1*c2);

    vec << atan(tan1), asin(s2), atan(tan3);
    return vec;
}

/* get Rotation Matrix for this parallel platform */
/* R(x)*R(y) -- R(z)=I */

Eigen::Matrix3f para::normal_vec_rotm(Eigen::Vector3f normal_vec)
{
    Eigen::Matrix<float,2,3> element;
    float s1,s2,s3,c1,c2,c3,arfa,beta,gama;
    float theta;
    float denominator,x,y,z;
    Eigen::Vector3f vector_z,vector_wall,vector;
    Eigen::Matrix3f rotm;

    /* normalize vector */
    element = get_rot_element(normal_vec);
    c1 = element(0,0);
    c2 = element(0,1);
    c3 = 1;
    s1 = element(1,0);
    s2 = element(1,1);
    s3 = 0;

    rotm <<             c2*c3,             -c2*s3,     s2,
               s1*s2*c3+c1*s3,  -s1*s2*s3 + c1*c3, -s1*c2,
            -c1*s2*c3 + s1*s3,   c1*s2*s3 + s1*c3,  c1*c2;
    

    return rotm;
}

Eigen::Matrix<float, 2, 3> para::get_rot_element(Eigen::Vector3f vec)
{
    Eigen::Matrix<float,2,3> element;
    float x,y,z,a,b,c;
    float s1,s2,s3,c1,c2,c3,tan1,tan3;
    float denominator;
    Eigen::Matrix3f rotm;

    x = vec(0);
    y = vec(1);
    z = vec(2);
    denominator = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
    a = x/denominator;
    b = y/denominator;
    c = z/denominator;

    s2 = a;
    c2 = sqrt(1-pow(s2,2));
    tan1 = -b/c;
    c1 = 1/(sqrt(pow(tan1,2) + 1));
    s1 = tan1/(sqrt(pow(tan1,2) + 1));
    tan3 = -s1*s2/(c1*c2);
    s3 = tan3/(sqrt(pow(tan3,2) + 1));
    c3 = 1/(sqrt(pow(tan3,2) + 1));

    element << c1,c2,c3,
               s1,s2,s3;
    return element;
}

/* get Normal Vector by Laser Sensor */
Eigen::Vector3f para::get_wall_plat_vec(Eigen::Vector3f laser_dist)
{
    Eigen::Matrix3f wall_fram;
    Eigen::Vector3f p12,p13,temp_col;

    wall_fram << LASERPLAT_RADIUS,                             -LASERPLAT_RADIUS/2,                 -LASERPLAT_RADIUS/2,
                               0,                       sqrt(3)*LASERPLAT_RADIUS/2,         -sqrt(3)*LASERPLAT_RADIUS/2,
                laser_dist(0) - LASER_DIST_UPPER,   laser_dist(1)-LASER_DIST_UPPER,     laser_dist(2)-LASER_DIST_UPPER;
    p12 = wall_fram.col(1) - wall_fram.col(0);
    p13 = wall_fram.col(2) - wall_fram.col(0);

    temp_col = wall_fram.col(1);
    return p12.cross(p13);
}

float para::max_error(Eigen::Vector3f vector)
{
    float max_temp=vector(0), min_temp=vector(0);
    for (size_t i = 0; i < MOTOR_NUM; i++)
    {
        /* code for loop body */
        if(max_temp<=vector(i)){
            max_temp = vector(i);
        }
    }
    for (size_t i = 0; i < MOTOR_NUM; i++)
    {
        /* code for loop body */
        if(min_temp>=vector(i)){
            min_temp = vector(i);
        }
    }
    return max_temp - min_temp;    
}

void para::control_loop(void)
{
    if(laser_state_ == 0){
    /* variable */
        Eigen::Vector3f wall_plat_vector,wall_eular,pid_error;
        Eigen::Matrix3f rot_matrix,rot_matrix_temp,laser2Base,laser2Base_matrix;
        Eigen::Matrix3f xyz_temp;
        Eigen::Vector3f temp_dist,xyzv_temp;
        ros::Time last_time,curr_time;
        ros::Duration control_duration;
        float error;

        para_.laser_dist << para_motor_[0].laser , para_motor_[1].laser, para_motor_[2].laser;
        wall_plat_vector = get_wall_plat_vec(para_.laser_dist);

        if(max_error(para_.laser_dist)>0.001){
            rot_matrix = normal_vec_rotm(wall_plat_vector);
        }
        else{
            rot_matrix = Eigen::Matrix3f::Identity(3,3);
        }
        wall_eular = rotm2Eul(rot_matrix.col(2));

        curr_time = ros::Time::now();
        control_duration = curr_time - last_time;

        float eular_threash_hold = 0.002;
        // PID
        for (size_t i = 0; i < 3; i++)
        {
            if (fabs(wall_eular(i)) >= eular_threash_hold)
            {
                pid_controllers_[i].setGains(0.02,0,0.001,0,0);
            }
            else
            {
                pid_controllers_[i].setGains(0.0005,0.1,0,0,0);
            }
            pid_error(i) = pid_controllers_[i].computeCommand(wall_eular(i),control_duration);
        }
        

        eul2Rotm(pid_error,rot_matrix_temp);

        para_.rot_matrix[NEXT] = rot_matrix_temp * para_.rot_matrix[CURRENT];

        temp_dist = inverse_solu(para_.rot_matrix[NEXT],PARA_LEN_MEAN,xyz_temp,xyzv_temp);

        for (size_t i = 0; i < para_motor_.size(); i++)
        {
            para_motor_[i].stat = para_motor_[i].cmd;
            error = (temp_dist(i) - MOTOR_LEN_INIT) - para_motor_[i].stat;
            para_motor_[i].cmd = para_motor_[i].stat + error ;
        }


        /* saved status */
        para_.rot_matrix[CURRENT] = para_.rot_matrix[NEXT];
        current_eular_ = rotm2Eul(para_.rot_matrix[CURRENT].col(2));
        current_dist_ = (para_.laser_dist(0) + para_.laser_dist(1) + para_.laser_dist(2))/3;
    }
    pub_msgs();
}


/* public function */

Eigen::Vector3f para::get_eular(void)
{
    return current_eular_;
}

double para::get_dist(void)
{
    return current_dist_;
}

// void para::run()
// {
//     ros::Rate loop_rate(100);
//     while (ros::ok())
//     {
//         /* code for loop body */
//         if(laser_state_ == 0){

//             control_loop();
        
//         }
//         pub_msgs();
    
//         ros::spinOnce();
//         loop_rate.sleep();
//     }
// }

// int main(int argc, char* argv[])
// {
//     ros::init(argc, argv, "paration");
//     ros::NodeHandle nh;

//     para para(nh);
//     para.run();
    
// }