#include "para_solu/para_solu.h"

const std::vector<double> para::filter_coefficient_ = {0.0015,0.0075,0.0217,0.0467,0.0809,0.1175,0.1459,0.1567,0.1459,0.1175,0.0809,0.0467,0.0217,0.0075,0.0015};

para::para(ros::NodeHandle &nh):nh_(nh),laser_state_(0),current_laser_state_(0)
{
    state_init();
    ros_init();
}

void para::state_init(void)
{
    top_fram_ << MOTORPLAT_RADIUS,               -MOTORPLAT_RADIUS/2,          -MOTORPLAT_RADIUS/2,
                               0,        sqrt(3)*MOTORPLAT_RADIUS/2,  -sqrt(3)*MOTORPLAT_RADIUS/2,
                               0,                                 0,                            0;
    base_fram_ = top_fram_;

    para_.inverse[CURRENT].target_dist = Eigen::Vector3d(MID_PARA_LEN,MID_PARA_LEN,MID_PARA_LEN);
    para_.inverse[CURRENT].n_vector3 << Eigen::Vector3d(0,0,1), Eigen::Vector3d(0,0,1), Eigen::Vector3d(0,0,1);
    para_.inverse[NEXT] = para_.inverse[CURRENT];

    para_.rot_matrix[CURRENT] = Eigen::Matrix3d::Identity(3,3);
    para_.rot_matrix[NEXT] = Eigen::Matrix3d::Identity(3,3);

    laser_filters_.resize(3);
    for (size_t i = 0; i < laser_filters_.size(); i++)
    {
        laser_filters_[i].reset(new LowPassFilter(filter_coefficient_));
    }
    
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
    joint_info1.stat = PARA_CMD_INIT;
    joint_info1.laser = 0;
    joint_info1.cmd = PARA_CMD_INIT;
    para_motor_.push_back(joint_info1);
    // 2
    Joint joint_info2;
    #ifndef SIMULATE
    joint_info2.pub = nh_.advertise<std_msgs::Float64>("/motors/joint2_position_controller/command", 10);
    #else
    joint_info2.pub = nh_.advertise<std_msgs::Float64>("/parabot/joint2_position_controller/command", 10);
    joint_info2.sub = nh_.subscribe("/parabot/laser2", 10, &para::laser_callback2,this);
    #endif
    joint_info2.stat = PARA_CMD_INIT;
    joint_info2.laser = 0;
    joint_info2.cmd = PARA_CMD_INIT;
    para_motor_.push_back(joint_info2);
    // 3
    Joint joint_info3;
    #ifndef SIMULATE
    joint_info3.pub = nh_.advertise<std_msgs::Float64>("/motors/joint3_position_controller/command", 10);
    #else
    joint_info3.pub = nh_.advertise<std_msgs::Float64>("/parabot/joint3_position_controller/command", 10);
    joint_info3.sub = nh_.subscribe("/parabot/laser3", 10, &para::laser_callback3,this);
    #endif
    joint_info3.stat = PARA_CMD_INIT;
    joint_info3.laser = 0;
    joint_info3.cmd = PARA_CMD_INIT;
    para_motor_.push_back(joint_info3);

    pub_msgs();
}

#ifndef SIMULATE
void para::laser_callback1(const ppr_msgs::laser &msg)
{
    para_motor_[0].laser = laser_filters_[0]->update_filter(msg.sensor1 / 1000.0);
    para_motor_[1].laser = laser_filters_[1]->update_filter(msg.sensor2 / 1000.0);
    para_motor_[2].laser = laser_filters_[2]->update_filter(msg.sensor3 / 1000.0);
    laser_state_ = msg.state;

    #ifdef TEST_FILTER
    laser_sensors_raw_(0) = msg.sensor1 / 1000.0;
    laser_sensors_raw_(1) = msg.sensor2 / 1000.0;
    laser_sensors_raw_(2) = msg.sensor3 / 1000.0;
    #endif
}
#else
void para::laser_callback1(const sensor_msgs::LaserScan &msg)
{
    para_motor_[0].laser = laser_filters_[0]->update_filter(msg.ranges[0]);
    if (msg.ranges[0] <= 1){
        current_laser_state_ &= ~((uint16_t)1<<0);
    }
    else{
        current_laser_state_ |= (uint16_t)1<<0; 
    }
    #ifdef TEST_FILTER
    laser_sensors_raw_(0) = msg.ranges[0];
    #endif
}
void para::laser_callback2(const sensor_msgs::LaserScan &msg)
{
    para_motor_[1].laser = laser_filters_[1]->update_filter(msg.ranges[0]);
    if (msg.ranges[0] <= 1){
        current_laser_state_ &= ~((uint16_t)1<<1);
    }
    else{
        current_laser_state_ |= (uint16_t)1<<1; 
    }
    #ifdef TEST_FILTER
    laser_sensors_raw_(1) = msg.ranges[0];
    #endif
}
void para::laser_callback3(const sensor_msgs::LaserScan &msg)
{
    para_motor_[2].laser = laser_filters_[2]->update_filter(msg.ranges[0]);
    if (msg.ranges[0] <= 1){
        current_laser_state_ &= ~((uint16_t)1<<2);
    }
    else{
        current_laser_state_ |= (uint16_t)1<<2; 
    }
    #ifdef TEST_FILTER
    laser_sensors_raw_(1) = msg.ranges[0];
    #endif
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

// TODO: Eular gamma: -atan( (sin(delta_eular(0))*sin(delta_eular(1))) / (cos(delta_eular(0))+cos(delta_eular(1))) );

// Eigen::Vector3d para::inverse_solu(Eigen::Matrix3d rotm, float top_z, Eigen::Matrix3d& xzy, Eigen::Vector3d& xyz_v)
inverse_info para::inverse_solu(Eigen::Matrix4d T)
{
    /* variable */
    inverse_info inverse;
    Eigen::Matrix3d link_matrix,R;
    Eigen::Vector3d eular;
    float s1,s2,s3,c1,c2,c3,arfa,beta,gama;
    Eigen::Vector3d desired_eula, rot_vectors;

    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            R(i,j) = T(i,j);
        }
    }

    rot_vectors << T(0,3), T(1,3), T(2,3);
    link_matrix << rot_vectors,rot_vectors,rot_vectors;

    inverse.n_vector3 = R * top_fram_ + link_matrix - base_fram_;
    
    inverse.target_dist << sqrt( pow(inverse.n_vector3(0,0),2) + pow(inverse.n_vector3(1,0),2) + pow(inverse.n_vector3(2,0),2)), 
                           sqrt( pow(inverse.n_vector3(0,1),2) + pow(inverse.n_vector3(1,1),2) + pow(inverse.n_vector3(2,1),2)), 
                           sqrt( pow(inverse.n_vector3(0,2),2) + pow(inverse.n_vector3(1,2),2) + pow(inverse.n_vector3(2,2),2));
    return inverse;
}
// length: motor lengths
// n_vector3 : 3motor n vector 
Jacobian_info para::Jacobian(Eigen::Matrix3d rotm, Eigen::Vector3d length, Eigen::Matrix3d n_vector3)
{
    Jacobian_info jacobian;
    double factor_1,factor_2,factor_3,factor_4,factor_5;
    double alpha,beta,gamma;
    Eigen::Vector3d eular;
    Eigen::Matrix3d r,n,r_n_cross,jac_part;
    Eigen::MatrixXd jac1_transposed(6,3);  // 转置矩阵
    Eigen::MatrixXd jac1(3,6);
    Eigen::MatrixXd jac2(6,6);
    Eigen::MatrixXd jac3(6,3);

    eular = rotm2Eul(rotm.col(2));
    alpha = eular(0);
    beta  = eular(1);
    gamma = eular(2);

    r = rotm * base_fram_;
    n.col(0) = n_vector3.col(0) / length(0);
    n.col(1) = n_vector3.col(1) / length(1);
    n.col(2) = n_vector3.col(2) / length(2);
    r_n_cross.col(0) = (r.col(0)).cross(n.col(0));
    r_n_cross.col(1) = (r.col(1)).cross(n.col(1));
    r_n_cross.col(2) = (r.col(2)).cross(n.col(2));

    jac1_transposed.row(0) = n.row(0);
    jac1_transposed.row(1) = n.row(1);
    jac1_transposed.row(2) = n.row(2);
    jac1_transposed.row(3) = r_n_cross.row(0);
    jac1_transposed.row(4) = r_n_cross.row(1);
    jac1_transposed.row(5) = r_n_cross.row(2);
    jac1 = jac1_transposed.transpose();
    jac2 << 1, 0, 0, 0,          0,                      0,
            0, 1, 0, 0,          0,                      0,
            0, 0, 1, 0,          0,                      0,
            0, 0, 0, 1,          0,              sin(beta),
            0, 0, 0, 0, cos(alpha),  -sin(alpha)*cos(beta),
            0, 0, 0, 0, sin(alpha),   cos(alpha)*cos(beta);
    
    factor_1 = 1 + cos(alpha)*cos(beta);
    factor_2 = sin(alpha)*cos(beta)*cos(gamma);
    factor_3 = sin(beta)*sin(gamma);
    factor_4 = sin(alpha)*sin(gamma);
    factor_5 = sin(beta)*cos(gamma);
    
    jacobian.trans <<   0, 
                            0.5*MOTORPLAT_RADIUS*( factor_2*(cos(beta)+cos(alpha)) + factor_3*cos(beta)*(1+cos(alpha)*cos(alpha)) ) / factor_1,  
                                                                0.5*MOTORPLAT_RADIUS*( 2*factor_4*cos(beta) - factor_5*(factor_1 + sin(alpha)*sin(alpha)) - factor_4*cos(alpha)*sin(beta)*sin(beta) ) / factor_1,
                        0,                                                                     -MOTORPLAT_RADIUS*factor_5*cos(beta) / factor_1,        -MOTORPLAT_RADIUS*( factor_3*factor_1 + factor_2 ) / factor_1,
                        1,                                                                                                                  0,                                                                   0,
                        0,                                                                                                                  1,                                                                   0,
                        0,                                                                                                                  0,                                                                   1,
                        0,                                                                                              -sin(beta) / factor_1,                                              -sin(alpha) / factor_1;
    
    jacobian.j3 = (jac1 * jac2 * jacobian.trans ).inverse();
    return jacobian;
}

Eigen::Matrix4d para::forward(Eigen::Matrix4d last_T, Eigen::Matrix3d last_nv3, Eigen::Vector3d last_length, Eigen::Vector3d current_length)
{

    Eigen::Matrix4d current_T;
    Eigen::Matrix3d last_R,current_R;
    inverse_info temp_inverse_info;
    Jacobian_info temp_jacobian;
    Eigen::Vector3d temp_length, error_length, rotm_vec, temp_eular,delta_eular,result_jacobian,delta_length,temp_xyzv;
    Eigen::Matrix3d temp_nv3,temp_rotm;
    int loop_count = 0;
    bool forward_finished = 0;
    float temp_z;

    const Eigen::Vector3d target_length = current_length;


    /* parse data */
    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            last_R(i,j) = last_T(i,j);
        }
    }    
    temp_length   = last_length;
    temp_nv3      = last_nv3;
    temp_rotm     = last_R;
    temp_z = temp_length.sum()/3 + result_jacobian(0);

    
    temp_eular = rotm2Eul(temp_rotm.col(2));
    error_length = target_length - temp_length;

    // temp_jacobian = Jacobian(temp_rotm, temp_length, temp_nv3);

    if ((current_length - Eigen::Vector3d(MID_PARA_LEN,MID_PARA_LEN,MID_PARA_LEN)).array().abs().maxCoeff() <= 1e-5)
    {
        current_T = makeTrans(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,0,MID_PARA_LEN));
        return current_T;
    }
    
    /* loop */
    while(error_length.array().abs().maxCoeff() >= 1e-4)
    {
        // std::cout << "error_length.array().abs().maxCoeff() = " << error_length.array().abs().maxCoeff() << std::endl;
        // std::cout << "error_length \r\n" << error_length << std::endl;
        loop_count++;
        temp_jacobian = Jacobian(temp_rotm, temp_length, temp_nv3);
        /* start forward */
        result_jacobian = temp_jacobian.j3 * error_length;
        delta_eular(0) = result_jacobian(1);
        delta_eular(1) = result_jacobian(2);
        // delta_eular(2) = -atan( (sin(delta_eular(0))*sin(delta_eular(1))) / (cos(delta_eular(0))+cos(delta_eular(1))) );
        delta_eular(2) = -(delta_eular(0) * sin(temp_eular(1)) + delta_eular(1) * sin(temp_eular(0)))/ (1 + cos(temp_eular(0))*cos(temp_eular(1)));
        temp_eular = temp_eular + delta_eular;
        // temp_rotm = eul2Rotm(temp_eular);
        temp_rotm = eul2Rotm(temp_eular);
        temp_z = temp_length.sum()/3 + result_jacobian(0);

        temp_inverse_info = inverse_solu(makeTrans(temp_rotm,Eigen::Vector3d(0,0,temp_z)));

        temp_length = temp_inverse_info.target_dist;
        
        // temp_length = inverse_solu(temp_rotm,temp_z,temp_xyz,temp_xyzv);
        /* whether finished */
        error_length = target_length - temp_length;
        if(loop_count>=100)
        {
            ROS_ERROR("Para robot forward error, too many loops!");
            ROS_INFO("current length is [%f, %f, %f]",current_length(0),current_length(1),current_length(2));
            ROS_INFO("last length is [%f, %f, %f]",last_length(0),last_length(1),last_length(2));
            ros::shutdown();
            break;
        }
    }
    // ROS_INFO("Forward finish, loop count is:%d",loop_count);
    // ROS_INFO("current length is [%f, %f, %f]",current_length(0),current_length(1),current_length(2));
    // ROS_INFO("last length is [%f, %f, %f]",last_length(0),last_length(1),last_length(2));

    // std::cout << "temp jacobian is \r\n" << temp_jacobian.j3<<std::endl;
    // para->jacobian = temp_jacobian;
    // para->xyz = temp_xyz;
    // para->xyz_v = temp_xyzv;
    current_T = makeTrans(temp_rotm,Eigen::Vector3d(0,0,temp_z));
    // ROS_INFO("tempz is [%f]",temp_z);
    return current_T;
}

Eigen::Matrix4d para::makeTrans(Eigen::Matrix3d R, Eigen::Vector3d V)
{
    // Make transformation 4x4 by R and V
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Vector3d eular,V_actual;
    Eigen::Matrix3d R_actual;
    float s1,s2,s3,c1,c2,c3;

    eular = rotm2Eul(R.col(2));

    c1 = cos(eular(0));
    c2 = cos(eular(1));
    s1 = sin(eular(0));
    s2 = sin(eular(1));
   
    eular(2) = -MOTORPLAT_RADIUS * (s1*s2/(c1+c2)); // passive gamma

    V_actual << 0.5*MOTORPLAT_RADIUS*(c2*c3+s1*s2*s3-c1*c3), MOTORPLAT_RADIUS*(c2*s3),V(2);

    R_actual = eul2Rotm(eular);

    // x y  and construct a new rotation matirx

    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            T(i,j) = R_actual(i,j);
        }
        T(i,3) = V(i);        
    }
    
    return T;
}

Eigen::Matrix3d para::eul2Rotm(Eigen::Vector3d& euler_ZYX)
{
    //将欧拉角转变为旋转矩阵
    Eigen::Matrix3d rotm;
    float tan3;
    Eigen::Vector3d c = Eigen::Vector3d(cos(euler_ZYX(0)), cos(euler_ZYX(1)), cos(euler_ZYX(2)));
    Eigen::Vector3d s = Eigen::Vector3d(sin(euler_ZYX(0)), sin(euler_ZYX(1)), sin(euler_ZYX(2)));

    tan3 = -s(0)*s(1)/(c(0)*c(1));
    s(2) = tan3/(sqrt(pow(tan3,2) + 1));
    c(2) = 1/(sqrt(pow(tan3,2) + 1));

    rotm <<                c(1)*c(2),                 -c(1)*s(2),        s(1),
            c(0)*s(2)+c(2)*s(0)*s(1),   c(0)*c(2)-s(0)*s(1)*s(2),  -c(1)*s(0),
            s(0)*s(2)-c(0)*c(2)*s(1),   c(2)*s(0)+c(0)*s(1)*s(2),   c(0)*c(1);

    return rotm;
}

Eigen::Vector3d para::rotm2Eul(Eigen::Vector3d vec_in)
{
    Eigen::Vector3d vec;
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

Eigen::Matrix3d para::normal_vec_rotm(Eigen::Vector3d normal_vec)
{
    Eigen::Matrix<double,2,3> element;
    float s1,s2,s3,c1,c2,c3,arfa,beta,gama;
    float theta;
    float denominator,x,y,z;
    Eigen::Vector3d vector_z,vector_wall,vector;
    Eigen::Matrix3d rotm;

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

Eigen::Matrix<double, 2, 3> para::get_rot_element(Eigen::Vector3d vec)
{
    Eigen::Matrix<double,2,3> element;
    float x,y,z,a,b,c;
    float s1,s2,s3,c1,c2,c3,tan1,tan3;
    float denominator;
    Eigen::Matrix3d rotm;

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
Eigen::Vector3d para::get_wall_plat_vec(Eigen::Vector3d laser_dist)
{
    Eigen::Matrix3d wall_fram;
    Eigen::Vector3d p12,p13,temp_col;

    wall_fram << LASERPLAT_RADIUS,                             -LASERPLAT_RADIUS/2,                 -LASERPLAT_RADIUS/2,
                               0,                       sqrt(3)*LASERPLAT_RADIUS/2,         -sqrt(3)*LASERPLAT_RADIUS/2,
                laser_dist(0) - LASER_DIST_UPPER,   laser_dist(1)-LASER_DIST_UPPER,     laser_dist(2)-LASER_DIST_UPPER;
    p12 = wall_fram.col(1) - wall_fram.col(0);
    p13 = wall_fram.col(2) - wall_fram.col(0);

    temp_col = wall_fram.col(1);
    return p12.cross(p13);
}

float para::max_error(Eigen::Vector3d vector)
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
    #ifndef SIMULATE
    if(laser_state_ == 0){
    #else
    if(current_laser_state_ == 0){
    #endif
    /* variable */
        Eigen::Vector3d wall_plat_vector,wall_eular,pid_error;
        Eigen::Matrix3d rot_matrix,rot_matrix_temp,laser2Base,laser2Base_matrix;
        Eigen::Vector3d temp_dist;
        ros::Time last_time,curr_time;
        ros::Duration control_duration;
        float error;

        para_.laser_dist << para_motor_[0].laser , para_motor_[1].laser, para_motor_[2].laser;
        wall_plat_vector = get_wall_plat_vec(para_.laser_dist);

        if(max_error(para_.laser_dist)>0.001){
            rot_matrix = normal_vec_rotm(wall_plat_vector);
        }
        else{
            rot_matrix = Eigen::Matrix3d::Identity(3,3);
        }
        wall_eular = rotm2Eul(rot_matrix.col(2));
        wall_eular(2) = 0; // gamma is a passive joint

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

        rot_matrix_temp = eul2Rotm(pid_error);

        para_.rot_matrix[NEXT] = rot_matrix_temp * para_.rot_matrix[CURRENT];
        //test forward
        Eigen::Matrix4d test_forward;
        Eigen::Matrix3d test_R;
        test_forward = forward(makeTrans(para_.rot_matrix[CURRENT],Eigen::Vector3d(0,0,para_.inverse[CURRENT].target_dist.sum()/3)),para_.inverse[CURRENT].n_vector3,para_.inverse[CURRENT].target_dist,para_.inverse[NEXT].target_dist);

        for (size_t i = 0; i < 3; i++)
        {
            for (size_t j = 0; j < 3; j++)
            {
                test_R(i,j)=test_forward(i,j);
            }
        }
        // para_.rot_matrix[NEXT] = rot_matrix_temp * test_R;

        // std::cout << "traditional T matrix is \r\n" << makeTrans(para_.rot_matrix[NEXT],Eigen::Vector3d(0,0,para_.inverse[NEXT].target_dist.sum()/3)) << std::endl;
        // std::cout << "Jacobian T matrix is \r\n" << test_forward << std::endl;

        para_.inverse[CURRENT] = para_.inverse[NEXT]; // update state
        para_.inverse[NEXT] = inverse_solu(makeTrans(para_.rot_matrix[NEXT],Eigen::Vector3d(0,0,MID_PARA_LEN)));
        // temp_dist = para_.inverse.target_dist;

        for (size_t i = 0; i < para_motor_.size(); i++)
        {
            para_motor_[i].stat = para_motor_[i].cmd;
            para_motor_[i].cmd = para_.inverse[NEXT].target_dist(i) - MIN_PARA_LEN;
        }


        /* saved status */
        para_.rot_matrix[CURRENT] = para_.rot_matrix[NEXT];
        current_wall_dist_ = (para_.laser_dist(0) + para_.laser_dist(1) + para_.laser_dist(2))/3;
        // current_ppr_dist_ = (para_motor_[0].stat + para_motor_[1].stat + para_motor_[2].stat)/3;
    }
    pub_msgs();
}

/* public function */
/*
 * 1. Update ppr motor length 
 * 2. ppr forward kinematics, update rotation matrix
 * 3. Calculate current Jaociban matrix, update current rotation matrix
*/
void para::ppr_update(void)
{
    // 1 
    para_.laser_dist << para_motor_[0].laser , para_motor_[1].laser, para_motor_[2].laser;
    current_wall_dist_ = para_.laser_dist.sum()/3;
    for (size_t i = 0; i < para_motor_.size(); i++)
    {
        para_motor_[i].stat = para_motor_[i].cmd;
    }
    para_.inverse[LAST] = para_.inverse[CURRENT];
    para_.rot_matrix[LAST] = para_.rot_matrix[CURRENT];
    para_.inverse[CURRENT].target_dist = Eigen::Vector3d(para_motor_[0].stat,para_motor_[1].stat,para_motor_[2].stat) + 
                                         Eigen::Vector3d(MIN_PARA_LEN,MIN_PARA_LEN,MIN_PARA_LEN);
    
    // 2
    para_.trans_matrix = forward(makeTrans(para_.rot_matrix[LAST],Eigen::Vector3d(0,0,para_.inverse[LAST].target_dist.sum()/3)),para_.inverse[LAST].n_vector3,para_.inverse[LAST].target_dist,para_.inverse[CURRENT].target_dist);
    para_.rot_matrix[CURRENT] = EigenT2EigenR(para_.trans_matrix);
    // std::cout << "para rot angle is \r\n" << EigenT2Pos(para_.trans_matrix) << std::endl;

    //3 
    para_.jacobian = Jacobian(para_.rot_matrix[CURRENT],para_.inverse[CURRENT].target_dist,para_.inverse[CURRENT].n_vector3);

}

/*
 * for redundant kinematics pub msg
*/
void para::pub_msgs(Eigen::Vector3d cmd)
{
    std_msgs::Float64 cmd_msgs;
    for (size_t i = 0; i < para_motor_.size(); i++)
    {
        // para_motor_[i].cmd += length_inc(i);
        cmd(i) = clamp(cmd(i),MIN_PARA_LEN,MAX_PARA_LEN);
        para_motor_[i].cmd = cmd(i) - MIN_PARA_LEN;
        cmd_msgs.data = para_motor_[i].cmd;
        para_motor_[i].pub.publish(cmd_msgs);
    }
}

// for redundant kinematics
Eigen::Matrix<double,6,3> para::get_Jacobian(void)
{
    return para_.jacobian.trans * para_.jacobian.j3;
}

Eigen::Matrix3d para::get_ppr_r(void){
    return para_.rot_matrix[CURRENT];
}

float para::get_wall_dist(void){
    return current_wall_dist_;
}

Eigen::Vector3d para::get_ppr_dist(void){
    return para_.inverse[CURRENT].target_dist;
}

Eigen::Vector3d para::get_laser_dist(void){
    return para_.laser_dist;
}

#ifdef TEST_FILTER
Eigen::Vector3d para::get_laser_raw(void){
    return laser_sensors_raw_;
}
#endif

uint16_t para::get_laser_state(void){
#ifndef SIMULATE
    return laser_state_;    
#else
    return current_laser_state_;
#endif
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