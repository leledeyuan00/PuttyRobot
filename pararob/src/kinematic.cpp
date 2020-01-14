#include "pararob/kinematic.h"


MOTOR_KINE::MOTOR_KINE():spinner(1),PLANNING_GROUP("manipulator"), move_group_(PLANNING_GROUP)
{
    spinner.start();
    joint_model_group_ = move_group_.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    current_state_ = move_group_.getCurrentState();
    // joint_names = joint_model_group_->getVariableNames();
}

/* 解算下一时刻电机杆长和UR关节角度 */
void MOTOR_KINE::motor_traj_gen(PARA_SOLU* para, UR5_SOLU* ur)
{
    /* variable */
    Eigen::Vector3f wall_plat_vector,wall_eular;
    Eigen::Matrix3f rot_matrix,laser2Base,laser2Base_matrix;
    /* algorithm */
    // static int i=0;
    Eigen::Vector3f desired_eula;
    Eigen::VectorXd desired_pp,diff_pp,current_pp;
    Eigen::VectorXd delta_pose,diff_pose;
    Eigen::MatrixXd inverse_base6,laser_matrix6,para_matrix6;

    inverse_base6.resize(6,6);
    laser_matrix6.resize(6,6);
    para_matrix6.resize(6,6);
    desired_pp.resize(6);
    current_pp.resize(6);
    diff_pp.resize(6);
    delta_pose.resize(6);
    diff_pose.resize(6);


    /* for test*/
    
    // static float t = 0;
    // static int motor_dir = 0;
    // float z1,z2,z3,a4,l1,l2,l3;
    // float period = 1;
    // z1 = 0.027;
    // z2 = 0.001;
    // z3 = 0.001;
    // l1 = 0.001;
    // l2 = 0;
    // l3 = 0;
    // a4 = -10 * 3.14 /180;
    // if(t<period*2-0.01)
    //     t+= 0.01;
    // else
    //     t = 0;
    // if(t>=0 && t <period)
    // {
    //     // para->motor_dist[NEXT](0) = MOTOR_LEN_INIT + z1*(t/period);
    //     // para->motor_dist[NEXT](1) = MOTOR_LEN_INIT + z2*(t/period);
    //     // para->motor_dist[NEXT](2) = MOTOR_LEN_INIT + z3*(t/period);
    //     // para->laser_dist(0) = l1 * (t/period);
    //     // para->laser_dist(1) = l2 * (t/period);
    //     // para->laser_dist(2) = l3 * (t/period);
    //     desired_eula << a4 * (t/period),0,0;
    // }
    // else if(t>=period && t<period*2)
    // {
    //     // para->motor_dist[NEXT](0) = MOTOR_LEN_INIT - z1*((t-period)/period) + z1;
    //     // para->motor_dist[NEXT](1) = MOTOR_LEN_INIT - z2*((t-period)/period) + z2;
    //     // para->motor_dist[NEXT](2) = MOTOR_LEN_INIT - z3*((t-period)/period) + z3; 
    //     // para->laser_dist(0) = -l1 * ((t-period)/period);
    //     // para->laser_dist(1) = -l2 * ((t-period)/period);
    //     // para->laser_dist(2) = -l3 * ((t-period)/period);
    //     desired_eula << -a4 * ((t-period)/period),0,0;
    // }

    /* algorithm */
    para_plat_update(para);
    // ROS_INFO("First Motor Rot Matrix is :\r\n [%f, %f, %f] \r\n [%f, %f, %f]\r\n [%f, %f, %f]", para->rot_matrix[LAST](0,0),para->rot_matrix[LAST](0,1),para->rot_matrix[LAST](0,2),para->rot_matrix[LAST](1,0),para->rot_matrix[LAST](1,1),para->rot_matrix[LAST](1,2),para->rot_matrix[LAST](2,0),para->rot_matrix[LAST](2,1),para->rot_matrix[LAST](2,2));
    /* 1 fetch laser data and solution tool-wall rotation matrix */
    wall_plat_vector = get_wall_plat_vec(para->laser_dist);
    ROS_INFO("Wall platform vector is : [%f, %f, %f]", wall_plat_vector(0), wall_plat_vector(1),wall_plat_vector(2));

    if(max_error(para->laser_dist)>0.001){
        rot_matrix = normal_vec_rotm(wall_plat_vector);
    }
    else{
        rot_matrix = Eigen::Matrix3f::Identity(3,3);
    }  
    wall_eular = rotm2Eul(rot_matrix.col(2));
    ROS_INFO("Wall Eular is : [%f, %f, %f]", wall_eular(0), wall_eular(1),wall_eular(2));
    //
    // for (size_t i = 0; i < 3; i++)
    // {
    //     if(fabs(wall_eular(i))>0.01)
    //         wall_eular(i) = wall_eular(i)>0?0.01:-0.01; 
    //     // else if(fabs(wall_eular(i))<0.001)
    //     //     wall_eular(i) = 0;
    // }
    // eul2Rotm(wall_eular,rot_matrix);
    
    ROS_INFO("Wall Rot Matrix is :\r\n [%f, %f, %f] \r\n [%f, %f, %f]\r\n [%f, %f, %f]", rot_matrix(0,0),rot_matrix(0,1),rot_matrix(0,2),rot_matrix(1,0),rot_matrix(1,1),rot_matrix(1,2),rot_matrix(2,0),rot_matrix(2,1),rot_matrix(2,2));

    /* 2 get tool transfer matrix for base frame by forward kinematic */
    Eigen::Matrix4d base_matrix,inverse_base;
    Eigen::Matrix3f base_matrix3,inverse_base3;
    Eigen::Matrix4d ur_matrix,tool_matrix;
    base_matrix3 << -0.707107, -0.707107, 0,
                    0.707107, -0.707107, 0,
                           0,         0, 1;
    base_matrix << -0.707107, -0.707107, 0, 0,
                    0.707107, -0.707107, 0, 0,
                           0,         0, 1, 0,
                           0,         0, 0, 1;
    // base_matrix << 0,-1, 0, 0,
    //                1, 0, 0, 0,
    //                0, 0, 1, 0,
    //                0, 0, 0, 1;
    laser2Base << 0,1,0,
                  -1,0,0,
                  0,0,1;
    inverse_base6 = Eigen::MatrixXd::Zero(6,6);
    laser_matrix6 = Eigen::MatrixXd::Zero(6,6);
    inverse_base = base_matrix.inverse();
    inverse_base3 = base_matrix3.inverse();

    rot_matrix_d_ = rot_matrix_d_ * rot_matrix;
    // laser2Base_matrix =  rot_matrix;
    // laser2Base_matrix = Eigen::Matrix3f::Identity(3,3);

    para->rot_matrix[CURRENT] = para_forward(para);
    ROS_INFO("Forward Motor Rot Matrix is :\r\n [%f, %f, %f] \r\n [%f, %f, %f]\r\n [%f, %f, %f]", para->rot_matrix[CURRENT](0,0),para->rot_matrix[CURRENT](0,1),para->rot_matrix[CURRENT](0,2),para->rot_matrix[CURRENT](1,0),para->rot_matrix[CURRENT](1,1),para->rot_matrix[CURRENT](1,2),para->rot_matrix[CURRENT](2,0),para->rot_matrix[CURRENT](2,1),para->rot_matrix[CURRENT](2,2));

    laser2Base_matrix =   ( para->rot_matrix[CURRENT] * rot_matrix);
    
    forward(ur->joint_cur,ur->transform_matrix);
    for (size_t i = 0; i < 4; i++)
    {
        for (size_t j = 0; j < 4; j++)
        {
            ur_matrix(i,j) = ur->transform_matrix[i*4+j];
        }
    }

    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            inverse_base6(i,j) = inverse_base(i,j);
            inverse_base6(i+3,j+3) = inverse_base(i,j);
            laser_matrix6(i,j) = laser2Base_matrix(i,j);
            laser_matrix6(i+3,j+3) = laser2Base_matrix(i,j);
            para_matrix6(i,j) = para->rot_matrix[CURRENT](i,j);
            para_matrix6(i+3,j+3) = para->rot_matrix[CURRENT](i,j);
        }
        
    }
        
    Eigen::Vector3f tool_eular,tool_vector;
    tool_matrix =  base_matrix*ur_matrix ;
    tool_vector << tool_matrix(0,2),tool_matrix(1,2),tool_matrix(2,2);
    tool_eular = rotm2Eul(tool_vector);

    static int dir = 0;
    ROS_INFO("current Z is :%f",current_z_);
    ROS_INFO("current X is :%f",current_x_);
    if(recored_z_ <= 10){
        recored_z_++;
        current_z_ = tool_matrix(1,3)-0.01;
        current_x_ = tool_matrix(0,3);
    }
    if(tool_matrix(0,3) >=(current_x_ + 0.2) && dir ==0){
        dir = 1;
    }
    else if (tool_matrix(1,3) >= (current_z_)&&dir ==1){
        dir = 2;
    }
    else if(tool_matrix(0,3) <= current_x_ &&dir==2 )
        dir = 3;
    else if(tool_matrix(1,3) <= (current_z_)&&dir==3){
        dir = 0;
    }
    if(dir==0){
        delta_pose << 0.01,0,0,0,0,0;
    }
    else if(dir==1){
        delta_pose << 0,0.003,0,0,0,0;
    }
    else if(dir ==2){
        delta_pose << -0.01,0,0,0,0,0;
    }
    else if(dir==3){
        delta_pose << 0,-0.002,0,0,0,0;
    }

    // if(wall_eular(1)>0){
    //     delta_pose(4) = 0.001;
    // }
    // else if(wall_eular(1)<0){
    //     delta_pose(4) = -0.001;
    // }
    // else{
    //     delta_pose(4) = 0;
    // }
    // if(tool_eular(1) >= PI/6)
    //     dir = 1;
    // else if(tool_eular(1)<=PI/6)
    //     dir = 0;
    // if(dir){
    //     delta_pose << 0,0,0,0,-0.01,0;
    // }
    // else{
    //     delta_pose << 0,0,0,0,0.01,0;
    // }


    diff_pose =  inverse_base6 * laser_matrix6 * delta_pose;
    diff_pose(1) = diff_pose(1) + diff_pose(2);
    diff_pose(2) = 0;
    // diff_pose =  para_matrix6 * laser_matrix6 * inverse_base6 * delta_pose;

    
    ROS_INFO("Tool Eular is:[%f %f %f]",tool_eular(0)*180/3.14,tool_eular(1)*180/3.14,tool_eular(2)*180/3.14);
    ROS_INFO("Delta pose incre is : [%f %f %f]", diff_pose(0),diff_pose(1),diff_pose(2));
    
    ROS_INFO("UR transform matrix is");
    for (size_t i = 0; i < 4; i++)
    {
        ROS_INFO("[%f, %f, %f, %f]",tool_matrix(i,0),tool_matrix(i,1),tool_matrix(i,2),tool_matrix(i,3));
    }
    ROS_INFO("TCP POSITION is : [x=%f, y=%f, z=%f]",tool_matrix(0,3),tool_matrix(1,3),tool_matrix(2,3));
    
    /* 3 get target transfer matrix for base frame by 2*1 */
    Eigen::Matrix4d para_trans,robot_trans,tool2Base;
    // eul2Rotm(desired_eula,para->rot_matrix[NEXT]);
    para->rot_matrix[NEXT] = rot_matrix * para->rot_matrix[CURRENT];
    para_trans << para->rot_matrix[NEXT](0,0),para->rot_matrix[NEXT](0,1),para->rot_matrix[NEXT](0,2),0,
                  para->rot_matrix[NEXT](1,0),para->rot_matrix[NEXT](1,1),para->rot_matrix[NEXT](1,2),0,
                  para->rot_matrix[NEXT](2,0),para->rot_matrix[NEXT](2,1),para->rot_matrix[NEXT](2,2),PARA_LEN_MEAN,
                               0,             0,             0,1;
    
    robot_trans =  ur_matrix * para_trans;
    tool2Base = base_matrix.inverse() * robot_trans;
    ur->tool_xyz << tool2Base(0,3), tool2Base(1,3),tool2Base(2,3);
    ROS_INFO("TOOL XYZ!!!! [%f, %f, %f]",ur->tool_xyz(0),ur->tool_xyz(1),ur->tool_xyz(2));
    Eigen::Vector3f eular_current,eular_next,trans_vector;

    for (size_t i = 0; i < 3; i++)
    {
        trans_vector(i) = robot_trans(i,2);
    }
    
    eular_current = rotm2Eul(para->rot_matrix[CURRENT].col(2));
    eular_next = rotm2Eul(para->rot_matrix[NEXT].col(2));
    ROS_INFO("Eular Current is :\r\n ************ [%f, %f, %f]*********",eular_current(0)*180/3.1415926,eular_current(1)*180/3.1415926,eular_current(2)*180/3.1415926);
    ROS_INFO("Eular Next is :\r\n ************ [%f, %f, %f]*********",eular_next(0)*180/3.1415926,eular_next(1)*180/3.1415926,eular_next(2)*180/3.1415926);
    /* 4 get current Jacobin */
    Eigen::MatrixXd UR_jacobian,Robot_jacobian;
    Eigen::MatrixXd Robot_jacobian_trans,Robot_jacobian_inv;
    Eigen::MatrixXd para_jacobian63,jacobian_e2w,para_jacobian2b,para_jacobian_last;
    Eigen::Matrix3d f2d_jacobian;
    Eigen::Matrix<double,1,3> trans_vx,trans_vy,trans_alpha,trans_beta,trans_gamma;
    float alpha = eular_current(0);
    float beta = eular_current(1);
    float gamma = eular_current(2);
    Robot_jacobian.resize(6,9);
    Robot_jacobian_trans.resize(9,6);
    Robot_jacobian_inv.resize(9,6);
    UR_jacobian.resize(6,6);
    para_jacobian2b.resize(6,6);
    jacobian_e2w.resize(6,6);
    para_jacobian63.resize(6,3);
    UR_jacobian = getJacobian();
    // Robot_jacobian(3,3) = UR_jacobian(3,3);
    para_jacobian2b = Eigen::MatrixXd::Zero(6,6);
    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            f2d_jacobian(i,j) = para->jacobian(i,j);
            para_jacobian2b(i,j) = ur_matrix(i,j);
            para_jacobian2b(i+3,j+3) = ur_matrix(i,j);
        }        
    }
    jacobian_e2w << 1, 0, 0, 0,          0,                      0,
                    0, 1, 0, 0,          0,                      0,
                    0, 0, 1, 0,          0,                      0,
                    0, 0, 0, 1,          0,              sin(beta),
                    0, 0, 0, 0, cos(alpha),  -sin(alpha)*cos(beta),
                    0, 0, 0, 0, sin(alpha),   cos(alpha)*cos(beta);
    
    
    trans_alpha = f2d_jacobian.row(1);
    trans_beta  = f2d_jacobian.row(2);
    for (size_t i = 0; i < 3; i++)
    {
        trans_gamma(i) = -(sin(beta)*trans_alpha(i)+sin(alpha)*trans_beta(i))/(1+cos(alpha)*cos(beta));
    }
        
    trans_vx = 0.5*MOTOR_LEN_INIT*(cos(alpha)*sin(beta)*sin(gamma)*trans_gamma + (sin(alpha)*cos(beta)*sin(gamma)-sin(beta)*cos(gamma))*trans_beta+(sin(alpha)*sin(beta)*cos(gamma)+cos(alpha)*sin(gamma)-cos(beta)*sin(gamma))*trans_gamma);
    trans_vy = MOTOR_LEN_INIT*(-sin(beta)*sin(gamma)*trans_beta + cos(beta)*cos(gamma)*trans_gamma);

    para_jacobian63.row(0) = trans_vx;
    para_jacobian63.row(1) = trans_vy;
    para_jacobian63.row(2) = f2d_jacobian.row(0);
    para_jacobian63.row(3) = trans_alpha;
    para_jacobian63.row(4) = trans_beta;
    para_jacobian63.row(5) = trans_gamma;
    para_jacobian_last = para_jacobian2b * jacobian_e2w * para_jacobian63;
    std::cout <<"para jacobian is \r\n" << para->jacobian<<std::endl;
    std::cout<<"para_jacobian 63 is : \r\n "<<para_jacobian63<<std::endl;
    std::cout<<"para_jacobian_last is : \r\n "<<para_jacobian_last<<std::endl;

    Robot_jacobian << UR_jacobian, para_jacobian_last;
    Robot_jacobian_trans = Robot_jacobian.transpose();
    Robot_jacobian_inv = pinv(Robot_jacobian);
    // /* 5 differ target pose and position to get all theta and motor length */
    Eigen::VectorXd target_pose,delta_theta,joint_error,cmd_v,cmd_v1,cmd_v2;
    Eigen::VectorXd curr_joint,diffH,weight_diag,cmd_kp_diag;
    Eigen::MatrixXd weight,weight_transpose,cmd_kp,jacobian_part,posE2R,baseE2R,cmd_identity;
    float diffH_k1 = 0;
    float diffH_k2 = -0.002;
    float w1 = 0.0001;
    float w2 = 10;
    float w3 = 1.0;
    float kp1 = 1;
    float kp2 = 1;
    
    curr_joint.resize(9);
    diffH.resize(9);
    weight_diag.resize(9);
    weight.resize(9,9);
    cmd_kp_diag.resize(9);
    cmd_kp.resize(9,9);
    cmd_identity.resize(9,9);
    jacobian_part.resize(6,6);
    posE2R.resize(6,6);
    baseE2R.resize(3,3);
    joint_error.resize(9);
    cmd_v.resize(9);
    cmd_v1.resize(9);
    cmd_v2.resize(9);
    delta_theta.resize(6);

    weight = Eigen::MatrixXd::Zero(9,9);
    cmd_kp = Eigen::MatrixXd::Zero(9,9);
    cmd_identity = Eigen::MatrixXd::Identity(9,9);
    eular_current = rotm2Eul(trans_vector);
    alpha = eular_current(0);
    beta  = eular_current(1);
    baseE2R << 1,          0,             sin(beta),
               0, cos(alpha), -sin(alpha)*cos(beta),
               0, sin(alpha),  cos(alpha)*cos(beta);
    posE2R = Eigen::MatrixXd::Identity(6,6);
    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            posE2R(i+3,j+3) = baseE2R(i,j);
        }
    }

    for (size_t i = 0; i < ALL_JOINT_NUM; i++)
    {
        if(i < UR_JOINT_NUM){
            curr_joint(i) = ur->joint_cur[i];
        }
        else{
            curr_joint(i) = para->motor_dist[CURRENT](i-UR_JOINT_NUM);
        }
    }
    diffH = perform_func(curr_joint,diffH_k1,diffH_k2);

    for (size_t i = 0; i < 9; i++)
    {
        if(i<3){
            weight(i,i) = w1;
            cmd_kp(i,i) = kp1;
        }
        else if(i>=3 && i<6){
            weight(i,i) = w2;
            cmd_kp(i,i) = kp1;
        }
        else{
            weight(i,i) = w3;
            cmd_kp(i,i) = kp2;
        }
    }
    
    jacobian_part = Robot_jacobian * weight.inverse() * Robot_jacobian_trans;

    joint_error = weight.inverse() * Robot_jacobian_trans * jacobian_part.inverse() * posE2R  * diff_pose;

    cmd_v1 = cmd_kp * joint_error;
    cmd_v2 = (cmd_identity - (Robot_jacobian_inv * Robot_jacobian))*diffH;

    // cmd_v2 = Eigen::MatrixXd::Zero(9,1);
    std::cout <<"diff pose is \r\n" <<diff_pose<<std::endl;
    std::cout<< "diffH is \r\n" << diffH << std::endl;
    // std::cout<<"Robot_jacobian_inv is \r\n" <<Robot_jacobian_inv<<std::endl;
    // std::cout<<"Robot_jacobian is \r\n" <<Robot_jacobian<<std::endl;

    cmd_v = cmd_v1 + cmd_v2;
    std::cout<< "cmd_v1 is \r\n" << cmd_v1<<std::endl;
    std::cout<< "cmd_v2 is \r\n" << cmd_v2<<std::endl;
    std::cout<< "cmd_v is \r\n" << cmd_v<<std::endl;
    
    /* add */
    Eigen::Vector3d motor_delta;
    for (size_t i = UR_JOINT_NUM; i < ALL_JOINT_NUM; i++)
    {
        motor_delta(i-UR_JOINT_NUM) = cmd_v(i);
    }
    

    for (size_t i = 0; i < UR_JOINT_NUM; i++)
    {
        delta_theta(i) = cmd_v(i);
    }
    
    
    target_pose.resize(6);
    target_pose << 0,0,0,0,0,0;
    for (size_t i = 0; i < 3; i++)
    {
        target_pose(i)=diff_pose[i];
    }   
    std::cout << "target_pose is : "<<"["<<target_pose<<"]"<<std::endl;

    delta_theta = UR_jacobian.inverse() * target_pose;
    std::cout << "Delta_theta is : "<<"["<<delta_theta<<"]"<<std::endl;
    /* 6 return theta and para_length */
    
    Eigen::Matrix3f xyz_temp;
    Eigen::Vector3f temp_dist,xyzv_temp;
    float delta_mm = 0.0000005;
    float limit_delta_mm = 0.0001;
    float limit_mm = 0.0005;
    static int dir_delta[MOTOR_NUM] = {0,0,0};
    static int count_delta[MOTOR_NUM] = {0,0,0};
    temp_dist = inverse_solu(para->rot_matrix[NEXT],PARA_LEN_MEAN,xyz_temp,xyzv_temp);
    /* para length send */
    // for (size_t i = 0; i < MOTOR_NUM; i++)
    // {
    //     motor_delta(i) = temp_dist(i) - para->motor_dist[CURRENT](i);
    //     /* velocity threshold */
    //     if(fabs(temp_dist(i)-para->motor_dist[CURRENT](i))>=delta_mm &&fabs(temp_dist(i)-para->motor_dist[CURRENT](i))<=limit_mm)
    //     // if(fabs(motor_delta(i))>=delta_mm && fabs(motor_delta(i)<=20*delta_mm))
    //     {
    //         if((((temp_dist(i)-para->motor_dist[CURRENT](i))>=delta_mm)&&!dir_delta[i])||((temp_dist(i)-para->motor_dist[CURRENT](i))<=delta_mm&&dir_delta[i])){
    //             if(count_delta[i]<5){
    //                 count_delta[i]++;
    //             }
    //             else{
    //                 para->delta_mm[i] = para->delta_mm[i]<delta_mm? para->delta_mm[i]+0.000002:delta_mm;
    //             }
    //         }
    //         else{
    //             count_delta[i] = 0;
    //             para->delta_mm[i] = 0;
    //         }
    //         if((temp_dist(i)-para->motor_dist[CURRENT](i))>=delta_mm){
    //             dir_delta[i] = 0;
    //         }
    //         else{
    //             dir_delta[i] = 1;
    //         }
    //         para->motor_dist[NEXT](i) = (temp_dist(i)-para->motor_dist[CURRENT](i))>=delta_mm?(para->motor_dist[CURRENT](i)+para->delta_mm[i]):(para->motor_dist[CURRENT](i)-para->delta_mm[i])*1 + 0*para->motor_dist[CURRENT](i); 
    //     }
    //     else if(fabs(temp_dist(i))> limit_mm){
    //         para->motor_dist[NEXT](i) = (temp_dist(i)-para->motor_dist[CURRENT](i))>=delta_mm?((para->motor_dist[CURRENT](i)+limit_delta_mm)*0.8+ 0.2*para->motor_dist[CURRENT](i)):((para->motor_dist[CURRENT](i)-limit_delta_mm)*1 + 0*para->motor_dist[CURRENT](i)); 
    //     }
    //     else
    //         para->motor_dist[NEXT](i) = temp_dist(i);
    //     /* position threshold */
    //     if(para->motor_dist[NEXT](i) <= MOTOR_LEN_INIT + 0.001)
    //         para->motor_dist[NEXT](i) = MOTOR_LEN_INIT + 0.001;
    //     else if(para->motor_dist[NEXT](i) >= (MOTOR_LEN_INIT + 0.026))
    //         para->motor_dist[NEXT](i) = MOTOR_LEN_INIT + 0.026;
    // }
    // for (size_t i = 0; i < MOTOR_NUM; i++)
    // {
    //     /* code for loop body */
    //     para->motor_dist[NEXT](i) = para->motor_dist[CURRENT](i) + motor_delta(i);
    // }
    
    
    /* UR theta send */
    float limit_delta = 0.05;
    for (size_t i = 0; i < UR_JOINT_NUM; i++)
    {
        if(delta_theta(i) > limit_delta)
        {
            ROS_ERROR("delta joint limited !!!");
            delta_theta(i) = limit_delta;
            // ur->state = 0;
            // break;
        }
        else if(delta_theta[i] < -limit_delta)
        {
            ROS_ERROR("delta joint limited !!!");
            delta_theta(i) = -limit_delta;
            // ur->state = 0;
            // break;
        }
    }
    if(ur->state!=0){
        for (size_t i = 0; i < UR_JOINT_NUM; i++)
        {
            std::cout << "UR joint Error \r\n" << (ur->joint_next[i] - ur->joint_cur[i]) <<std::endl;
            /* code for loop body */
            ur->joint_next[i] = ur->joint_cur[i] + delta_theta(i);
        }        
    }
    ROS_INFO("next joint is [%f %f %f %f %f %f]",ur->joint_next[0],ur->joint_next[1],ur->joint_next[2],ur->joint_next[3],ur->joint_next[4],ur->joint_next[5]);
    

    #ifdef DEBUG    
    /* for test function */
    test_func();
    #endif
}


/* get Normal Vector by Laser Sensor */
Eigen::Vector3f MOTOR_KINE::get_wall_plat_vec(Eigen::Vector3f laser_dist)
{
    Eigen::Matrix3f wall_fram;
    Eigen::Vector3f p12,p13,temp_col;

    wall_fram << LASERPLAT_RADIUS,               -LASERPLAT_RADIUS/2,         -LASERPLAT_RADIUS/2,
                               0,        sqrt(3)*LASERPLAT_RADIUS/2, -sqrt(3)*LASERPLAT_RADIUS/2,
                   laser_dist(0),                     laser_dist(1),               laser_dist(2);
    p12 = wall_fram.col(1) - wall_fram.col(0);
    p13 = wall_fram.col(2) - wall_fram.col(0);

    temp_col = wall_fram.col(1);
    ROS_INFO("laser_dist : %f, %f, %f;",laser_dist(0),laser_dist(1),laser_dist(2)); 
    ROS_INFO("wall_col1 : %f, %f, %f;",temp_col(0),temp_col(1),temp_col(2));
    ROS_INFO("P12 : %f ,%f, %f;",p12(0),p12(1),p12(2));
    ROS_INFO("P13 : %f ,%f, %f;",p13(0),p13(1),p13(2));
    return p12.cross(p13);
}

/* get Rotation Matrix for this parallel platform */
/* R(x)*R(y) -- R(z)=I */

Eigen::Matrix3f MOTOR_KINE::normal_vec_rotm(Eigen::Vector3f normal_vec)
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

Eigen::Matrix<float, 2, 3> MOTOR_KINE::get_rot_element(Eigen::Vector3f vec)
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

void MOTOR_KINE::eul2Rotm(Eigen::Vector3f& euler_ZYX, Eigen::Matrix3f& rotm)
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

Eigen::Vector3f MOTOR_KINE::rotm2Eul(Eigen::Vector3f vec_in)
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

/* pinv */
Eigen::MatrixXd MOTOR_KINE::pinv(Eigen::MatrixXd A)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);//M=USV*
    double  pinvtoler = 1.e-8; //tolerance
    int row = A.rows();
    int col = A.cols();
    int k = 6;    //int k = min(row,col);
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col,row);
    Eigen::MatrixXd singularValues_inv = svd.singularValues();//奇异值
    Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
    for (long i = 0; i<k; ++i) {
        if (singularValues_inv(i) > pinvtoler)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else singularValues_inv(i) = 0;
    }
    for (long i = 0; i < k; ++i) 
    {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }
    X=(svd.matrixV())*(singularValues_inv_mat)*(svd.matrixU().transpose());//X=VS+U*
 
    return X; 
}

/**并联机构正运动学解算子函数**/

Eigen::Vector3f MOTOR_KINE::inverse_solu(Eigen::Matrix3f rotm, float top_z, Eigen::Matrix3f& xzy, Eigen::Vector3f& xyz_v)
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

Eigen::Matrix3f MOTOR_KINE::para_jacobian(Eigen::Matrix3f rotm, Eigen::Vector3f length, Eigen::Matrix3f xyz)
{
    double factor_1,factor_2,factor_3,factor_4,factor_5;
    double alpha,beta,gamma;
    Eigen::Vector3f eular;
    Eigen::Matrix3f r,n,r_n_cross,jac_part;
    Eigen::MatrixXf jac1_transposed(6,3);  // 转置矩阵
    Eigen::MatrixXf jac1(3,6);
    Eigen::MatrixXf jac2(6,6);
    Eigen::MatrixXf jac3(6,3);

    eular = rotm2Eul(rotm.col(2));
    alpha = eular(0);
    beta  = eular(1);
    gamma = eular(2);

    r = rotm * para_fram_base_;
    n.col(0) = xyz.col(0) / length(0);
    n.col(1) = xyz.col(1) / length(1);
    n.col(2) = xyz.col(2) / length(2);
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
    
    jac3 << 0,  
                0.5*MOTORPLAT_RADIUS*( factor_2*(cos(beta)+cos(alpha)) + factor_3*cos(beta)*(1+cos(alpha)*cos(alpha)) ) / factor_1,  
                                                    0.5*MOTORPLAT_RADIUS*( 2*factor_4*cos(beta) - factor_5*(factor_1 + sin(alpha)*sin(alpha)) - factor_4*cos(alpha)*sin(beta)*sin(beta) ) / factor_1,
            0,                                                                     -MOTORPLAT_RADIUS*factor_5*cos(beta) / factor_1,        -MOTORPLAT_RADIUS*( factor_3*factor_1 + factor_2 ) / factor_1,
            1,                                                                                                                  0,                                                                   0,
            0,                                                                                                                  1,                                                                   0,
            0,                                                                                                                  0,                                                                   1,
            0,                                                                                              -sin(beta) / factor_1,                                              -sin(alpha) / factor_1;
    
    jac_part = (jac1 * jac2 * jac3 ).inverse();
    return jac_part;
}

Eigen::Matrix3f MOTOR_KINE::para_forward(PARA_SOLU* para)
{
    Eigen::Vector3f temp_length, error_length, rotm_vec, temp_eular,delta_eular,result_jacobian,delta_length,temp_xyzv;
    Eigen::Matrix3f temp_jacobian,temp_xyz,temp_rotm;
    int loop_count = 0;
    bool forward_finished = 0;
    float temp_z;

    const Eigen::Vector3f target_length = para->motor_dist[CURRENT];
    /* parse data */
    temp_length   = para->motor_dist[LAST];
    temp_xyz      = para->xyz;
    temp_rotm     = para->rot_matrix[LAST];
    
    temp_eular = rotm2Eul(temp_rotm.col(2));
    error_length = target_length - temp_length;
    /* loop */
    temp_jacobian = para_jacobian(temp_rotm, temp_length, temp_xyz);
    while(error_length.array().abs().maxCoeff() >= 0.000001)
    {
        loop_count++;
        /* start forward */
        result_jacobian = temp_jacobian * error_length;
        delta_eular(0) = result_jacobian(1);
        delta_eular(1) = result_jacobian(2);
        delta_eular(2) = -atan( (sin(delta_eular(0))*sin(delta_eular(1))) / (cos(delta_eular(0))+cos(delta_eular(1))) );
        temp_eular = temp_eular + delta_eular;
        eul2Rotm(temp_eular,temp_rotm);
        temp_z = temp_xyz.row(2).sum()/3 + result_jacobian(0);
        temp_length = inverse_solu(temp_rotm,temp_z,temp_xyz,temp_xyzv);
        /* whether finished */
        error_length = target_length - temp_length;
        if(loop_count>=100)
        {
            para->state = 0;
            ROS_ERROR("Para robot forward error, too many loops!");
            break;
        }
    }
    ROS_INFO("Forward finish, loop count is:%d",loop_count);
    std::cout << "temp jacobian is \r\n" << temp_jacobian<<std::endl;
    para->jacobian = temp_jacobian;
    para->xyz = temp_xyz;
    para->xyz_v = temp_xyzv;
    return temp_rotm;
}

Eigen::MatrixXd MOTOR_KINE::getJacobian(void)
{
    Eigen::MatrixXd m;
    current_state_ = move_group_.getCurrentState();
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    current_state_->getJacobian(joint_model_group_,
                                current_state_->getLinkModel(joint_model_group_->getLinkModelNames().back()),
                                reference_point_position, m);
    return m;
}

Eigen::VectorXd MOTOR_KINE::perform_func(Eigen::VectorXd joint, float k1, float k2)
{
    double distance = 0;
    double t,k;
    Eigen::VectorXd diffH;

    diffH.resize(9);
    for (size_t i = UR_JOINT_NUM; i < ALL_JOINT_NUM; i++)
    {
        distance = distance + fabs(joint(i) - joint_mean_(i));
    }

    t = distance /DIS_RANGE;
    k = 1 + ((0.1-1)*t*t*(3-2*t));
    for (size_t i = 0; i < ALL_JOINT_NUM; i++)
    {
        if(i < UR_JOINT_NUM)
        {
            diffH(i) = k1*2*(joint(i)-joint_mean_(i))/(pow((joint_mean_(i) - joint_mini_(i)),2));
        }
        else
        {
            diffH(i) = k*k2*2*(joint(i)-joint_mean_(i))/(pow((joint_mean_(i) - joint_mini_(i)),2));
        }
    }
    return diffH;
}

/* struct init */
void MOTOR_KINE::robot_state_init(PARA_SOLU* para, UR5_SOLU* ur)
{ 
    para->rot_matrix[CURRENT] = Eigen::Matrix3f::Identity(3,3);
    para->rot_matrix[NEXT] = Eigen::Matrix3f::Identity(3,3);
    para->laser_cor_z[CURRENT] = LASER_DIST_INIT;
    para->motor_cor_z[CURRENT] = PARA_LEN_MEAN;
    para->laser_dist << LASER_DIST_INIT,LASER_DIST_INIT,LASER_DIST_INIT;
    para->motor_dist[NEXT] << PARA_LEN_MEAN , PARA_LEN_MEAN ,PARA_LEN_MEAN;
    para->motor_dist[CURRENT] = para->motor_dist[NEXT];
    // para->xyz << 0,0,0,0,0,0,PARA_LEN_MEAN , PARA_LEN_MEAN ,PARA_LEN_MEAN;
    para->xyz << 0,0,0,0,0,0,PARA_LEN_MEAN , PARA_LEN_MEAN ,PARA_LEN_MEAN;

    para->state = 1;
    ur->state = 1;
    recored_z_ = 0;

    para_fram_base_ << MOTORPLAT_RADIUS,               -MOTORPLAT_RADIUS/2,          -MOTORPLAT_RADIUS/2,
                                      0,        sqrt(3)*MOTORPLAT_RADIUS/2,  -sqrt(3)*MOTORPLAT_RADIUS/2,
                                      0,                                 0,                            0;
    
    joint_mean_.resize(9);
    joint_mini_.resize(9);
    joint_mean_ << UR_JNT_MEAN,UR_JNT_MEAN,UR_JNT_MEAN,UR_JNT_MEAN,UR_JNT_MEAN,UR_JNT_MEAN,PARA_LEN_MEAN,PARA_LEN_MEAN,PARA_LEN_MEAN;
    joint_mini_ << MIN_UR_JOINT,MIN_UR_JOINT,MIN_UR_JOINT,MIN_UR_JOINT,MIN_UR_JOINT,MIN_UR_JOINT,MIN_PARA_LEN,MIN_PARA_LEN,MIN_PARA_LEN;    

    rot_matrix_d_ = Eigen::Matrix3f::Identity(3,3);
}

/* struct update */
void MOTOR_KINE::para_plat_update(PARA_SOLU* para)
{
    para->rot_matrix[LAST] = para->rot_matrix[CURRENT];
    para->laser_cor_z[LAST] = para->laser_cor_z[CURRENT];
    para->motor_cor_z[LAST] = para->motor_cor_z[CURRENT];
    para->motor_dist[LAST] = para->motor_dist[CURRENT];
    para->motor_dist[CURRENT] = para->motor_dist[NEXT]; 
}


#ifdef DEBUG
void MOTOR_KINE::test_func(void)
{
    Eigen::Matrix<double,6,9> A;
    Eigen::MatrixXd B;

       A << 1,1,1,1,1,1,1,1,1,
            1,1,1,1,1,1,1,1,1,
            1,1,1,1,1,1,1,1,1,
            1,1,1,1,1,1,1,1,1,
            1,1,1,1,1,1,1,1,1,
            1,1,1,1,1,1,1,1,1;       

    B = pinv(A);
    std::cout<< B <<std::endl;
}

#endif

/*  fundamental math algorithm  */
float MOTOR_KINE::max_error(Eigen::Vector3f vector)
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