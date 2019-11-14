#include "pararob/kinematic.h"


/* 解算下一时刻电机杆长 */
void MOTOR_KINE::motor_traj_gen(PARA_SOLU* para)
{
    /* variable */
    Eigen::Vector3f wall_plat_vector;
    Eigen::Matrix3f rot_matrix;
    /* algorithm */
    static int i=0;
    Eigen::Vector3f desired_eula;
 
    para_plat_update(para);
    ROS_INFO("First Motor Rot Matrix is :\r\n [%f, %f, %f] \r\n [%f, %f, %f]\r\n [%f, %f, %f]", para->rot_matrix[CURRENT](0,0),para->rot_matrix[CURRENT](0,1),para->rot_matrix[CURRENT](0,2),para->rot_matrix[CURRENT](1,0),para->rot_matrix[CURRENT](1,1),para->rot_matrix[CURRENT](1,2),para->rot_matrix[CURRENT](2,0),para->rot_matrix[CURRENT](2,1),para->rot_matrix[CURRENT](2,2));
    
    wall_plat_vector = get_wall_plat_vec(para->laser_dist);
    ROS_INFO("Wall platform vector is : [%f, %f, %f]", wall_plat_vector(0), wall_plat_vector(1),wall_plat_vector(2));

    rot_matrix = normal_vec_rotm(wall_plat_vector);
   

    ROS_INFO("Wall Rot Matrix is :\r\n [%f, %f, %f] \r\n [%f, %f, %f]\r\n [%f, %f, %f]", rot_matrix(0,0),rot_matrix(0,1),rot_matrix(0,2),rot_matrix(1,0),rot_matrix(1,1),rot_matrix(1,2),rot_matrix(2,0),rot_matrix(2,1),rot_matrix(2,2));

    para->rot_matrix[CURRENT] =  rot_matrix * para->rot_matrix[LAST];
    
    inverse_solu(para);
    ROS_INFO("Second Motor Rot Matrix is :\r\n [%f, %f, %f] \r\n [%f, %f, %f]\r\n [%f, %f, %f]", para->rot_matrix[CURRENT](0,0),para->rot_matrix[CURRENT](0,1),para->rot_matrix[CURRENT](0,2),para->rot_matrix[CURRENT](1,0),para->rot_matrix[CURRENT](1,1),para->rot_matrix[CURRENT](1,2),para->rot_matrix[CURRENT](2,0),para->rot_matrix[CURRENT](2,1),para->rot_matrix[CURRENT](2,2));
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

/* Rodrigues rotation matrix */
Eigen::Matrix3f MOTOR_KINE::rodrigues(float theta, Eigen::Vector3f vector)
{
    float s,c;
    float kx,ky,kz;
    Eigen::Matrix3f rotm;

    s = sin(theta);
    c = cos(theta);

    kx = vector(0);
    ky = vector(1);
    kz = vector(2);

    rotm <<  c + kx*kx*(1-c),  -s*kz + (1-c)*kx*ky,   s*ky+(1-c)*kx*kz,
            s*kz+(1-c)*kx*ky,        c+ky*ky*(1-c),  -s*kx+(1-c)*ky*kz,
           -s*ky+(1-c)*kx*kz,     s*kx+(1-c)*ky*kz,      c+kz*kz*(1-c);
    return rotm;
}

/**并联机构正运动学解算子函数**/

void MOTOR_KINE::inverse_solu(PARA_SOLU* para)
{
    /* variable */
    Eigen::Matrix3f top_fram, base_fram, link_vectors, link_matrix;
    Eigen::Matrix<float,2,3> element;
    float s1,s2,s3,c1,c2,c3,arfa,beta,gama;
    Eigen::Vector3f desired_eula;

    element = get_rot_element(para->rot_matrix[CURRENT].row(2));
    c1 = element(0,0);
    c2 = element(0,1);
    c3 = element(0,2);
    s1 = element(1,0);
    s2 = element(1,1);
    s3 = element(1,2);
   

    top_fram << MOTORPLAT_RADIUS,               -MOTORPLAT_RADIUS/2,          -MOTORPLAT_RADIUS/2,
                               0,        sqrt(3)*MOTORPLAT_RADIUS/2,  -sqrt(3)*MOTORPLAT_RADIUS/2,
                               0,                                 0,                            0;
    base_fram = top_fram;


    // para->rot_vectors << 0.5*MOTORPLAT_RADIUS*(c2*c3+s1*s2*s3-c1*c3), MOTORPLAT_RADIUS*(c2*s3),(para->motor_dist.sum()/3);
    para->rot_vectors << 0.5*MOTORPLAT_RADIUS*(c2*c3+s1*s2*s3-c1*c3), MOTORPLAT_RADIUS*(c2*s3), 0.12 + MOTOR_LEN_INIT;
    // para->rot_vectors << 0,0,MOTOR_LEN_INIT + 0.12;

    link_matrix << para->rot_vectors,para->rot_vectors,para->rot_vectors;

    link_vectors = para->rot_matrix[CURRENT] * top_fram + link_matrix - base_fram;

    para->motor_dist << sqrt( pow(link_vectors(0,0),2) + pow(link_vectors(1,0),2) + pow(link_vectors(2,0),2)), 
                                      sqrt( pow(link_vectors(0,1),2) + pow(link_vectors(1,1),2) + pow(link_vectors(2,1),2)), 
                                      sqrt( pow(link_vectors(0,2),2) + pow(link_vectors(1,2),2) + pow(link_vectors(2,2),2));
}
/* struct init */
void MOTOR_KINE::para_plat_init(PARA_SOLU* para)
{
    para->rot_matrix[CURRENT] = Eigen::Matrix3f::Identity(3,3);
    para->laser_cor_z[CURRENT] = LASER_DIST_INIT;
    para->motor_cor_z[CURRENT] = MOTOR_LEN_INIT + 0.12;
    // para->motor_dist << para->motor_cor_z[CURRENT],para->motor_cor_z[CURRENT],para->motor_cor_z[CURRENT];
    // para->motor_dist << MOTOR_LEN_INIT+0.175352,MOTOR_LEN_INIT+0.094019,MOTOR_LEN_INIT+0.094019;
    para->rot_matrix [CURRENT] << 1, 0, 0,
                                  0, 1, 0,
                                  0, 0, 1;
}

/* struct update */
void MOTOR_KINE::para_plat_update(PARA_SOLU* para)
{
    para->rot_matrix[LAST] = para->rot_matrix[CURRENT];
    para->laser_cor_z[LAST] = para->laser_cor_z[CURRENT];
    para->motor_cor_z[LAST] = para->motor_cor_z[CURRENT];
}