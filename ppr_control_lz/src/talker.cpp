#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <sstream>
#include <iostream>
#include <cmath>
#include "eigen3/Eigen/Dense"
// #include <Eigen/Dense>
#include "eigen3/Eigen/LU"  
#include "eigen3/Eigen/Core"  
#include <time.h>
#include <unistd.h>
#include <sensor_msgs/LaserScan.h>
#define PI 3.1415926

/*
//计算旋转角
double calculateAngle(const Eigen::Vector3d &vectorBefore, const Eigen::Vector3d &vectorAfter)
{
    double ab, a1, b1, cosr;
    ab = vectorBefore.x()*vectorAfter.x() + vectorBefore.y()*vectorAfter.y() + vectorBefore.z()*vectorAfter.z();
    a1 = sqrt(vectorBefore.x()*vectorBefore.x() + vectorBefore.y()*vectorBefore.y() + vectorBefore.z()*vectorBefore.z());
    b1 = sqrt(vectorAfter.x()*vectorAfter.x() + vectorAfter.y()*vectorAfter.y() + vectorAfter.z()*vectorAfter.z());
    cosr = ab / a1 / b1;
    return (acos(cosr) * 180 / PI);
}
//计算旋转轴
inline Eigen::Vector3d calculateRotAxis(const Eigen::Vector3d &vectorBefore, const Eigen::Vector3d &vectorAfter)
{
    return Eigen::Vector3d(vectorBefore.y()*vectorAfter.z() - vectorBefore.z()*vectorAfter.y(), \
        vectorBefore.z()*vectorAfter.y() - vectorBefore.x()*vectorAfter.z(), \
        vectorBefore.x()*vectorAfter.y() - vectorBefore.y()*vectorAfter.x());
}
//计算旋转矩阵
void rotationMatrix(const Eigen::Vector3d &vectorBefore, const Eigen::Vector3d &vectorAfter, Eigen::Matrix3d &rotMatrix)
{
    Eigen::Vector3d vector = calculateRotAxis(vectorBefore, vectorAfter);
    double angle = calculateAngle(vectorBefore, vectorAfter);
    std::cout << "angle\n" <<  angle << std::endl;
    Eigen::AngleAxisd rotationVector(angle, vector.normalized());
    Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
    rotMatrix =  rotationVector.toRotationMatrix();//所求旋转矩阵
}
*/

class kin_alg
{
  public:
      kin_alg();
      void registerNodeHandle(ros::NodeHandle& _n,ros::NodeHandle& _n1,ros::NodeHandle& _n2,ros::NodeHandle& _n3);
      void register2paras(Eigen::Matrix3d& _LA, Eigen::Matrix3d& _SB);
      void registerPubSub();
      void chatter_Callback1(const sensor_msgs::LaserScan::ConstPtr &msg );
      void chatter_Callback2(const sensor_msgs::LaserScan::ConstPtr &msg );
      void chatter_Callback3(const sensor_msgs::LaserScan::ConstPtr &msg );
      void calcul();
  private:
      ros::NodeHandle n, n1, n2, n3;
      ros::Subscriber sub1, sub2, sub3;
      ros::Publisher chatter1_pub, chatter2_pub, chatter3_pub;
      float laser1, laser2, laser3;
      // float laser1Before, laser2Before, laser3Before;
      float target_dist[3];
      Eigen::Matrix3d LA; 
      Eigen::Matrix3d SB; 
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ppr_controller_lz");

    ros::NodeHandle n, n1, n2, n3;

    Eigen::Matrix3d LA = Eigen::Matrix3d::Zero(); 
    Eigen::Matrix3d SB;
    kin_alg pubSub1;

  SB << -0.06,             0.030087,                0.029913, 
          -0.0001,               -0.051912,           0.052012, 
             0.0,                         0.0,                           0.0;

    pubSub1.registerNodeHandle(n, n1, n2, n3);

    pubSub1.registerPubSub();

    pubSub1.register2paras(LA, SB);

    ros::Rate sr(10);
    
    while (ros::ok())
    {
    ros::spinOnce();
    // sleep(0.1);
    pubSub1.calcul( );
        
    sr.sleep();
    }

}

kin_alg::kin_alg(){};

void kin_alg::registerNodeHandle(ros::NodeHandle& _n,ros::NodeHandle& _n1,ros::NodeHandle& _n2,ros::NodeHandle& _n3)
{
  n = _n;
  n1 = _n1;
  n2 = _n2;
  n3 = _n3;
}

void kin_alg::register2paras(Eigen::Matrix3d& _LA, Eigen::Matrix3d& _SB  )
{
  _LA  << 0.0,                                  0.0,                             0.0, 
                  0.0,                                 0.0,                               0.0, 
                  0.1069,             0.1069,                   0.1069;
  LA = _LA;
_SB << -0.06,             0.030087,                0.029913, 
          -0.0001,               -0.051912,           0.052012, 
             0.0,                         0.0,                           0.0;
SB = _SB;
}

void kin_alg::registerPubSub()
{
  chatter1_pub = n.advertise<std_msgs::Float64>("/parabot/joint1_position_controller/command", 1);
  chatter2_pub = n.advertise<std_msgs::Float64>("/parabot/joint2_position_controller/command", 1);
  chatter3_pub = n.advertise<std_msgs::Float64>("/parabot/joint3_position_controller/command", 1);

   sub1 = n1.subscribe("/parabot/laser1", 1, &kin_alg::chatter_Callback1, this);
   sub2 = n2.subscribe("/parabot/laser2", 1, &kin_alg::chatter_Callback2, this);
   sub3 = n3.subscribe("/parabot/laser3", 1, &kin_alg::chatter_Callback3, this);
}

void kin_alg::chatter_Callback1(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  laser1 = msg->ranges[0];
}

void kin_alg::chatter_Callback2(const sensor_msgs::LaserScan::ConstPtr &msg )
{
  laser2 = msg->ranges[0];
}

void kin_alg::chatter_Callback3(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  laser3 = msg->ranges[0];
}

void kin_alg::calcul( )
{
  Eigen::MatrixXf point1(1,3);
  Eigen::MatrixXf point2(1,3);
  Eigen::MatrixXf point3(1,3);
  Eigen::MatrixXf vector12(1,3);
  Eigen::MatrixXf vector13(1,3);
  Eigen::MatrixXf normal(1,3);
  Eigen::MatrixXf euler(1,3);

    Eigen::Matrix3d RA;
    Eigen::Matrix3d SB_after;
    Eigen::Matrix3d PAB;

    Eigen::Matrix3d rotMatrix;
    int flag_keep_state = 0;
    float Length = 0.0;
    float a, b, c, t1, s1, c1, s2, c2, t3, s3, c3;

    float delta_length[3] = {0.0, 0.0, 0.0};


// 当检测到laser数值相差小于2mm时，应使得delta_length保持不变
 if ((abs(laser1 - laser2) < 0.002) && (abs(laser2 - laser3) < 0.002) && (abs(laser3 - laser1) < 0.002))  
 {
flag_keep_state = 1;
std::cout << "已经和墙面保持平行！！！！" << std::endl;
 } 
// 设置标签，当上平台已经保持与墙面平行时，无需再进行计算
 if (!flag_keep_state)
 {
 std::cout << "laser: \n" << laser1 << " " << laser2 << " " << laser3  << std::endl;

// 对应传感器颜色为红，蓝，绿，分别为laser1，laser2，laser3！！！！！！
  point1 << -0.1175, 0, laser1; // 存在初始位置偏差
  point2 << 0.05875, -0.101758, laser2;
  point3 << 0.05875, 0.101758, laser3;

  vector12 = point2 - point1;
  vector13 = point3 - point1;
// normal(0,0) = vector13(0,1) * vector12(0,2) - vector13(0,2) * vector12(0,1); // 法向量x坐标
// normal(0,1) = -vector13(0,0) * vector12(0,2) + vector13(0,2) * vector12(0,0); // 法向量y坐标
// normal(0,2) = vector13(0,0) * vector12(0,1) - vector13(0,1) * vector12(0,0); // 法向量z坐标

normal(0,0) = vector12(0,1) * vector13(0,2) - vector12(0,2) * vector13(0,1); // 法向量x坐标
normal(0,1) = -vector12(0,0) * vector13(0,2) + vector12(0,2) * vector13(0,0); // 法向量y坐标
normal(0,2) = vector12(0,0) * vector13(0,1) - vector12(0,1) * vector13(0,0); // 法向量z坐标

Length = sqrt (pow (normal(0,0),2) + pow(normal(0,1),2) + pow (normal(0,2) ,2));

a = normal(0,0) / Length;
b = normal(0,1) / Length;
c = normal(0,2) / Length;

// std::cout << "normal\n" << normal << std::endl;

// 采用欧拉角进行计算得到的方法
t1 = -b / c;
c1 = 1/(sqrt(pow(t1,2) + 1));
s1 = t1/(sqrt(pow(t1,2) + 1));

s2 = a;
c2 = sqrt(1-pow(s2,2));

t3 = -s1*s2/(c1*c2);
s3 = t3/(sqrt(pow(t3,2) + 1));
c3 = 1/(sqrt(pow(t3,2) + 1));

t3 = 0.0;
s3 = 0.0;
c3 = 1.0;

// Eigen::Matrix3d rotMatrix ;
 // 旋转矩阵, 绕xyz轴，右乘来绕运动坐标系进行运动

 rotMatrix <<             c2*c3,             -c2*s3,     s2,
               s1*s2*c3+c1*s3,  -s1*s2*s3 + c1*c3, -s1*c2,
            -c1*s2*c3 + s1*s3,   c1*s2*s3 + s1*c3,  c1*c2;

euler(0,0) = -atan(t1); //俯仰角
euler(0, 1) = asin(s2); //偏航角
euler(0, 2) = 0.0; //翻滚角

// std::cout << "euler:\n" <<euler << std::endl;

  RA << -0.06,             0.030087,                0.029913, 
          -0.0001,               -0.051912,           0.052012, 
             0.0,                         0.0,                           0.0;
    // RA << -0.06, 0.030087, 0.029913, -0.0001, -0.051912, 0.052012, 0.0, 0.0, 0.0;


  // SB = RA;  //即使后面的SB更新了，但由于此语句，导致SB永远等于RA
                                                        

PAB << 0.5*0.06*(cos(euler(0,1))*cos(euler (0,2)) + sin(euler (0,0))*sin(euler (0,1))*sin(euler(0,2))-cos(euler(0,0))*cos(euler(0,2))), 
              0.5*0.06*(cos(euler(0,1))*cos(euler (0,2)) + sin(euler (0,0))*sin(euler (0,1))*sin(euler(0,2))-cos(euler(0,0))*cos(euler(0,2))), 
              0.5*0.06*(cos(euler(0,1))*cos(euler (0,2)) + sin(euler (0,0))*sin(euler (0,1))*sin(euler(0,2))-cos(euler(0,0))*cos(euler(0,2))), 
              0.06*cos(euler(0,1))*sin(euler(0,2)),         0.06*cos(euler(0,1))*sin(euler(0,2)),         0.06*cos(euler(0,1))*sin(euler(0,2)), 
              0.1069 ,                                                             0.1069 ,                                                              0.1069;   
  // 不用加上delta_length

// SB_after =rotMatrix * SB;

std::cout << "SB:\n" << SB  << std::endl;

std::cout << "rotMatrix:\n" << rotMatrix << std::endl;

SB_after =rotMatrix *  SB;

// std::cout << "rotMatrixBefore:\n" << rotMatrixBefore << std::endl;

// rotMatrixBefore = rotMatrix;

std::cout << "PAB:\n" << PAB  << std::endl;

LA = SB_after + PAB - RA; // 驱动杆矢量在固定坐标系下的表示，是目标位置

std::cout << "LA:\n" << LA << std::endl;

SB = SB_after;

 }

std::cout << "target_dist:" << std::endl;

for(int i = 0; i<3; i++)
{
    target_dist[i] = sqrt(pow(LA(0, i), 2) + pow(LA(1, i), 2) + pow(LA(2, i), 2));
     std::cout << target_dist[i] << " ";
}

  // std::cout << "\ndelta_length: " << std::endl;


   for (int i = 0; i < 3; i++)
  {
      delta_length[i] = target_dist[i] - 0.1069;
      std::cout << "\ndelta_length:" << std::endl;
       std::cout<< delta_length[i]  << " ";
  }

  std_msgs::Float64 msg1;
  std_msgs::Float64 msg2;
  std_msgs::Float64 msg3;
// 计算得到上平台最大倾斜角度为17.458°
if (delta_length[0] < 0.027 && delta_length[0] > 0.0 && delta_length[1] < 0.027 && delta_length[1] > 0.0 && delta_length[2] < 0.027 && delta_length[2] > 0.0)
{
    msg1.data = delta_length[0];
    msg2.data = delta_length[1];
    msg3.data = delta_length[2];

    chatter1_pub.publish(msg1);
    chatter2_pub.publish(msg2);
    chatter3_pub.publish(msg3);
}

      std::cout << "\n________________________________"  << std::endl;
}








