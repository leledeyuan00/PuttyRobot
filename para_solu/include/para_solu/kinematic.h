#ifndef __KINEMATIC_H__
#define __KINEMATIC_H__

#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ur_kinematics/ur_kin.h>
#include <kdl/frames_io.hpp>

using namespace ur_kinematics;

namespace macmic_kinematic
{

    typedef Eigen::Matrix<double,6,1> Vector6d;
    
    Eigen::Matrix3d EigenT2EigenR(Eigen::Matrix4d T);
    Eigen::Matrix3d KDLR2EigenR(KDL::Rotation R);
    Eigen::Matrix4d KDLR2EigenT(KDL::Rotation R);
    KDL::Rotation EigenT2KDLR(Eigen::Matrix4d T);
    Eigen::Matrix4d D2EigenT(double* T);
    Eigen::Matrix<double,6,6> Jacobian(const double* q,const Eigen::Matrix4d T_Base);
    void EigenT2Eular(Eigen::Matrix4d T, double* q);
    Vector6d EigenT2Pos(Eigen::Matrix4d T);
    
    Vector6d D2V6(double *q);

    Eigen::Matrix4d ur_forward(const double* q,const Eigen::Matrix4d T_Base);
    
} // namespace ur_solu




#endif