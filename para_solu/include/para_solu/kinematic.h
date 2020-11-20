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
    
    Eigen::Matrix3d KDLR2EigenR(KDL::Rotation R);
    Eigen::Matrix4d KDLR2EigenT(KDL::Rotation R);
    Eigen::Matrix4d D2EigenT(double* T);
    Eigen::Matrix<double,6,6> Jacobian(const double* q,const Eigen::Matrix4d T_Base);
    
} // namespace ur_solu




#endif