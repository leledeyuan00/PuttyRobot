#ifndef __KINEMATIC_H__
#define __KINEMATIC_H__


#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ur_kinematics/ur_kin.h>
#include <kdl/frames_io.hpp>
#include <vector>

#define COUT_RED     "\033[31m"      /* Red */

using namespace ur_kinematics;

namespace macmic_kinematic
{

    typedef Eigen::Matrix<double,6,1> Vector6d;
    typedef Eigen::Matrix<double,9,1> Vector9d;

    typedef enum{
        PPR_LOWER= 0,
        PPR_STANDARD,
        PPR_UPPER ,
    }PPR_INCLINE;

    typedef enum{
        GB_SOFTER = 0,
        GB_STANDARD,
        GB_HARDER,
    }GB_FORCE;
    
    Eigen::Matrix3d EigenT2EigenR(Eigen::Matrix4d T);

    Eigen::Matrix3d KDLR2EigenR(KDL::Rotation R);

    Eigen::Matrix4d KDLR2EigenT(KDL::Rotation R);

    KDL::Rotation EigenT2KDLR(Eigen::Matrix4d T);

    Eigen::Matrix4d D2EigenT(double* T);

    Eigen::Matrix<double,6,6> Jacobian(const double* q,const Eigen::Matrix4d T_Base, const Eigen::Vector3d T_tool);

    void EigenT2Eular(Eigen::Matrix4d T, double* q);

    Vector6d EigenT2Pos(Eigen::Matrix4d T);

    Eigen::Matrix<double,6,6> EigenT2M6(Eigen::Matrix4d T);
    
    Vector6d D2V6(double *q);

    Eigen::Vector4d EigenT2Qua(Eigen::Matrix4d T);

    Eigen::Matrix4d ur_forward(const double* q,const Eigen::Matrix4d T_Base);

    Eigen::Matrix<double,6,9> rdJacobian(const Eigen::Matrix<double,6,6> ur_j, const Eigen::Matrix<double,6,3> ppr_j, const Eigen::Matrix4d T_b2ur);

    Vector9d graProjVector(const Eigen::Vector3d ppr_stat, const float dist_mid, const float dist_max);

    Eigen::Matrix<double,6,6> JacoInvSVD(Eigen::Matrix<double,6,6> Jacobian);

    Eigen::Matrix4d V2EigenT(Vector6d V);
    
    // static const std::vector<double> filter_coefficient_;
    const std::vector<double> filter_coefficient_ = {0.0015,0.0075,0.0217,0.0467,0.0809,0.1175,0.1459,0.1567,0.1459,0.1175,0.0809,0.0467,0.0217,0.0075,0.0015};


    template<class T>
    constexpr const T& clamp( const T& v, const T& lo, const T& hi )
    {
        assert( !(hi < lo) );
        return (v < lo) ? lo : (hi < v) ? hi : v;
    }
} // namespace ur_solu


/**
 * @breif: A FIR lowpass filter
 * @param: 1. std::vector<double> coefficient 
 *         2. size_t order
*/

class LowPassFilter
{
private:
    std::vector<double> data_buffer_;
    std::vector<double> coeffient_;
    size_t order_;
    
public:
    LowPassFilter(std::vector<double> coeffient);
    ~LowPassFilter(){};

    double update_filter(double data);
};


#endif