#include "para_solu/kinematic.h"

const double d_1 =  0.089159;
const double a_2 = -0.42500;
const double a_3 = -0.39225;
const double d_4 =  0.10915;
const double d_5 =  0.09465;
const double d_6 =  0.0823;

namespace macmic_kinematic
{

    Eigen::Matrix3d EigenT2EigenR(Eigen::Matrix4d T)
    {
        Eigen::Matrix3d R;
        for (size_t i = 0; i < 3; i++)
        {
            for (size_t j = 0; j < 3; j++)
            {
                R(i,j)=T(i,j);
            }
        }
        return R;
    }

    Eigen::Matrix3d KDLR2EigenR(KDL::Rotation R)
    {
        Eigen::Matrix3d R_eigen;

            for (int i = 0; i < 3; i ++)
            {
                for (int j = 0; j < 3; j ++)
                {
                    (R_eigen)(i,j) = R.data[i*3+j];
                }
            }
        return R_eigen;
    }

    Eigen::Matrix4d KDLR2EigenT(KDL::Rotation R)
    {
        Eigen::Matrix4d R_eigen = Eigen::Matrix4d::Identity();

        for (int i = 0; i < 3; i ++)
        {
            for (int j = 0; j < 3; j ++)
            {
                (R_eigen)(i,j) = R.data[i*3+j];
            }
        }
            
            
        return R_eigen;
    }

    Eigen::Matrix4d D2EigenT(double* T)
    {
        Eigen::Matrix4d R_eigen = Eigen::Matrix4d::Identity();

        for (int i = 0; i < 4; i ++)
        {
            for (int j = 0; j < 4; j ++)
            {
                (R_eigen)(i,j) = T[i*4+j];
            }
        }
        return R_eigen;
    }

    Eigen::Matrix4d ur_forward(const double* q,const Eigen::Matrix4d T_Base)
    {
        double* T1 = new double[16];
        double* T2 = new double[16];
        double* T3 = new double[16];
        double* T4 = new double[16];
        double* T5 = new double[16];
        double* T6 = new double[16];
        forward_all(q, T1, T2, T3, T4, T5, T6);

        return  D2EigenT(T6);
    }

    Eigen::Matrix<double,6,6> Jacobian(const double* q,const Eigen::Matrix4d T_Base, const Eigen::Vector3d T_tool)
    {
        double* T1 = new double[16];
        double* T2 = new double[16];
        double* T3 = new double[16];
        double* T4 = new double[16];
        double* T5 = new double[16];
        double* T6 = new double[16];

        forward_all(q, T1, T2, T3, T4, T5, T6);

        Eigen::Matrix<double,6,6> Jacobian;
        Eigen::Matrix4d T_1 = D2EigenT(T1);
        Eigen::Matrix4d T_2 = D2EigenT(T2);
        Eigen::Matrix4d T_3 = D2EigenT(T3);
        Eigen::Matrix4d T_4 = D2EigenT(T4);
        Eigen::Matrix4d T_5 = D2EigenT(T5);
        Eigen::Matrix4d T_6 = D2EigenT(T6);

        // T_1 = T_Base * T_1 ;
        // T_2 = T_Base * T_2 ;
        // T_3 = T_Base * T_3 ;
        // T_4 = T_Base * T_4 ;
        // T_5 = T_Base * T_5 ;
        // T_6 = T_Base * T_6 * T_tool;
        // T_6 = T_6 * T_tool;
        // std::cout << " after T transfer \r\n" << T_6.col(3).head(3) << std::endl;

        // Eigen::Vector3d pe(T_6(0,3), T_6(1,3), T_6(2,3));
        // std::cout << " before T transfer \r\n" << pe << std::endl;
        Eigen::Vector4d V_tool;
        V_tool << T_tool, 1;
        Eigen::Vector3d pe = (T_6 * V_tool).head(3);
        // std::cout << "after T transfer \r\n" << pe << std::endl;
        Eigen::Vector3d jpi(0.0, 0.0, 0.0);

        // J1
        Eigen::Matrix4d T0 = Eigen::Matrix4d::Identity();
        // T0 = T_Base * T0;
        Eigen::Vector3d z0(T0(0,2),T0(1,2),T0(2,2));
        Eigen::Vector3d p0(0.0, 0.0, 0.0);
        Eigen::Vector3d pe_minus_p0(pe - p0);

        jpi = z0.cross(pe_minus_p0);
        Eigen::Matrix<double,6,1> J1;
        J1 << jpi , z0;

        // printf("pe_minus_p0 = [%1.4f, %1.4f, %1.4f] \n", pe_minus_p0[0], pe_minus_p0[1], pe_minus_p0[2]);

        // J2
        Eigen::Vector3d z1(T_1(0,2), T_1(1,2), T_1(2,2));
        Eigen::Vector3d p1(T_1(0,3), T_1(1,3), T_1(2,3));
        Eigen::Vector3d pe_minus_p1(pe - p1);

        jpi = z1.cross(pe_minus_p1);
        Eigen::Matrix<double,6,1> J2;
        J2 << jpi, z1;
        // J3
        Eigen::Vector3d z2(T_2(0,2), T_2(1,2), T_2(2,2));
        Eigen::Vector3d p2(T_2(0,3), T_2(1,3), T_2(2,3));
        Eigen::Vector3d pe_minus_p2(pe - p2);

        jpi = z2.cross(pe_minus_p2);
        Eigen::Matrix<double,6,1> J3;
        J3 << jpi, z2;

        // J4
        Eigen::Vector3d z3(T_3(0,2), T_3(1,2), T_3(2,2));
        Eigen::Vector3d p3(T_3(0,3), T_3(1,3), T_3(2,3));
        Eigen::Vector3d pe_minus_p3(pe - p3);

        jpi = z3.cross(pe_minus_p3);
        Eigen::Matrix<double,6,1> J4;
        J4 << jpi, z3;

        // J5
        Eigen::Vector3d z4(T_4(0,2), T_4(1,2), T_4(2,2));
        Eigen::Vector3d p4(T_4(0,3), T_4(1,3), T_4(2,3));
        Eigen::Vector3d pe_minus_p4(pe - p4);

        jpi = z4.cross(pe_minus_p4);
        Eigen::Matrix<double,6,1> J5;
        J5 << jpi, z4;

        // J6
        Eigen::Vector3d z5(T_5(0,2), T_5(1,2), T_5(2,2));
        Eigen::Vector3d p5(T_5(0,3), T_5(1,3), T_5(2,3));
        Eigen::Vector3d pe_minus_p5(pe - p5);

        jpi = z5.cross(pe_minus_p5);
        Eigen::Matrix<double,6,1> J6;
        J6 << jpi, z5;

        // printf("T1 x y z = [%1.4f, %1.4f, %1.4f] \n", p1[0], p1[1], p1[2]);
        // printf("T2 x y z = [%1.4f, %1.4f, %1.4f] \n", p2[0], p2[1], p2[2]);
        // printf("T3 x y z = [%1.4f, %1.4f, %1.4f] \n", p3[0], p3[1], p3[2]);
        // printf("T4 x y z = [%1.4f, %1.4f, %1.4f] \n", p4[0], p4[1], p4[2]);
        // printf("T5 x y z = [%1.4f, %1.4f, %1.4f] \n", p5[0], p5[1], p5[2]);
        // printf("T6 x y z = [%1.4f, %1.4f, %1.4f] \n", T6[3], T6[3+4], T6[3+8]);

        // for (int i = 0; i < 6; i ++)
        // {
        //     J[i*6 + 0] = J1[i];
        //     J[i*6 + 1] = J2[i];
        //     J[i*6 + 2] = J3[i];
        //     J[i*6 + 3] = J4[i];
        //     J[i*6 + 4] = J5[i];
        //     J[i*6 + 5] = J6[i];
        // }
        Jacobian << J1, J2, J3, J4, J5, J6;
        Eigen::Matrix<double,6,6> R_ur_forward;
        R_ur_forward << EigenT2EigenR(T_Base) , Eigen::Matrix3d::Zero(),
                        Eigen::Matrix3d::Zero(),EigenT2EigenR(T_Base) ;

        // Eigen::Matrix<double,6,6> R_tool_forward1;
        // R_ur_forward << KDLR2EigenR(KDL::Rotation::RotX(KDL::PI)) , Eigen::Matrix3d::Zero(),
        //             Eigen::Matrix3d::Zero(), KDLR2EigenR(KDL::Rotation::RotX(KDL::PI)) ;
        // Eigen::Matrix<double,6,6> R_ur_forward;
        // R_ur_forward << EigenT2EigenR(T_Base) * KDLR2EigenR(KDL::Rotation::RotX(KDL::PI/2)), Eigen::Matrix3d::Zero(),
        //             Eigen::Matrix3d::Zero(), EigenT2EigenR(T_Base) * KDLR2EigenR(KDL::Rotation::RotX(KDL::PI/2));

        return R_ur_forward  *Jacobian ;
    }

    KDL::Rotation EigenT2KDLR(Eigen::Matrix4d T)
    {
        KDL::Rotation R;
        for (int i = 0; i < 3; i ++)
        {
            for (int j = 0; j < 3; j ++)
            {
                R.data[i*3+j] = (T)(i,j);
            }
        }
        return R;
    }

    void EigenT2Eular(Eigen::Matrix4d T,double* q)
    {
        KDL::Rotation R;
        double qt[3];
        R = EigenT2KDLR(T);
        R.GetRPY(qt[0],qt[1],qt[2]);
        q[1] = qt[1]; q[0] = qt[0]; q[2] = qt[2];
    }

    Vector6d EigenT2Pos(Eigen::Matrix4d T)
    {
        double q[6];
        for (size_t i = 0; i < 3; i++)
        {
            q[i] = T(i,3);
        }
        EigenT2Eular(T,&q[3]);
        return D2V6(q);
    }

    Eigen::Vector4d EigenT2Qua(Eigen::Matrix4d T)
    {
        double q[4];
        KDL::Rotation R;
        R = EigenT2KDLR(T);
        R.GetQuaternion(q[0],q[1],q[2],q[3]);

        return Eigen::Vector4d(q);
    }


    Vector6d D2V6(double *q)
    {
        Vector6d V;
        for (size_t i = 0; i < 6; i++)
        {
            V(i) = q[i];
        }
        return V;
    }
    
    Eigen::Matrix<double,6,9> rdJacobian(const Eigen::Matrix<double,6,6> ur_j, const Eigen::Matrix<double,6,3> ppr_j, const Eigen::Matrix4d T_b2ur)
    {
        Eigen::Matrix<double,6,9> rdjacobian;
        Eigen::Matrix3d R_b2ur = EigenT2EigenR(T_b2ur);
        Eigen::Matrix<double,6,6> M_b2ur;
        M_b2ur << R_b2ur , Eigen::Matrix3d::Zero(),
                  Eigen::Matrix3d::Zero(), R_b2ur;

        rdjacobian << ur_j , M_b2ur * ppr_j;
        
        return rdjacobian;
    }

    Vector9d graProjVector(const Eigen::Vector3d ppr_stat, const float dist_mid, const float dist_max)
    {
        Vector9d phi;

        Vector6d Zeros;
        Zeros = Vector6d::Zero();

        Eigen::Matrix3d K;
        K = -0.05 * Eigen::Matrix3d::Identity();

        Eigen::Vector3d pH;
        for (size_t i = 0; i < 3; i++)
        {
            (ppr_stat(i) - dist_mid) / pow((dist_mid - dist_max),2);
        }

        phi << Zeros , K * pH;
        return phi;
    }

    Eigen::Matrix<double,6,6> EigenT2M6(Eigen::Matrix4d T)
    {
        Eigen::Matrix<double,6,6> M;
        Eigen::Matrix3d R;
        R = EigenT2EigenR(T);

        M << R , Eigen::Matrix3d::Zero(),
             Eigen::Matrix3d::Zero(),  R;
        return M;
    }

    Eigen::Matrix<double,6,6> JacoInvSVD(Eigen::Matrix<double,6,6> Jacobian)
    {
        // singularity ?
        double lamda = 10;

        Eigen::JacobiSVD<Eigen::MatrixXd> jacobian_svd(Jacobian,0x28);
        Eigen::VectorXd jacobian_eigenV;
        Eigen::Matrix<double,6,6> jacobian_inverse, j_jt,jacobian_psu, sigama_psudo,jacobian_matrixv;
        sigama_psudo << 0,0,0,0,0,0,
                        0,0,0,0,0,0,
                        0,0,0,0,0,0,
                        0,0,0,0,0,0,
                        0,0,0,0,0,0,
                        0,0,0,0,0,1;
        jacobian_eigenV = jacobian_svd.singularValues();
        jacobian_matrixv = jacobian_svd.matrixV();
        if (jacobian_eigenV(5)>= 0.1)
        {
            jacobian_inverse = Jacobian.inverse();
        }
        else{
            // svd psudo inverse
            std::cout << COUT_RED << "I'm in SVD inverse" << std::endl;
            j_jt = (Jacobian.transpose()*Jacobian + (jacobian_matrixv*(lamda* sigama_psudo)*jacobian_matrixv.transpose()));
            jacobian_psu = j_jt.inverse() * Jacobian.transpose();
            jacobian_inverse = jacobian_psu;
        }
        return jacobian_inverse;
    }

    Eigen::Matrix4d V2EigenT(Vector6d V)
    {
        Eigen::Matrix4d T =  KDLR2EigenT(KDL::Rotation::RPY(V(3),V(4),V(5)));
        T(3,0) = V(0);
        T(3,1) = V(1);
        T(3,2) = V(2);
        return T;
    }
} // namespace ur_solu


LowPassFilter::LowPassFilter(std::vector<double> coeffient):coeffient_(coeffient),order_(coeffient_.size())
{
    data_buffer_.resize(order_);
}

double LowPassFilter::update_filter(double data)
{
    double output = 0;
    data_buffer_.erase(data_buffer_.begin());
    data_buffer_.push_back(data);

    // error detect
    if (data_buffer_.size() != coeffient_.size())
    {
        std::cout << COUT_RED << "There is something wrong in low-pass filter, please check it or reset!" << std::endl;
        return 0;
    }
    
    for (size_t i = 0; i < order_; i++)
    {
        output+= coeffient_[order_-1 -i] * data_buffer_[i];
    }
    return output;
}