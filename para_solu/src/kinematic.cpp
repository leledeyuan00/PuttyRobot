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

        return T_Base * D2EigenT(T6);
    }

    Eigen::Matrix<double,6,6> Jacobian(const double* q,const Eigen::Matrix4d T_Base)
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

        T_1 = T_Base * T_1;
        T_2 = T_Base * T_2;
        T_3 = T_Base * T_3;
        T_4 = T_Base * T_4;
        T_5 = T_Base * T_5;
        T_6 = T_Base * T_6;

        Eigen::Vector3d pe(T_6(0,3), T_6(1,3), T_6(2,3));
        Eigen::Vector3d jpi(0.0, 0.0, 0.0);

        // J1
        Eigen::Matrix4d T0 = Eigen::Matrix4d::Identity();
        T0 = T_Base * T0;
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
        return Jacobian;
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
        R = EigenT2KDLR(T);
        R.GetRPY(q[0],q[1],q[2]);
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

    Vector6d D2V6(double *q)
    {
        Vector6d V;
        for (size_t i = 0; i < 6; i++)
        {
            V(i) = q[i];
        }
        return V;
    }
    
} // namespace ur_solu
