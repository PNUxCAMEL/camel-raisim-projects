//
// Created by hs on 22. 8. 16.
//

#ifndef RAISIM_QPSOLVER_H
#define RAISIM_QPSOLVER_H

#include <iostream>

#include <raisim/World.hpp>
#include <unsupported/Eigen/MatrixFunctions>
#include <qpOASES.hpp>

#include <canine_util/EigenTypes.hpp>
#include <GaitScheduler/Gait.hpp>

using Eigen::Dynamic;

class ConvexMPCSolver{
public:
    ~ConvexMPCSolver();

    void InitMatrix(raisim::Mat<3,3> bdyInertia);
    void SetParameters(int Horizon, double Dt);
    void SetWeights(Vec13<double> weight, double alpha);

    void ResizeMatrix();
    void GetMetrices(int *_mpcTable, raisim::VecDyn pos, raisim::VecDyn vel, raisim::Vec<3> footPosition[4]);
    void qpSolver();
    void SetTrajectory(double currentTime, GaitType currentGait);
    void GetJacobian(Eigen::Matrix<double,3,3>& J, double hip, double thigh, double calf, int side);
    void GetGRF(Vec3<double> f[4]);

    Vec4<double> quat;
    Vec3<double> p,v,w,q;

    double stopPosX = 0.0;

private:
    void transformQuat2Euler(const Vec4<double>& quat, Vec3<double>& q);
    void getStateSpaceMatrix(Eigen::Matrix<double,13,13>& Ac, Eigen::Matrix<double,13,12>& Bc, raisim::Vec<3> footPosition[4]);
    void transformC2QP(Eigen::Matrix<double,13,13> A, Eigen::Matrix<double,13,12> B);
    void transformMat2Real(qpOASES::real_t* dst, Eigen::Matrix<double,Dynamic,Dynamic> src, int16_t rows, int16_t cols);

private:
    uint8_t mHorizon;
    double mDt;
    double mAlpha;

    Eigen::Matrix<double,3,3> I_world;
    Eigen::Matrix<double,3,3> I_inv;

    Vec13<double> mWeightMat;

    Eigen::Matrix<double,13,13> Ac;
    Eigen::Matrix<double,13,12> Bc;

    Eigen::Matrix<double, Dynamic, 13> Aqp;
    Eigen::Matrix<double, Dynamic, Dynamic> Bqp;

    Eigen::Matrix<double, 13, 1> x0;
    Eigen::Matrix<double, Dynamic, 1> xd;

    Eigen::Matrix<double, Dynamic, Dynamic> L;
    Eigen::Matrix<double, Dynamic, Dynamic> K;
    Eigen::Matrix<double, Dynamic, Dynamic> H;
    Eigen::Matrix<double, Dynamic, 1> g;

    Eigen::Matrix<double, Dynamic, 1> U_b;
    Eigen::Matrix<double, Dynamic, Dynamic> fmat;

    qpOASES::real_t* H_qpoases;
    qpOASES::real_t* g_qpoases;
    qpOASES::real_t* A_qpoases;
    qpOASES::real_t* ub_qpoases;
    qpOASES::real_t* lb_qpoases;
    qpOASES::real_t* q_soln;

    qpOASES::real_t* H_red;
    qpOASES::real_t* g_red;
    qpOASES::real_t* A_red;
    qpOASES::real_t* lb_red;
    qpOASES::real_t* ub_red;
    qpOASES::real_t* q_red;
    int8_t real_allocated = 0;

    Eigen::Matrix<double,25,25> ABc, expmm;
    Eigen::Matrix<double,13,13> Adt;
    Eigen::Matrix<double,13,12> Bdt;

    Eigen::Matrix<double,3,3> R_yaw;
};
#endif //RAISIM_QPSOLVER_H
