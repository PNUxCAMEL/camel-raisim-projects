//
// Created by hs on 22. 10. 27.
//

#ifndef RAISIM_QPSOLVER_HPP
#define RAISIM_QPSOLVER_HPP

#include <iostream>

#include <qpOASES.hpp>
#include <unsupported/Eigen/MatrixFunctions>

#include <canine_util/EigenTypes.hpp>
#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotMath.hpp>

using Eigen::Dynamic;

class QPsolver{
public:
    QPsolver();
    ~QPsolver();

    void SolveQP(const Vec13<double>& x0, const Vec13<double>& xd, const double mFoot[4][3]);
    void GetGRF(Vec3<double>* f);
private:
    void setStateSpaceMatrix(const Vec13<double>& x0, const double mFoot[4][3]);
    void setWeights(const Vec13<double>& x0, const Vec13<double>& xd);
    void setConstraints();
    void transformMat2Real(qpOASES::real_t* dst, Eigen::Matrix<double,Dynamic,Dynamic> src, int16_t rows, int16_t cols);
    void solveQP();
private:
    char var_elim[2000];
    char con_elim[2000];

    const double mDt;
    const double mAlpha;
    const double mBigNum;
    const double mForceMax;
    const double mMu;

    Vec13<double> mWeightMat;
    Mat3<double> mYaw;
    Mat3<double> mBodyInertia;
    Mat3<double> mBodyInertiaInverse;

    Eigen::Matrix<double,13,13> Ac;
    Eigen::Matrix<double,13,12> Bc;
    Eigen::Matrix<double,25,25> expmm;
    Eigen::Matrix<double,25,25> ABc;
    Eigen::Matrix<double,13,13> Aqp;
    Eigen::Matrix<double,13,12> Bqp;

    Eigen::Matrix<double,13,13> L;
    Eigen::Matrix<double,12,12> K;
    Eigen::Matrix<double,12,12> H;
    Eigen::Matrix<double,12,1> g;

    Eigen::Matrix<double,20,1> U_b;
    Eigen::Matrix<double,20,12> fmat;

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
};

#endif //RAISIM_QPSOLVER_HPP
