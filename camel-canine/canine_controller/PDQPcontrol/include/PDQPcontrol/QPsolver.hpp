//
// Created by hs on 22. 11. 9.
//

#ifndef QPSOLVER_HPP
#define QPSOLVER_HPP

#include <iostream>

#include <qpOASES.hpp>
#include <unsupported/Eigen/MatrixFunctions>

#include <canine_util/EigenTypes.hpp>
#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotMath.hpp>

using Eigen::Dynamic;

class PDQPsolver{
public:
    PDQPsolver();
    ~PDQPsolver();

    void SolveQP(const Vec13<double>& x0, const Vec13<double>& xd, const double mFoot[4][3]);
    void GetGRF(Vec3<double>* f);
private:
    void setConstraints();
    void transformMat2Real(qpOASES::real_t* dst, Eigen::Matrix<double,Dynamic,Dynamic> src, int16_t rows, int16_t cols);
    void solveQP();
private:
    const double mAlpha;
    const double mForceMax;
    const double mMu;
    const double mMass;
    const double mBigNum;

    Vec3<double> mDesiredLinearAccel;
    Vec3<double> mDesiredAngularAccel;
    Vec3<double> mKpLinear, mKdLinear;
    Vec3<double> mKpAnguler, mKdAngular;
    Vec3<double> gravity;

    Eigen::Matrix<double,6,6> S;
    Eigen::Matrix<double,12,12> W;
    Eigen::Matrix<double,6,12> A;
    Eigen::Matrix<double,12,1> f;
    Eigen::Matrix<double,6,1> b;

    Eigen::Matrix<double,12,12> H;
    Eigen::Matrix<double,12,1> g;

    Eigen::Matrix<double,3,3> unit;

    Mat3<double> mBodyInertia;

    Eigen::Matrix<double,6,1> mSweight;
    Eigen::Matrix<double,3,1> mWweight;

    Eigen::Matrix<double,20,1> U_b;
    Eigen::Matrix<double,20,1> L_b;
    Eigen::Matrix<double,20,12> fmat;

    qpOASES::real_t* H_qpoases;
    qpOASES::real_t* g_qpoases;
    qpOASES::real_t* A_qpoases;
    qpOASES::real_t* ub_qpoases;
    qpOASES::real_t* lb_qpoases;
    qpOASES::real_t* q_soln;
};

#endif //QPSOLVER_HPP
