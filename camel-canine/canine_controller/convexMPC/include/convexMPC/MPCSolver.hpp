//
// Created by hs on 22. 8. 16.
//

#ifndef RAISIM_QPSOLVER_H
#define RAISIM_QPSOLVER_H

#include <iostream>

#include <unsupported/Eigen/MatrixFunctions>
#include <qpOASES.hpp>

#include <camel-tools/trajectory.hpp>
#include <canine_util/EigenTypes.hpp>
#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotMath.hpp>

using Eigen::Dynamic;

class MPCSolver{
public:
    MPCSolver(const uint8_t& horizon);
    ~MPCSolver();

    void SetTrajectory(CubicTrajectoryGenerator Trajectory[3], CubicTrajectoryGenerator LegTrajectory[3]);
    void GetMetrices(const Vec13<double>&  x0, const double mFoot[4][3]);
    void SolveQP();
    void GetGRF(Vec3<double> f[4]);

private:
    void initMatrix();
    void resizeMatrix();
    void getStateSpaceMatrix(const Vec13<double>& x0, const double mFoot[4][3]);
    void transformC2QP();

    static void transformMat2Real(qpOASES::real_t* dst, Eigen::Matrix<double,Dynamic,Dynamic> src, int16_t rows, int16_t cols);

private:
    const double mDt;
    const double mAlpha;
    const double mMu;
    const int mFmax;
    char var_elim[2000];
    char con_elim[2000];

    Vec13<double> mWeightMat;
    const uint8_t mHorizon;

    Eigen::Matrix<double,3,3> mBodyInertia;
    Eigen::Matrix<double,3,3> mBodyInertiaInverse;

    Eigen::Matrix<double,13,13> Ac;
    Eigen::Matrix<double,13,12> Bc;

    Eigen::Matrix<double, Dynamic, 13> Aqp;
    Eigen::Matrix<double, Dynamic, Dynamic> Bqp;

    Eigen::Matrix<double, Dynamic, 1> xd;

    Eigen::Matrix<double, Dynamic, Dynamic> L;
    Eigen::Matrix<double, Dynamic, Dynamic> K;
    Eigen::Matrix<double, Dynamic, Dynamic> H;
    Eigen::Matrix<double, Dynamic, 1> g;

    Eigen::Matrix<double, Dynamic, 1> U_b;
    Eigen::Matrix<double, Dynamic, Dynamic> fmat;

    qpOASES::real_t* H_qpoases{};
    qpOASES::real_t* g_qpoases{};
    qpOASES::real_t* A_qpoases{};
    qpOASES::real_t* ub_qpoases{};
    qpOASES::real_t* lb_qpoases{};
    qpOASES::real_t* q_soln{};

    qpOASES::real_t* H_red{};
    qpOASES::real_t* g_red{};
    qpOASES::real_t* A_red{};
    qpOASES::real_t* lb_red{};
    qpOASES::real_t* ub_red{};
    qpOASES::real_t* q_red{};
    int8_t real_allocated = 0;

    Eigen::Matrix<double,25,25> ABc, expmm;
    Eigen::Matrix<double,13,13> Adt;
    Eigen::Matrix<double,13,12> Bdt;

    Eigen::Matrix<double,3,3> R_yaw;


};
#endif //RAISIM_QPSOLVER_H
