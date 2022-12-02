//
// Created by hs on 22. 11. 20.
//

#ifndef RAISIM_SIMULKALMANFILTER_HPP
#define RAISIM_SIMULKALMANFILTER_HPP

#endif //RAISIM_SIMULKALMANFILTER_HPP

#include <raisim/World.hpp>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotMath.hpp>

class SimulKalmanFilter{
public:
    SimulKalmanFilter(raisim::ArticulatedSystem* robot);

    void StateEstimatorFunction();
private:
    void updateState();
    void getJointState();
    void getRobotAngulerState();
    void getRobotFootPosition();
    void getRobotLinearState();
    void initLinearKalmanFilter();
    void doLinearKalmanFilter();

private:
    raisim::ArticulatedSystem* mRobot;
    raisim::VecDyn mPosition;
    raisim::VecDyn mVelocity;

    Vec4<double> mQuaternion;
    Mat4<double> mTransMat[4];


    Eigen::Matrix<double, 18, 1> mX;
    Eigen::Matrix<double, 18, 1> mXprev;
    Eigen::Matrix<double, 18, 1> mXp;
    Eigen::Matrix<double, 18, 18> mP;
    Eigen::Matrix<double, 18, 18> mPp;
    Eigen::Matrix<double, 12, 1> mZ;
    Eigen::Matrix<double, 12, 1> mZp;

    Eigen::Matrix<double, 18, 18> mA;
    Eigen::Matrix<double, 18,  3> mB;
    Eigen::Matrix<double, 12, 18> mH;
    Eigen::Matrix<double, 18, 18> mQ;
    Eigen::Matrix<double, 12, 12> mR;
    Eigen::Matrix<double, 12, 12> mS;
    Eigen::Matrix<double, 18, 12> mK;

    Vec3<double> mAccel;

    bool IsKalmanFirstRun;
};