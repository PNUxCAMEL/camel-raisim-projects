//
// Created by hs on 22. 10. 14.
//

#ifndef RAISIM_STATEESTIMATOR_HPP
#define RAISIM_STATEESTIMATOR_HPP

#include <raisim/World.hpp>

#include "RobotDescription.hpp"
#include "SharedMemory.hpp"
#include "RobotMath.hpp"
#include "Filter.hpp"

class StateEstimator{
public:
    StateEstimator();

    void StateEstimatorFunction();
private:
    void updateState();
    void getRobotLinearState();
    void getRobotFootPosition();
    void initLinearKalmanFilter();
    void doLinearKalmanFilter();
private:
    Vec4<double> mQuaternion;
    Mat4<double> mTransMat[4];
    Vec3<double> mAcceleration;

    Eigen::Matrix<double, 18, 1> mX;
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

    bool IsKalmanFirstRun;
};

#endif //RAISIM_STATEESTIMATOR_HPP
