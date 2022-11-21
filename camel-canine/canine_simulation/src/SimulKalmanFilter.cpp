//
// Created by hs on 22. 11. 20.
//

#include <canine_simulation/SimulKalmanFilter.hpp>

extern pSHM sharedMemory;

SimulKalmanFilter::SimulKalmanFilter(raisim::ArticulatedSystem* robot)
    : mRobot(robot)
    , mPosition(raisim::VecDyn(19))
    , mVelocity(raisim::VecDyn(18))
    , IsKalmanFirstRun(true)
{
    mX.setZero();
    mZ.setZero();
    mA.setZero();
    mB.setZero();
    mH.setZero();
    mK.setZero();
    mAccel.setZero();
    mXprev.setZero();
}

void SimulKalmanFilter::StateEstimatorFunction()
{
    updateState();
    getJointState();
    getRobotAngulerState();
    getRobotFootPosition();
    getRobotLinearState();
}

void SimulKalmanFilter::updateState()
{
    mPosition = mRobot->getGeneralizedCoordinate();
    mVelocity = mRobot->getGeneralizedVelocity();
}

void SimulKalmanFilter::getJointState()
{
    for (int idx=0; idx<MOTOR_NUM; idx++)
    {
        sharedMemory->motorPosition[idx] = mPosition[idx+7];
        sharedMemory->motorVelocity[idx] = mVelocity[idx+6];
    }
}

void SimulKalmanFilter::getRobotAngulerState()
{
    for(int idx=0; idx<3; idx++)
    {
        sharedMemory->baseEulerVelocity[idx] = mVelocity[idx+3];
    }
    for(int idx=0; idx<4; idx++)
    {
        mQuaternion[idx] = mPosition[idx+3];
    }
    TransformQuat2Euler(mQuaternion, sharedMemory->baseEulerPosition);
}

void SimulKalmanFilter::getRobotFootPosition()
{
    TransMatBody2Foot(&mTransMat[0], R_FRON, mQuaternion,
                      sharedMemory->motorPosition[0],
                      sharedMemory->motorPosition[1],
                      sharedMemory->motorPosition[2]);
    TransMatBody2Foot(&mTransMat[1], L_FRON, mQuaternion,
                      sharedMemory->motorPosition[3],
                      sharedMemory->motorPosition[4],
                      sharedMemory->motorPosition[5]);
    TransMatBody2Foot(&mTransMat[2], R_BACK, mQuaternion,
                      sharedMemory->motorPosition[6],
                      sharedMemory->motorPosition[7],
                      sharedMemory->motorPosition[8]);
    TransMatBody2Foot(&mTransMat[3], L_BACK, mQuaternion,
                      sharedMemory->motorPosition[9],
                      sharedMemory->motorPosition[10],
                      sharedMemory->motorPosition[11]);

    for (int leg=0; leg<4; leg++)
    {
        for (int idx=0; idx<3; idx++)
        {
            sharedMemory->footPosition[leg][idx] = mTransMat[leg](idx,3);
        }
    }
}

void SimulKalmanFilter::getRobotLinearState()
{
    if (IsKalmanFirstRun)
    {
        initLinearKalmanFilter();
        IsKalmanFirstRun = false;
    }
    else
    {
        setLinearKalmanFilter();
    }
    doLinearKalmanFilter();

    sharedMemory->basePosition[0] = mX[0];
    sharedMemory->basePosition[1] = mX[1];
    sharedMemory->basePosition[2] = mX[2];

    sharedMemory->baseVelocity[0] = mX[3];
    sharedMemory->baseVelocity[1] = mX[4];
    sharedMemory->baseVelocity[2] = mX[5];
}

void SimulKalmanFilter::initLinearKalmanFilter()
{
    mX[2] = -(mTransMat[0](2,3)+mTransMat[1](2,3)+mTransMat[2](2,3)+mTransMat[3](2,3)) / 4;
    Vec3<double> mGlobalFootPos[4];
    for (int leg=0; leg<4; leg++)
    {
        for (int idx=0; idx<3; idx++)
        {
            mGlobalFootPos[leg][idx] = mX[idx] + mTransMat[leg](idx,3);
        }
    }
    mX.block( 6, 0, 3, 1) = mGlobalFootPos[0];
    mX.block( 9, 0, 3, 1) = mGlobalFootPos[1];
    mX.block(12, 0, 3, 1) = mGlobalFootPos[2];
    mX.block(15, 0, 3, 1) = mGlobalFootPos[3];

    mP.setIdentity();
    mP = mP*1e-1;

    mA.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    mA.block(0, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * ESTIMATOR_dT;
    mA.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    mA.block(6, 6, 12, 12) = Eigen::Matrix<double, 12, 12>::Identity();

    mB.block(3, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * ESTIMATOR_dT;

    mH.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    mH.block(3, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    mH.block(6, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    mH.block(9, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    mH.block(0, 6, 12, 12) = (-1.0)*Eigen::Matrix<double, 12, 12>::Identity();

    mQ.setIdentity();
    mQ = mQ*1e-2;
    mR.setIdentity();
    mR = mR*10;
}

void SimulKalmanFilter::setLinearKalmanFilter()
{
    mAccel[0] = (mX[3] - mXprev[3])/ESTIMATOR_dT;
    mAccel[1] = (mX[4] - mXprev[4])/ESTIMATOR_dT;
    mAccel[2] = (mX[5] - mXprev[5])/ESTIMATOR_dT;
}

void SimulKalmanFilter::doLinearKalmanFilter()
{
    mZ.block(0, 0, 3, 1) = -mTransMat[0].block(0, 3, 3, 1);
    mZ.block(3, 0, 3, 1) = -mTransMat[1].block(0, 3, 3, 1);
    mZ.block(6, 0, 3, 1) = -mTransMat[2].block(0, 3, 3, 1);
    mZ.block(9, 0, 3, 1) = -mTransMat[3].block(0, 3, 3, 1);

    mXp = mA*mX;
    mPp = mA*mP*mA.transpose() + mQ;

    mS = mH*mPp*mH.transpose() + mR;
    mK = mPp*mH.transpose()*mS.inverse();

    mXprev = mX;
    mZp = mH*mXp;
    mX = mXp + mK*(mZ-mZp);
    mP = mPp - mK*mH*mPp;
}