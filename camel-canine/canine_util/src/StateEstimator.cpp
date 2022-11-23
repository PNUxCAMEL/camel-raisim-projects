//
// Created by hs on 22. 10. 14.
//

#include <canine_util/StateEstimator.hpp>

extern pSHM sharedMemory;

StateEstimator::StateEstimator()
    : IsKalmanFirstRun(true)
{
}

void StateEstimator::StateEstimatorFunction()
{
    if (sharedMemory->motorForeState && sharedMemory->motorBackState)
    {
        updateState();
        getRobotFootPosition();
//        getRobotLinearState();
    }
}

void StateEstimator::updateState()
{
    for(int idx=0; idx<4; idx++)
    {
        mQuaternion[idx] = sharedMemory->baseQuartPosition[idx];
    }
    for(int idx=0; idx<3; idx++)
    {
        mAcceleration[idx] = sharedMemory->baseAcceleration[idx];
    }
}

void StateEstimator::getRobotFootPosition()
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

void StateEstimator::getRobotLinearState()
{
    if (IsKalmanFirstRun)
    {
        initLinearKalmanFilter();
        IsKalmanFirstRun = false;
    }
    doLinearKalmanFilter();

    sharedMemory->basePosition[0] = mX[0];
    sharedMemory->basePosition[1] = mX[1];
    sharedMemory->basePosition[2] = mX[2];

    sharedMemory->baseVelocity[0] = mX[3];
    sharedMemory->baseVelocity[1] = mX[4];
    sharedMemory->baseVelocity[2] = mX[5];
}

void StateEstimator::initLinearKalmanFilter()
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
    mP = mP*10;

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

    mQ(0,0)   = 1e-7;
    mQ(1,1)   = 1e-7;
    mQ(2,2)   = 1e-7;
    mQ(3,3)   = 1e-2;
    mQ(4,4)   = 1e-2;
    mQ(5,5)   = 1e-2;
    mQ(6,6)   = 1e-10;
    mQ(7,7)   = 1e-10;
    mQ(8,8)   = 1e-10;
    mQ(9,9)   = 1e-10;
    mQ(10,10) = 1e-10;
    mQ(11,11) = 1e-10;
    mQ(12,12) = 1e-10;
    mQ(13,13) = 1e-10;
    mQ(14,14) = 1e-10;
    mQ(15,15) = 1e-10;
    mQ(16,16) = 1e-10;
    mQ(17,17) = 1e-10;

    mR.setIdentity();
    mR = mR*1e-2;

}
void StateEstimator::doLinearKalmanFilter()
{
    mZ.block(0, 0, 3, 1) = -mTransMat[0].block(0, 3, 3, 1);
    mZ.block(3, 0, 3, 1) = -mTransMat[1].block(0, 3, 3, 1);
    mZ.block(6, 0, 3, 1) = -mTransMat[2].block(0, 3, 3, 1);
    mZ.block(9, 0, 3, 1) = -mTransMat[3].block(0, 3, 3, 1);


//    mXp = mA*mX + mB*mAcceleration;
    mXp = mA*mX;
    mPp = mA*mP*mA.transpose() + mQ;

    mS = mH*mPp*mH.transpose() + mR;
    mK = mPp*mH.transpose()*mS.inverse();

    for (int idx=0; idx<4; idx++)
    {
        if (sharedMemory->gaitTable[idx] == 0)
        {
            for (int j=0; j<6; j++)
            {
                mK(j, idx*3  ) = 0;
                mK(j, idx*3+1) = 0;
                mK(j, idx*3+2) = 0;
            }
        }
    }
    mZp = mZ - mH*mXp;
    mX = mXp + mK*mZp;
    mP = mPp - mK*mH*mPp;
}