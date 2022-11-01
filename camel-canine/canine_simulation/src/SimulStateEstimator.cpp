//
// Created by hs on 22. 10. 27.
//

#include <canine_simulation/SimulStateEstimator.hpp>

extern pSHM sharedMemory;

SimulStateEstimator::SimulStateEstimator(raisim::ArticulatedSystem* robot)
        : mRobot(robot)
        , mPosition(raisim::VecDyn(19))
        , mVelocity(raisim::VecDyn(18))
        , bIsFirstRun(true)
        , mPosFilter(ESTIMATOR_dT, 100)
        , mVelFilter(ESTIMATOR_dT, 10)
{
}

void SimulStateEstimator::StateEstimatorFunction()
{
    updateState();
    getJointState();
    getRobotAngulerState();
    getRobotFootPosition();
    getRobotLinearState();
}

void SimulStateEstimator::updateState()
{
    mPosition = mRobot->getGeneralizedCoordinate();
    mVelocity = mRobot->getGeneralizedVelocity();
}

void SimulStateEstimator::getJointState()
{
    for (int idx=0; idx<MOTOR_NUM; idx++)
    {
        sharedMemory->motorPosition[idx] = mPosition[idx+7];
        sharedMemory->motorVelocity[idx] = mVelocity[idx+6];
    }
}

void SimulStateEstimator::getRobotAngulerState()
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

void SimulStateEstimator::getRobotFootPosition()
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

void SimulStateEstimator::getRobotLinearState()
{
    if (bIsFirstRun)
    {
        for(int leg=0; leg<4; leg++)
        {
            mInitPosition[leg][0] = -mTransMat[leg](0,3);
            mInitPosition[leg][1] = -mTransMat[leg](1,3);
            mInitPosition[leg][2] = 0;
        }
        sharedMemory->basePosition[0] = 0;
        sharedMemory->basePosition[1] = 0;
        sharedMemory->basePosition[2] = -(mTransMat[0](2,3)+mTransMat[1](2,3)+mTransMat[2](2,3)+mTransMat[3](2,3))/4;

        bIsFirstRun = false;
    }
    else
    {
        mStandCount = 0;
        mTempPosMean.setZero();
        mTempPosPrev = sharedMemory->basePosition;

        for (int i=0; i<4; i++)
        {
            std::cout << sharedMemory->gaitTable[i] << "\t";
        }
        std::cout << std::endl;
        for (int leg=0; leg<4; leg++)
        {
            if (sharedMemory->gaitTable[leg] == 1)
            {
                mTempPos[leg][0] = -mTransMat[leg](0,3)-mInitPosition[leg][0];
                mTempPos[leg][1] = -mTransMat[leg](1,3)-mInitPosition[leg][1];
                mTempPos[leg][2] = -mTransMat[leg](2,3)-mInitPosition[leg][2];
                mTempPosMean += mTempPos[leg];
                mStandCount++;
            }
        }
        mTempPosMean /= mStandCount;
        sharedMemory->basePosition = mPosFilter.GetFilteredVar(mTempPosMean);

        mTempVel = (sharedMemory->basePosition-mTempPosPrev)/ESTIMATOR_dT;
        sharedMemory->baseVelocity = mVelFilter.GetFilteredVar(mTempVel);
    }
}