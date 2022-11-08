//
// Created by hs on 22. 10. 14.
//

#include <canine_util/StateEstimator.hpp>

extern pSHM sharedMemory;

StateEstimator::StateEstimator()
    : mPosFilter(ESTIMATOR_dT, 100)
    , mVelFilter(ESTIMATOR_dT, 10)
    , bIsFirstRun(true)
    , bIsRightFirst(true)
    , bIsLeftFirst(true)
{
}

void StateEstimator::StateEstimatorFunction()
{
    updateState();
    getRobotLinearState();
    getRobotFootPosition();
}

void StateEstimator::updateState()
{
    for(int idx=0; idx<4; idx++)
    {
        mQuaternion[idx] = sharedMemory->baseQuartPosition[idx];
    }
}

void StateEstimator::getRobotLinearState()
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

void StateEstimator::getRobotFootPosition()
{
    if (bIsFirstRun)
    {
        sharedMemory->basePosition[0] = 0;
        sharedMemory->basePosition[1] = 0;
        sharedMemory->basePosition[2] = -mTransMat[2](2,3);
        bIsFirstRun = false;
    }
    else
    {
        if (sharedMemory->gaitTable[2] == 1)
        {
            mBodyPosDiff = -mTransMat[2].block(0,3,3,1)-mBodyPrev[2];
        }
        else
        {
            mBodyPosDiff = -mTransMat[3].block(0,3,3,1)-mBodyPrev[3];
        }
        sharedMemory->basePosition += mBodyPosDiff;
        sharedMemory->baseVelocity = mVelFilter.GetFilteredVar(mBodyPosDiff/ESTIMATOR_dT);
    }

    mBodyPrev[2] = -mTransMat[2].block(0,3,3,1);
    mBodyPrev[3] = -mTransMat[3].block(0,3,3,1);
}
