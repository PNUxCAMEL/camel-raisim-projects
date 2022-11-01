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
        , mPosFilterX(ESTIMATOR_dT, 100)
        , mPosFilterY(ESTIMATOR_dT, 100)
        , mPosFilterZ(ESTIMATOR_dT, 100)
        , mVelFilterX(ESTIMATOR_dT, 10)
        , mVelFilterY(ESTIMATOR_dT, 10)
        , mVelFilterZ(ESTIMATOR_dT, 10)
{
}

void SimulStateEstimator::StateEstimatorFunction()
{
    updateState();
    getJointState();
    getRobotAngulerState();
    getRobotLinearState();
    getRobotFootPosition();
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

void SimulStateEstimator::getRobotLinearState()
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

    if (bIsFirstRun)
    {
        sharedMemory->initPosition[0] = -mTransMat[2](0,3);
        sharedMemory->initPosition[1] = -mTransMat[2](1,3);
        sharedMemory->initPosition[2] = 0;

        sharedMemory->basePosition[0] = 0;
        sharedMemory->basePosition[1] = 0;
        sharedMemory->basePosition[2] = -mTransMat[2](2,3);

        bIsFirstRun = false;
    }
    else
    {
        double tempVal;
        for (int idx=0; idx<3; idx++)
        {
            tempVal = sharedMemory->basePosition[idx];
            sharedMemory->basePosition[idx] = -mTransMat[2](idx,3)-sharedMemory->initPosition[idx];
            sharedMemory->baseVelocity[idx] = (sharedMemory->basePosition[idx]-tempVal)/ESTIMATOR_dT;

            switch (idx) {
                case(0):
                {
                    sharedMemory->basePosition[idx] = mPosFilterX.GetFilteredVar(-mTransMat[2](idx,3)-sharedMemory->initPosition[idx]);
                    sharedMemory->baseVelocity[idx] = mVelFilterX.GetFilteredVar((sharedMemory->basePosition[idx]-tempVal)/ESTIMATOR_dT);
                    break;
                }
                case(1):
                {
                    sharedMemory->basePosition[idx] = mPosFilterY.GetFilteredVar(-mTransMat[2](idx,3)-sharedMemory->initPosition[idx]);
                    sharedMemory->baseVelocity[idx] = mVelFilterY.GetFilteredVar((sharedMemory->basePosition[idx]-tempVal)/ESTIMATOR_dT);
                    break;
                }
                case(2):
                {
                    sharedMemory->basePosition[idx] = mPosFilterZ.GetFilteredVar(-mTransMat[2](idx,3)-sharedMemory->initPosition[idx]);
                    sharedMemory->baseVelocity[idx] = mVelFilterZ.GetFilteredVar((sharedMemory->basePosition[idx]-tempVal)/ESTIMATOR_dT);
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
/*        sharedMemory->basePosition = mPosFilter.GetFilteredVar(sharedMemory->basePosition);
        sharedMemory->baseVelocity = mVelFilter.GetFilteredVar(sharedMemory->baseVelocity);*/
    }
}

void SimulStateEstimator::getRobotFootPosition()
{
    for (int leg=0; leg<4; leg++)
    {
        for (int idx=0; idx<3; idx++)
        {
            sharedMemory->footPosition[leg][idx] = mTransMat[leg](idx,3);
        }
    }
}
