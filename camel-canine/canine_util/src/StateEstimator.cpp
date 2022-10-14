//
// Created by hs on 22. 10. 14.
//

#include <canine_util/StateEstimator.hpp>

extern pSHM sharedMemory;

StateEstimator::StateEstimator(raisim::ArticulatedSystem* robot)
    : mRobot(robot)
    , mPosition(raisim::VecDyn(19))
    , mVelocity(raisim::VecDyn(18))
{
}

void StateEstimator::StateEstimatorFunction()
{
    if (sharedMemory->simulState == WITH_SIMULATION)
    {
        getJointStateReal();
        getRobotAngulerStateReal();
    }
    else
    {
        getJointStateSimul();
        getRobotAngulerStateSimul();
    }
    calculateRobotLinearState();
}

void StateEstimator::getJointStateReal()
{
    //TODO: Get real robot encoder value from can
}

void StateEstimator::getJointStateSimul()
{
    mPosition = mRobot->getGeneralizedCoordinate();
    mVelocity = mRobot->getGeneralizedVelocity();

    for (int idx=0; idx<MOTOR_NUM; idx++)
    {
        sharedMemory->motorPosition[idx] = mPosition[idx+7];
        sharedMemory->motorVelocity[idx] = mVelocity[idx+6];
    }
}

void StateEstimator::getRobotAngulerStateReal()
{
    //TODO: Get angular velocity, position from IMU
}

void StateEstimator::getRobotAngulerStateSimul()
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

void StateEstimator::calculateRobotLinearState()
{
    //TODO: Calculate global body position and velocity
    for(int idx=0; idx<3; idx++)
    {
        sharedMemory->basePosition[idx] = mPosition[idx];
        sharedMemory->baseVelocity[idx] = mVelocity[idx];
    }
}
