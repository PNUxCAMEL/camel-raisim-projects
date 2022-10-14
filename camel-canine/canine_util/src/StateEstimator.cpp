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

    }
    else
    {
        EstimatorSimul();
    }
}

void StateEstimator::EstimatorSimul()
{
    mPosition = mRobot->getGeneralizedCoordinate();
    mVelocity = mRobot->getGeneralizedVelocity();

    for (int idx=0; idx<MOTOR_NUM; idx++)
    {
        sharedMemory->motorPosition[idx] = mPosition[idx+7];
        sharedMemory->motorVelocity[idx] = mVelocity[idx+6];
    }

}
