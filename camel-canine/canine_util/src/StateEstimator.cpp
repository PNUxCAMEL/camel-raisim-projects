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
        getRobotFootPositionSimul();
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

void StateEstimator::getRobotFootPositionSimul() {
    auto FRfootFrameIndex = mRobot->getFrameIdxByName("RF_FOOT");
    auto FLfootFrameIndex = mRobot->getFrameIdxByName("LF_FOOT");
    auto RRfootFrameIndex = mRobot->getFrameIdxByName("RH_FOOT");
    auto RLfootFrameIndex = mRobot->getFrameIdxByName("LH_FOOT");

    //Get foot position on the world frame
    mRobot->getFramePosition(FRfootFrameIndex, mFootPosition[0]);
    mRobot->getFramePosition(FLfootFrameIndex, mFootPosition[1]);
    mRobot->getFramePosition(RRfootFrameIndex, mFootPosition[2]);
    mRobot->getFramePosition(RLfootFrameIndex, mFootPosition[3]);

    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 3; col++) {
            sharedMemory->footPosition[row][col] = mFootPosition[row][col];
        }
    }
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
