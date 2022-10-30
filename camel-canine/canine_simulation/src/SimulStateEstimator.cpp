//
// Created by hs on 22. 10. 27.
//

#include <canine_simulation/SimulStateEstimator.hpp>

extern pSHM sharedMemory;

SimulStateEstimator::SimulStateEstimator(raisim::ArticulatedSystem* robot)
        : mRobot(robot)
        , mPosition(raisim::VecDyn(19))
        , mVelocity(raisim::VecDyn(18))
{
}

void SimulStateEstimator::StateEstimatorFunction()
{
    mPosition = mRobot->getGeneralizedCoordinate();
    mVelocity = mRobot->getGeneralizedVelocity();

    getJointState();
    getRobotAngulerState();
    getRobotLinearState();
    getRobotFootPosition();
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
    for(int idx=0; idx<3; idx++)
    {
        sharedMemory->basePosition[idx] = mPosition[idx];
        sharedMemory->baseVelocity[idx] = mVelocity[idx];
    }
}

void SimulStateEstimator::getRobotFootPosition()
{
    auto FRfootFrameIndex = mRobot->getFrameIdxByName("RF_FOOT");
    auto FLfootFrameIndex = mRobot->getFrameIdxByName("LF_FOOT");
    auto RRfootFrameIndex = mRobot->getFrameIdxByName("RH_FOOT");
    auto RLfootFrameIndex = mRobot->getFrameIdxByName("LH_FOOT");

    mRobot->getFramePosition(FRfootFrameIndex, mFootPosition[0]);
    mRobot->getFramePosition(FLfootFrameIndex, mFootPosition[1]);
    mRobot->getFramePosition(RRfootFrameIndex, mFootPosition[2]);
    mRobot->getFramePosition(RLfootFrameIndex, mFootPosition[3]);

    for (int leg=0; leg<4; leg++)
    {
        for (int mt=0; mt<3; mt++)
        {
            sharedMemory->footPosition[leg][mt] = mFootPosition[leg][mt];
        }
    }
}