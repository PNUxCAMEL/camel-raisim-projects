//
// Created by hs on 22. 6. 27.
//

#include <convexMPC/MPCController.hpp>

extern pSHM sharedMemory;

MPCController::MPCController(const uint8_t& horizon)
    : mHorizon(horizon)
    , ConvexMPCSolver(mHorizon)
{
    mGRF->setZero();
    mTorque->setZero();
    mInitState.setZero();
}

void MPCController::InitUpTrajectory()
{
    double timeDuration = 2.0;
    mBaseTrajectory[0].updateTrajectory(sharedMemory->basePosition[0], 0.0,
                                        sharedMemory->localTime, timeDuration);
    mBaseTrajectory[1].updateTrajectory(sharedMemory->basePosition[1], 0.0,
                                        sharedMemory->localTime, timeDuration);
    mBaseTrajectory[2].updateTrajectory(sharedMemory->basePosition[2], 0.35,
                                        sharedMemory->localTime, timeDuration);
}

void MPCController::InitDownTrajectory()
{
    double timeDuration = 2.0;
    mBaseTrajectory[0].updateTrajectory(sharedMemory->basePosition[0], sharedMemory->basePosition[0],
                                        sharedMemory->localTime, timeDuration);
    mBaseTrajectory[1].updateTrajectory(sharedMemory->basePosition[1], sharedMemory->basePosition[1],
                                        sharedMemory->localTime, timeDuration);
    mBaseTrajectory[2].updateTrajectory(sharedMemory->basePosition[2], 0.0,
                                        sharedMemory->localTime, timeDuration);
}


void MPCController::DoControl()
{
    updateState();
    ConvexMPCSolver.SetTrajectory(mBaseTrajectory);
    ConvexMPCSolver.GetMetrices(mInitState, mFootPosition);
    ConvexMPCSolver.SolveQP();
    ConvexMPCSolver.GetGRF(mGRF);

    computeControlInput();
}

void MPCController::updateState()
{
    for (int idx=0; idx<3; idx++)
    {
        mBasePosition[idx] = sharedMemory->basePosition[idx];
        mBaseVelocity[idx] = sharedMemory->baseVelocity[idx];
        mBaseEulerPosition[idx] = sharedMemory->baseEulerPosition[idx];
        mBaseEulerVelocity[idx] = sharedMemory->baseEulerVelocity[idx];
    }

    for (int leg=0; leg<4; leg++)
    {
        for (int mt=0; mt<3; mt++)
        {
            mFootPosition[leg][mt] = sharedMemory->footPosition[leg][mt];
            mMotorPosition[leg][mt] = sharedMemory->motorPosition[leg*3+mt];
        }
    }

    mInitState << mBaseEulerPosition[0], mBaseEulerPosition[1], mBaseEulerPosition[2],
            mBasePosition[0], mBasePosition[1], mBasePosition[2],
            mBaseEulerVelocity[0], mBaseEulerVelocity[1], mBaseEulerVelocity[2],
            mBaseVelocity[0], mBaseVelocity[1], mBaseVelocity[2], GRAVITY;
}

void MPCController::computeControlInput()
{
    GetJacobian(mJacobian[0], mMotorPosition[0],1);
    GetJacobian(mJacobian[1], mMotorPosition[1],-1);
    GetJacobian(mJacobian[2], mMotorPosition[2],1);
    GetJacobian(mJacobian[3], mMotorPosition[3],-1);

    for(int idx=0; idx<4; idx++)
    {
        mJacobian[idx].transposeInPlace();
        mTorqueJacobian[idx] = mJacobian[idx]*mGRF[idx];
        sharedMemory->mpcTorque[idx][0] = mTorqueJacobian[idx][0];
        sharedMemory->mpcTorque[idx][1] = mTorqueJacobian[idx][1];
        sharedMemory->mpcTorque[idx][2] = mTorqueJacobian[idx][2];
    }
}