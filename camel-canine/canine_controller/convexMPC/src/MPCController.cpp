//
// Created by hs on 22. 6. 27.
//

#include <convexMPC/MPCController.hpp>

extern pSHM sharedMemory;

MPCController::MPCController(const uint8_t& horizon)
    : mHorizon(horizon)
    , ConvexMPCSolver(mHorizon)
{
    for(int motorIdx = 0; motorIdx < MOTOR_NUM ; motorIdx++)
    {
        mTorqueLimit[motorIdx] = 11.0;
    }

    mGRF->setZero();
    robotJacobian->setZero();
    robottorque->setZero();
}

void MPCController::DoControl()
{
    ConvexMPCSolver.SetTrajectory();
    ConvexMPCSolver.GetMetrices();
    ConvexMPCSolver.SolveQP();
    ConvexMPCSolver.GetGRF(mGRF);

    computeControlInput();
    setControlInput();
}

void MPCController::computeControlInput()
{
    for(int idx=0; idx<4; idx++)
    {
        if (idx%2 == 0)
        {
            ConvexMPCSolver.GetJacobian(robotJacobian[idx],
                                   sharedMemory->motorPosition[idx*3],
                                   sharedMemory->motorPosition[idx*3+1],
                                   sharedMemory->motorPosition[idx*3+2],
                                   1);
        }
        {
            ConvexMPCSolver.GetJacobian(robotJacobian[idx],
                                   sharedMemory->motorPosition[idx*3],
                                   sharedMemory->motorPosition[idx*3+1],
                                   sharedMemory->motorPosition[idx*3+2],
                                   -1);
        }


        robotJacobian[idx].transposeInPlace();
        robottorque[idx] = robotJacobian[idx]*mGRF[idx];
        mTorque[idx*3+6] = robottorque[idx][0];
        mTorque[idx*3+7] = robottorque[idx][1];
        mTorque[idx*3+8] = robottorque[idx][2];
    }
}

void MPCController::setControlInput()
{
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        if (mTorque[index] > mTorqueLimit[index])
        {
            mTorque[index] = mTorqueLimit[index];
        }
        else if (mTorque[index] < -mTorqueLimit[index])
        {
            mTorque[index] = -mTorqueLimit[index];
        }
        sharedMemory->motorDesiredTorque[index] = mTorque[index];
    }
}
