//
// Created by hs on 22. 10. 27.
//

#include <WBController/WBC.hpp>

extern pSHM sharedMemory;

WBC::WBC()
{
    mTorque->setZero();
    mTorqueJacobian->setZero();
    mGRF->setZero();
    mMotorPosition->setZero();
    mTorqueLimit->setZero();
    mJacobian->setZero();
}

void WBC::InitTrajectory()
{
    double timeDuration = 2.0;
    mCubicTrajectoryGen.updateTrajectory(sharedMemory->basePosition[2],
                                         sharedMemory->basePosition[2]-0.1,
                                         sharedMemory->localTime, timeDuration);
}

void WBC::DoWBControl()
{
    updateState();
    setTrajectory();
    ForceQPsolver.SolveQP(mInitState, mDesiredState, mFootPosition);
    ForceQPsolver.GetGRF(mGRF);
    computeControlInput();
    setControlInput();
}

void WBC::updateState()
{
    memcpy(mBasePosition, sharedMemory->basePosition, sizeof(double)*3);
    memcpy(mBaseVelocity, sharedMemory->baseVelocity, sizeof(double)*3);
    memcpy(mBaseEulerPosition, sharedMemory->baseEulerPosition, sizeof(double)*3);
    memcpy(mBaseEulerVelocity, sharedMemory->baseEulerVelocity, sizeof(double)*3);
    memcpy(mFootPosition, sharedMemory->footPosition, sizeof(double)*4*3);

    mInitState << mBaseEulerPosition[0], mBaseEulerPosition[1], mBaseEulerPosition[2],
                  mBasePosition[0], mBasePosition[1], mBasePosition[2],
                  mBaseEulerVelocity[0], mBaseEulerVelocity[1], mBaseEulerVelocity[2],
                  mBaseVelocity[0], mBaseVelocity[1], mBaseVelocity[2], -9.81;
}

void WBC::setTrajectory()
{
    mDesiredState << 0, 0, 0,
                     0, 0, mCubicTrajectoryGen.getPositionTrajectory(sharedMemory->localTime),
                     0, 0, 0,
                     0, 0, mCubicTrajectoryGen.getVelocityTrajectory(sharedMemory->localTime),
                     0;
}

void WBC::computeControlInput()
{
    GetJacobian(mJacobian[0], mMotorPosition[0],1);
    GetJacobian(mJacobian[1], mMotorPosition[1],-1);
    GetJacobian(mJacobian[2], mMotorPosition[2],1);
    GetJacobian(mJacobian[3], mMotorPosition[3],-1);

    for(int idx=0; idx<4; idx++)
    {
        mJacobian[idx].transposeInPlace();
        mTorqueJacobian[idx] = mJacobian[idx]*mGRF[idx];
        mTorque[idx][0] = mTorqueJacobian[idx][0];
        mTorque[idx][1] = mTorqueJacobian[idx][1];
        mTorque[idx][2] = mTorqueJacobian[idx][2];
    }
}

void WBC::setControlInput()
{
    for (int leg = 0; leg < 4; leg++)
    {
        for (int motor = 0; motor < 3; motor++)
        {
            if (mTorque[leg][motor] > mTorqueLimit[leg][motor])
            {
                mTorque[leg][motor] = mTorqueLimit[leg][motor];
            }
            else if (mTorque[leg][motor] < -mTorqueLimit[leg][motor])
            {
                mTorque[leg][motor] = -mTorqueLimit[leg][motor];
            }
            sharedMemory->motorDesiredTorque[leg*3+motor] = mTorque[leg][motor];
        }
    }
}
