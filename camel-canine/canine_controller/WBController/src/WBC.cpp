//
// Created by hs on 22. 10. 27.
//

#include <WBController/WBC.hpp>

extern pSHM sharedMemory;

WBC::WBC()
{
    for(auto & idx : mTorqueLimit)
    {
        idx << 11.0, 11.0, 11.0;
    }
    mTorque->setZero();
    mTorqueJacobian->setZero();
    mGRF->setZero();
    mMotorPosition->setZero();
    mJacobian->setZero();
}

void WBC::InitTrajectory()
{
    double timeDuration = 4.0;
    mCubicTrajectoryGen[0].updateTrajectory(sharedMemory->basePosition[0],
                                            0.0,
                                            sharedMemory->localTime, timeDuration);
    mCubicTrajectoryGen[1].updateTrajectory(sharedMemory->basePosition[1],
                                            0.0,
                                            sharedMemory->localTime, timeDuration);
    mCubicTrajectoryGen[2].updateTrajectory(sharedMemory->basePosition[2],
                                            0.3,
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
            mMotorVelocity[leg][mt] = sharedMemory->motorVelocity[leg*3+mt];
        }
    }

    mInitState << mBaseEulerPosition[0], mBaseEulerPosition[1], mBaseEulerPosition[2],
                  mBasePosition[0], mBasePosition[1], mBasePosition[2],
                  mBaseEulerVelocity[0], mBaseEulerVelocity[1], mBaseEulerVelocity[2],
                  mBaseVelocity[0], mBaseVelocity[1], mBaseVelocity[2], -9.81;
}

void WBC::setTrajectory()
{
    mDesiredState.setZero();
    mDesiredState(3,0) = mCubicTrajectoryGen[0].getPositionTrajectory(sharedMemory->localTime);
    mDesiredState(4,0) = mCubicTrajectoryGen[1].getPositionTrajectory(sharedMemory->localTime);
    mDesiredState(5,0) = mCubicTrajectoryGen[2].getPositionTrajectory(sharedMemory->localTime);
    mDesiredState(9,0) = mCubicTrajectoryGen[0].getVelocityTrajectory(sharedMemory->localTime);
    mDesiredState(10,0) = mCubicTrajectoryGen[1].getVelocityTrajectory(sharedMemory->localTime);
    mDesiredState(11,0) = mCubicTrajectoryGen[2].getVelocityTrajectory(sharedMemory->localTime);

    sharedMemory->baseDesiredPosition[0] = mDesiredState(3,0);
    sharedMemory->baseDesiredPosition[1] = mDesiredState(4,0);
    sharedMemory->baseDesiredPosition[2] = mDesiredState(5,0);

    sharedMemory->baseDesiredVelocity[0] = mDesiredState(9,0);
    sharedMemory->baseDesiredVelocity[1] = mDesiredState(10,0);
    sharedMemory->baseDesiredVelocity[2] = mDesiredState(11,0);
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
            std::cout << mTorque[leg][motor] << "\t";
            sharedMemory->motorDesiredTorque[leg*3+motor] = mTorque[leg][motor];
        }
    }
    std::cout << std::endl << std::endl;
}
