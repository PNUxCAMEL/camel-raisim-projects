//
// Created by camel on 22. 9. 21.
//

#include <PDQPcontrol/PDQPcontroller.hpp>
#include <iostream>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

PDQPController::PDQPController()
    : mRefTime(0.0)
    , mHomeState(HOME_NO_ACT)
{
    for (int motorIdx = 0; motorIdx < MOTOR_NUM; motorIdx++)
    {
        Kp[motorIdx] = 150.0;
        Kd[motorIdx] = 4.5;
        mTorqueLimit[motorIdx] = 15.0;
    }
}

void PDQPController::DoHomeControl()
{
    updateState();
    updateHomeTrajectory();
    setHomeTrajectory();

    setTrajectory();
    ForceQPsolver.SolveQP(mInitState, mDesiredState, mFootPosition);
    ForceQPsolver.GetGRF(mGRF);

    computeControlInput();
    SetControlInput();
}

void PDQPController::updateState()
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

void PDQPController::setTrajectory()
{
    mDesiredState.setZero();
    mDesiredState(5,0) = mBasePosition[2];

    sharedMemory->baseDesiredPosition[0] = mDesiredState(3,0);
    sharedMemory->baseDesiredPosition[1] = mDesiredState(4,0);
    sharedMemory->baseDesiredPosition[2] = mDesiredState(5,0);

    sharedMemory->baseDesiredVelocity[0] = mDesiredState(9,0);
    sharedMemory->baseDesiredVelocity[1] = mDesiredState(10,0);
    sharedMemory->baseDesiredVelocity[2] = mDesiredState(11,0);
}

void PDQPController::InitTrajectory()
{
    double timeDuration = 3.5;
    mBodyTrajectory[0].updateTrajectory(sharedMemory->basePosition[0],
                                        0.0,
                                        sharedMemory->localTime, timeDuration);
    mBodyTrajectory[1].updateTrajectory(sharedMemory->basePosition[1],
                                        0.0,
                                        sharedMemory->localTime, timeDuration);
    mBodyTrajectory[2].updateTrajectory(sharedMemory->basePosition[2],
                                        0.3,
                                        sharedMemory->localTime, timeDuration);
}

void PDQPController::InitHomeStandUpTrajectory()
{
    mHomeState = HOME_STAND_UP_PHASE1;
}

void PDQPController::InitHomeStandDownTrajectory()
{
    mHomeState = HOME_STAND_DOWN_PHASE1;
}

void PDQPController::computeControlInput()
{
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        mTorque[index] = Kp[index] * (mDesiredPosition[index] - sharedMemory->motorPosition[index])
            + Kd[index] * (mDesiredVelocity[index] - sharedMemory->motorVelocity[index]);
    }

    GetJacobian(mJacobian[0], mMotorPosition[0],1);
    GetJacobian(mJacobian[1], mMotorPosition[1],-1);
    GetJacobian(mJacobian[2], mMotorPosition[2],1);
    GetJacobian(mJacobian[3], mMotorPosition[3],-1);

    for(int idx=0; idx<4; idx++)
    {
        mJacobian[idx].transposeInPlace();
        mTorqueJacobian[idx] = mJacobian[idx]*mGRF[idx];
    }

    double coef = 0.3;

    for(int idx=0; idx<MOTOR_NUM; idx++)
    {
//        mTorque[idx] = coef*mTorque[idx]+(1-coef)*mTorqueJacobian[idx%4][idx/4];
        mTorque[idx] += coef*mTorqueJacobian[idx%4][idx/4];
    }
}

void PDQPController::updateHomeTrajectory()
{
    switch(mHomeState)
    {
    case HOME_NO_ACT:
        break;
    case HOME_STAND_UP_PHASE1:
        for (int idx = 0; idx < 4; idx++)
        {
            double homeHip = 105;
            double homeKnee = -157;
            double timeDuration = 1.0;
            mRefTime = sharedMemory->localTime + timeDuration;
            mCubicTrajectoryGen[idx * 3].updateTrajectory(sharedMemory->motorPosition[idx * 3], 0.0, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 1].updateTrajectory(sharedMemory->motorPosition[idx * 3 + 1], homeHip * D2R, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 2].updateTrajectory(sharedMemory->motorPosition[idx * 3 + 2], homeKnee * D2R, sharedMemory->localTime, timeDuration);
        }
        mHomeState = HOME_STAND_UP_PHASE2;
        break;
    case HOME_STAND_UP_PHASE2:
        if(sharedMemory->localTime > mRefTime + 0.5)
        {
            mHomeState = HOME_STAND_UP_PHASE3;
        }
        break;
    case HOME_STAND_UP_PHASE3:
        for (int idx = 0; idx < 4; idx++)
        {
            double homeHip = 55.5;
            double homeKnee = -98.0;
            double timeDuration = 2.0;
            mRefTime = sharedMemory->localTime + timeDuration;
            mCubicTrajectoryGen[idx * 3].updateTrajectory(mDesiredPosition[idx * 3], 0.0, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 1].updateTrajectory(mDesiredPosition[idx * 3 + 1], homeHip * D2R, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 2].updateTrajectory(mDesiredPosition[idx * 3 + 2], homeKnee * D2R, sharedMemory->localTime, timeDuration);
        }
        mHomeState = HOME_NO_ACT;
        break;
    case HOME_STAND_DOWN_PHASE1:
        for (int idx = 0; idx < 4; idx++)
        {
            double homeHip = 88;
            double homeKnee = -157;
            double timeDuration = 2.0;
            mRefTime = sharedMemory->localTime + timeDuration;
            mCubicTrajectoryGen[idx * 3].updateTrajectory(mDesiredPosition[idx * 3], 0.0, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 1].updateTrajectory(mDesiredPosition[idx * 3 + 1], homeHip * D2R, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 2].updateTrajectory(mDesiredPosition[idx * 3 + 2], homeKnee * D2R, sharedMemory->localTime, timeDuration);
        }
        mHomeState = HOME_STAND_DOWN_PHASE2;
        break;
    case HOME_STAND_DOWN_PHASE2:
        if(sharedMemory->localTime > mRefTime + 0.5)
        {
            mHomeState = HOME_STAND_DOWN_PHASE3;
        }
        break;
    case HOME_STAND_DOWN_PHASE3:
        for (int idx = 0; idx < 4; idx++)
        {
            double homeHip = 126;
            double homeKnee = -157;
            double timeDuration = 1.5;
            mRefTime = sharedMemory->localTime + timeDuration;
            mCubicTrajectoryGen[idx * 3].updateTrajectory(mDesiredPosition[idx * 3], 0.0, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 1].updateTrajectory(mDesiredPosition[idx * 3 + 1], homeHip * D2R, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 2].updateTrajectory(mDesiredPosition[idx * 3 + 2], homeKnee * D2R, sharedMemory->localTime, timeDuration);
        }
        mHomeState = HOME_NO_ACT;
        break;
    default:
        break;
    }
}

void PDQPController::setHomeTrajectory()
{
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        mDesiredPosition[index] = mCubicTrajectoryGen[index].getPositionTrajectory(sharedMemory->localTime);
        mDesiredVelocity[index] = mCubicTrajectoryGen[index].getVelocityTrajectory(sharedMemory->localTime);
    }
}

void PDQPController::SetControlInput()
{
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        if (sharedMemory->controlState == STATE_READY)
        {
            mTorque[index] = 0;
        }
        else
        {
            if (mTorque[index] > mTorqueLimit[index])
            {
                mTorque[index] = mTorqueLimit[index];
            }
            else if (mTorque[index] < -mTorqueLimit[index])
            {
                mTorque[index] = -mTorqueLimit[index];
            }
        }
        sharedMemory->motorDesiredTorque[index] = mTorque[index];
    }
}


void PDQPController::SetPDgain(const double& kp, const double& kd)
{
    for (int motorIdx = 0; motorIdx < MOTOR_NUM; motorIdx++)
    {
        Kp[motorIdx] = kp;
        Kd[motorIdx] = kd;
    }
}