//
// Created by camel on 22. 9. 21.
//

#include <PDcontroller/JointPDController.hpp>
#include <iostream>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

JointPDController::JointPDController()
    : mRefTime(0.0)
    , mHomeState(HOME_NO_ACT)
{
    for (int motorIdx = 0; motorIdx < MOTOR_NUM; motorIdx++)
    {
        Kp[motorIdx] = 150.0;
        Kd[motorIdx] = 4.5;
        mTorqueLimit[motorIdx] = 11.0;
    }
}

void JointPDController::DoHomeControl()
{
    updateHomeTrajectory();
    setHomeTrajectory();
    computeControlInput();
    SetControlInput();
}

void JointPDController::DoPDControl()
{
    setTrajectory();
    computeControlInput();
    SetControlInput();
}

void JointPDController::InitHomeStandUpTrajectory()
{
    mHomeState = HOME_STAND_UP_PHASE1;
}

void JointPDController::InitHomeStandDownTrajectory()
{
    mHomeState = HOME_STAND_DOWN_PHASE1;
}

void JointPDController::InitSwingTrajectory()
{

}

void JointPDController::setTrajectory()
{

}

void JointPDController::computeControlInput()
{
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        mTorque[index] = Kp[index] * (mDesiredPosition[index] - sharedMemory->motorPosition[index])
            + Kd[index] * (mDesiredVelocity[index] - sharedMemory->motorVelocity[index]);
    }
}

void JointPDController::updateHomeTrajectory()
{
    switch(mHomeState)
    {
    case HOME_NO_ACT:
        break;
    case HOME_STAND_UP_PHASE1:
        {
            double homeHip = 105;
            double homeKnee = -157;
            double timeDuration = 1.0;
            mRefTime = sharedMemory->localTime + timeDuration;
            mCubicTrajectoryGen[0].updateTrajectory(sharedMemory->motorPosition[0], homeHip * D2R, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[1].updateTrajectory(sharedMemory->motorPosition[1], homeKnee * D2R, sharedMemory->localTime, timeDuration);
            mHomeState = HOME_STAND_UP_PHASE2;
        }
        break;

    case HOME_STAND_UP_PHASE2:
        if(sharedMemory->localTime > mRefTime + 0.5)
        {
            mHomeState = HOME_STAND_UP_PHASE3;
        }
        break;
    case HOME_STAND_UP_PHASE3:
        {
            double homeHip = 55.5;
            double homeKnee = -98.0;
            double timeDuration = 2.0;
            mRefTime = sharedMemory->localTime + timeDuration;
            mCubicTrajectoryGen[0].updateTrajectory(sharedMemory->motorPosition[0], homeHip * D2R, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[1].updateTrajectory(sharedMemory->motorPosition[1], homeKnee * D2R, sharedMemory->localTime, timeDuration);
            mHomeState = HOME_NO_ACT;
        }
        break;
    case HOME_STAND_DOWN_PHASE1:
        {
            double homeHip = 88;
            double homeKnee = -157;
            double timeDuration = 2.0;
            mRefTime = sharedMemory->localTime + timeDuration;
            mCubicTrajectoryGen[0].updateTrajectory(sharedMemory->motorPosition[0], homeHip * D2R, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[1].updateTrajectory(sharedMemory->motorPosition[1], homeKnee * D2R, sharedMemory->localTime, timeDuration);
            mHomeState = HOME_STAND_DOWN_PHASE2;
        }
        break;
    case HOME_STAND_DOWN_PHASE2:
        if(sharedMemory->localTime > mRefTime + 0.5)
        {
            mHomeState = HOME_STAND_DOWN_PHASE3;
        }
        break;
    case HOME_STAND_DOWN_PHASE3:
        {
            double homeHip = 126;
            double homeKnee = -157;
            double timeDuration = 1.5;
            mRefTime = sharedMemory->localTime + timeDuration;
            mCubicTrajectoryGen[0].updateTrajectory(sharedMemory->motorPosition[0], homeHip * D2R, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[1].updateTrajectory(sharedMemory->motorPosition[1], homeKnee * D2R, sharedMemory->localTime, timeDuration);
            mHomeState = HOME_NO_ACT;
        }
        break;
    default:
        break;
    }
}

void JointPDController::setHomeTrajectory()
{
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        mDesiredPosition[index] = mCubicTrajectoryGen[index].getPositionTrajectory(sharedMemory->localTime);
        mDesiredVelocity[index] = mCubicTrajectoryGen[index].getVelocityTrajectory(sharedMemory->localTime);
    }
}

void JointPDController::SetControlInput()
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
