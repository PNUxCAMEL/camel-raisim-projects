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

void JointPDController::SetPDgain(const double& kp, const double& kd)
{
    for (int motorIdx = 0; motorIdx < MOTOR_NUM; motorIdx++)
    {
        Kp[motorIdx] = kp;
        Kd[motorIdx] = kd;
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
/*
    mBezierTrajectoryGen.InitTrajectorySet(sharedMemory->localTime, 1);
*/
}

void JointPDController::setTrajectory()
{
/*    double d = 0.0;
    double phi = 0.0;
    double psi = 0.0;

    mBezierTrajectoryGen.SetCurrentTime(sharedMemory->localTime);
    for (int idx=0; idx<4; idx++)
    {
        if (sharedMemory->gaitTable[idx] == 0)
        {
            mBezierTrajectoryGen.SwingTrajectory(mDesiredP);
        }
        else
        {
            mBezierTrajectoryGen.StandTrajectory(mDesiredP);
        }

        d = sqrt(pow(mDesiredP[0], 2) + pow(mDesiredP[1], 2));
        phi = acos(abs(mDesiredP[0]) / d);
        psi = acos(pow(d, 2) / (2 * 0.23 * d));

        mDesiredPosition[idx*3+0] = 0.0;

        if (mDesiredP[0] < 0)
        {
            mDesiredPosition[idx*3+1] = 1.57 - phi + psi;
        }
        else if (mDesiredP[0] == 0)
        {
            mDesiredPosition[idx*3+1] = psi;
        }
        else
        {
            mDesiredPosition[idx*3+1] = phi + psi - 1.57;
        }

        mDesiredPosition[idx*3+2] = -acos((pow(d, 2) - 2 * pow(0.23, 2)) / (2 * 0.23 * 0.23));

    }

    for (int index = 0 ; index < MOTOR_NUM ; index++)
    {
        mDesiredVelocity[index] = 0.0;
    }*/
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
