//
// Created by camel on 22. 9. 21.
//

#include <PDcontroller/JointPDController.hpp>

extern pSHM sharedMemory;

JointPDController::JointPDController()
    : mRefTime(0.0)
    , mHomeState(HOME_NO_ACT)
{
    for (int motorIdx = 0; motorIdx < MOTOR_NUM; motorIdx++)
    {
        Kp[motorIdx] = 150.0;
        Kd[motorIdx] = 4.5;
        mTorqueLimit[motorIdx] = 13.0;
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
    solveIK();
    computeControlInput();
    SetControlInput();
}

void JointPDController::DoSineControl()
{
    setSineTrajectory();
    solveIK();
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

void JointPDController::InitSineTrajectory()
{
    mSineTrajectoryGenerator.updateTrajectory(sharedMemory->desiredHipVerticalPosition, sharedMemory->localTime, 0.04, 0.7);
}

void JointPDController::InitCubicTrajectory()
{
    mCubicTrjectoryGenHipVertical.updateTrajectory(sharedMemory->desiredHipVerticalPosition, 0.40, sharedMemory->localTime, 1.0);
}

void JointPDController::InitCubicTrajectory2()
{
    mCubicTrjectoryGenHipVertical.updateTrajectory(sharedMemory->desiredHipVerticalPosition, 0.20, sharedMemory->localTime, 1.0);
}

void JointPDController::setTrajectory()
{
    sharedMemory->desiredHipVerticalPosition = mCubicTrjectoryGenHipVertical.getPositionTrajectory(sharedMemory->localTime);
    sharedMemory->desiredHipVerticalVelocity = mCubicTrjectoryGenHipVertical.getVelocityTrajectory(sharedMemory->localTime);
}

void JointPDController::solveIK()
{
    mDesiredPosition[0] = acos(sharedMemory->desiredHipVerticalPosition / 0.46);
    mDesiredPosition[1] = -2*mDesiredPosition[0];
    mDesiredVelocity[0] = acos(sharedMemory->desiredHipVerticalVelocity / 0.46);
    mDesiredVelocity[1] = -2*mDesiredVelocity[0];
}

void JointPDController::setSineTrajectory()
{
    sharedMemory->desiredHipVerticalPosition = mSineTrajectoryGenerator.getPositionTrajectory(sharedMemory->localTime);
    sharedMemory->desiredHipVerticalVelocity = mSineTrajectoryGenerator.getVelocityTrajectory(sharedMemory->localTime);
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
    switch (mHomeState)
    {
    case HOME_NO_ACT:
        break;
    case HOME_STAND_UP_PHASE1:
    {
        double homeHip = 60;
        double homeKnee = -120;
        double timeDuration = 1.0;

        /*
         * For Jump
         * double homeHip = 70;
         * double homeKnee = -140;
         * double timeDuration = 1.0;
         */
        mRefTime = sharedMemory->localTime + timeDuration;
        mCubicTrajectoryGen[0].updateTrajectory(sharedMemory->motorPosition[0], homeHip * D2R, sharedMemory->localTime, timeDuration);
        mCubicTrajectoryGen[1].updateTrajectory(sharedMemory->motorPosition[1], homeKnee * D2R, sharedMemory->localTime, timeDuration);
        mHomeState = HOME_STAND_UP_PHASE2;
    }
        break;

    case HOME_STAND_UP_PHASE2:
        if (sharedMemory->localTime > mRefTime + 0.5)
        {
            mHomeState = HOME_STAND_UP_PHASE3;
        }
        break;
    case HOME_STAND_UP_PHASE3:
    {
        double homeHip = 45;
        double homeKnee = -90;
        double timeDuration = 1.0;

        /*
         * For Jump
         * double homeHip = 30;
         * double homeKnee = -60;
         * double timeDuration = 0.1;
         */

        mRefTime = sharedMemory->localTime + timeDuration;
        mCubicTrajectoryGen[0].updateTrajectory(mDesiredPosition[0], homeHip * D2R, sharedMemory->localTime, timeDuration);
        mCubicTrajectoryGen[1].updateTrajectory(mDesiredPosition[1], homeKnee * D2R, sharedMemory->localTime, timeDuration);
        mHomeState = HOME_NO_ACT;
    }
        break;
    case HOME_STAND_DOWN_PHASE1:
    {
        double homeHip = 60;
        double homeKnee = -120;
        double timeDuration = 2.0;
        mRefTime = sharedMemory->localTime + timeDuration;
        mCubicTrajectoryGen[0].updateTrajectory(mDesiredPosition[0], homeHip * D2R, sharedMemory->localTime, timeDuration);
        mCubicTrajectoryGen[1].updateTrajectory(mDesiredPosition[1], homeKnee * D2R, sharedMemory->localTime, timeDuration);
        mHomeState = HOME_STAND_DOWN_PHASE2;
    }
        break;
    case HOME_STAND_DOWN_PHASE2:
        if (sharedMemory->localTime > mRefTime + 0.5)
        {
            mHomeState = HOME_STAND_DOWN_PHASE3;
        }
        break;
    case HOME_STAND_DOWN_PHASE3:
    {
        double homeHip = 80;
        double homeKnee = -160;
        double timeDuration = 1.5;
        mRefTime = sharedMemory->localTime + timeDuration;
        mCubicTrajectoryGen[0].updateTrajectory(mDesiredPosition[0], homeHip * D2R, sharedMemory->localTime, timeDuration);
        mCubicTrajectoryGen[1].updateTrajectory(mDesiredPosition[1], homeKnee * D2R, sharedMemory->localTime, timeDuration);
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
    double pastDesiredHipVerticalPosition = sharedMemory->desiredHipVerticalPosition;
    sharedMemory->desiredHipVerticalPosition = 0.23 * cos(mDesiredPosition[0])+0.23 * cos(mDesiredPosition[0]+mDesiredPosition[1]);
    sharedMemory->desiredHipVerticalVelocity = (sharedMemory->desiredHipVerticalPosition - pastDesiredHipVerticalPosition) / CONTROL_dT;
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
