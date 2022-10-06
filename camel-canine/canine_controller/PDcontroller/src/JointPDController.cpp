//
// Created by camel on 22. 9. 21.
//

#include <PDcontroller/JointPDController.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

JointPDController::JointPDController()
{
    mIteration = 0;
    for(int motorIdx = 0; motorIdx < MOTOR_NUM ; motorIdx++)
    {
        Kp[motorIdx] = 80.0;
        Kd[motorIdx] = 1.5;
        mTorqueLimit[motorIdx] = 11.0;
    }
}

void JointPDController::DoHomeControl()
{
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        mTorque[index] =
                Kp[index] * (mCubicTrajectoryGen[index].getPositionTrajectory(sharedMemory->localTime) - sharedMemory->motorPosition[index])
                + Kd[index] * (mCubicTrajectoryGen[index].getVelocityTrajectory(sharedMemory->localTime) - sharedMemory->motorVelocity[index]);
    }
    setControlInput();
}

void JointPDController::DoPDControl()
{
    setTrajectory();
    computeControlInput();
    setControlInput();
}

void JointPDController::SetPDGain(double* Kp, double* Kd)
{
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        this->Kp[index] = Kp[index];
        this->Kd[index] = Kd[index];
    }
}

void JointPDController::InitHomeTrajectory()
{
    double timeDuration = 2.0;
    double homeHip = 55.0;
    double homeKnee = -90.0;
    mCubicTrajectoryGen[0].updateTrajectory(sharedMemory->motorPosition[LFHR_IDX], 0.0, sharedMemory->localTime, timeDuration);
    mCubicTrajectoryGen[1].updateTrajectory(sharedMemory->motorPosition[LFHP_IDX], homeHip * D2R, sharedMemory->localTime, timeDuration);
    mCubicTrajectoryGen[2].updateTrajectory(sharedMemory->motorPosition[LFKP_IDX], homeKnee * D2R, sharedMemory->localTime, timeDuration);
    mCubicTrajectoryGen[3].updateTrajectory(sharedMemory->motorPosition[RFHR_IDX], 0.0, sharedMemory->localTime, timeDuration);
    mCubicTrajectoryGen[4].updateTrajectory(sharedMemory->motorPosition[RFHP_IDX], homeHip * D2R, sharedMemory->localTime, timeDuration);
    mCubicTrajectoryGen[5].updateTrajectory(sharedMemory->motorPosition[RFKP_IDX], homeKnee * D2R, sharedMemory->localTime, timeDuration);
    mCubicTrajectoryGen[6].updateTrajectory(sharedMemory->motorPosition[LBHR_IDX], 0.0, sharedMemory->localTime, timeDuration);
    mCubicTrajectoryGen[7].updateTrajectory(sharedMemory->motorPosition[LBHP_IDX], homeHip * D2R, sharedMemory->localTime, timeDuration);
    mCubicTrajectoryGen[8].updateTrajectory(sharedMemory->motorPosition[LBKP_IDX], homeKnee * D2R, sharedMemory->localTime, timeDuration);
    mCubicTrajectoryGen[9].updateTrajectory(sharedMemory->motorPosition[RBHR_IDX], 0.0, sharedMemory->localTime, timeDuration);
    mCubicTrajectoryGen[10].updateTrajectory(sharedMemory->motorPosition[RBHP_IDX], homeHip * D2R, sharedMemory->localTime, timeDuration);
    mCubicTrajectoryGen[11].updateTrajectory(sharedMemory->motorPosition[RBKP_IDX], homeKnee * D2R, sharedMemory->localTime, timeDuration);
}

void JointPDController::InitSwingTrajectory()
{
    mBezierTrajectoryGen.updateTrajectory(sharedMemory->localTime, 1);
}

void JointPDController::setTrajectory()
{
    mBezierTrajectoryGen.getPositionTrajectory(sharedMemory->localTime);
    mDesiredP[0] = mBezierTrajectoryGen.sumX;
    mDesiredP[1] = mBezierTrajectoryGen.sumZ;

    double d = sqrt(pow(mDesiredP[0], 2) + pow(mDesiredP[1], 2));
    double phi = acos(abs(mDesiredP[0]) / d);
    double psi = acos(pow(d, 2) / (2 * 0.23 * d));

    if (mDesiredP[0] < 0)
    {
        mDesiredPosition[0] = 1.57 - phi + psi;
    }
    else if (mDesiredP[0] == 0)
    {
        mDesiredPosition[0] = psi;
    }
    else
    {
        mDesiredPosition[0] = phi + psi - 1.57;
    }
    mDesiredPosition[1] = -acos((pow(d, 2) - 2 * pow(0.23, 2)) / (2 * 0.23 * 0.23));

    mDesiredVelocity[0] = 0.0;
    mDesiredVelocity[1] = 0.0;
}

void JointPDController::computeControlInput()
{
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        mTorque[index] = Kp[index] * (mDesiredPosition[index] - sharedMemory->motorPosition[index])
            + Kd[index] * (mDesiredVelocity[index] - sharedMemory->motorVelocity[index]);
    }
}

void JointPDController::setControlInput()
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
