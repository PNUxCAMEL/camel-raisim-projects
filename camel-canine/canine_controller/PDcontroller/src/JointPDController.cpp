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
    double homeHip = 45.0;
    double homeKnee = -80.0;
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
    mBezierTrajectoryGen.updateTrajectory(sharedMemory->localTime, 0.7);
}

void JointPDController::setTrajectory()
{
    mBezierTrajectoryGen.getPositionTrajectory(sharedMemory->localTime);
    mDesiredP[0] = mBezierTrajectoryGen.sumX; //hip
    mDesiredP[1] = mBezierTrajectoryGen.sumZ; //knee

    double d = sqrt(pow(mDesiredP[0], 2) + pow(mDesiredP[1], 2));
    double phi = acos(abs(mDesiredP[0]) / d);
    double psi = acos(pow(d, 2) / (2 * 0.23 * d));

    if (mDesiredP[0] < 0)
    {
        mDesiredPosition[LFHP_IDX] = 1.57 - phi + psi;
        mDesiredPosition[RFHP_IDX] = 1.57 - phi + psi;
        mDesiredPosition[LBHP_IDX] = 1.57 - phi + psi;
        mDesiredPosition[RBHP_IDX] = 1.57 - phi + psi;
    }
    else if (mDesiredP[0] == 0)
    {
        mDesiredPosition[LFHP_IDX] = psi;
        mDesiredPosition[RFHP_IDX] = psi;
        mDesiredPosition[LBHP_IDX] = psi;
        mDesiredPosition[RBHP_IDX] = psi;
    }
    else
    {
        mDesiredPosition[LFHP_IDX] = phi + psi - 1.57;
        mDesiredPosition[RFHP_IDX] = phi + psi - 1.57;
        mDesiredPosition[LBHP_IDX] = phi + psi - 1.57;
        mDesiredPosition[RBHP_IDX] = phi + psi - 1.57;
    }

    mDesiredPosition[LFKP_IDX] = -acos((pow(d, 2) - 2 * pow(0.23, 2)) / (2 * 0.23 * 0.23));
    mDesiredPosition[RFKP_IDX] = -acos((pow(d, 2) - 2 * pow(0.23, 2)) / (2 * 0.23 * 0.23));
    mDesiredPosition[LBKP_IDX] = -acos((pow(d, 2) - 2 * pow(0.23, 2)) / (2 * 0.23 * 0.23));
    mDesiredPosition[RBKP_IDX] = -acos((pow(d, 2) - 2 * pow(0.23, 2)) / (2 * 0.23 * 0.23));

    mDesiredPosition[LFHR_IDX] = 0.0;
    mDesiredPosition[RFHR_IDX] = 0.0;
    mDesiredPosition[LBHR_IDX] = 0.0;
    mDesiredPosition[RBHR_IDX] = 0.0;

    for(int index = 0 ; index < MOTOR_NUM ; index++)
    {
        mDesiredVelocity[index] = 0.0;
    }
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
