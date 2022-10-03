//
// Created by camel on 22. 9. 21.
//

#include <PDcontroller/JointPDController.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

void JointPDController::controllerFunction()
{
    sharedMemory->localTime = mIteration * CONTROL_dT;
    mIteration++;
    switch(sharedMemory->controlState)
    {
        case STATE_CONTROL_STOP:
        {
            break;
        }
        case STATE_MOTOR_OFF:
        {
            mCan->turnOffMotor();
            sharedMemory->controlState = STATE_CONTROL_STOP;
            break;
        }
        case STATE_READY:
        {
            mCan->readMotorErrorStatus();
            for(int index = 0; index < MOTOR_NUM; index++)
            {
                mTorque[index] = 0;
            }
            setControlInput();
            break;
        }
        case STATE_HOME_READY:
        {
            /*mCubicTrajectoryGen[0].updateTrajectory(sharedMemory->motorPosition[HIP_IDX], 45.0 * D2R, sharedMemory->localTime, 1.0);
            mCubicTrajectoryGen[1].updateTrajectory(sharedMemory->motorPosition[KNEE_IDX], -90.0 * D2R, sharedMemory->localTime, 1.0);*/
            mCubicTrajectoryGen[0].updateTrajectory(sharedMemory->motorPosition[HIP_IDX], 0.881691, sharedMemory->localTime, 1.0);
            mCubicTrajectoryGen[1].updateTrajectory(sharedMemory->motorPosition[KNEE_IDX], -1.11338, sharedMemory->localTime, 1.0);
            sharedMemory->controlState = STATE_HOME_CONTROL;
            break;
        }
        case STATE_HOME_CONTROL:
        {
            mCan->readMotorErrorStatus();
            doHomeControl();
            break;
        }
        case STATE_PD_READY:
        {
            mBezierTrajectoryGen.updateTrajectory(sharedMemory->localTime, 1);
            sharedMemory->controlState = STATE_PD_CONTROL;
            break;
        }
        case STATE_PD_CONTROL:
        {
            mCan->readMotorErrorStatus();
            doPDControl();
            break;
        }
        default:
            break;
    }
}

void JointPDController::setPDGain(double *Kp, double *Kd)
{
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        this->Kp[index] = Kp[index];
        this->Kd[index] = Kd[index];
    }
}

void JointPDController::setTrajectory()
{
    mBezierTrajectoryGen.getPositionTrajectory(sharedMemory->localTime);
    mDesiredP[0] = mBezierTrajectoryGen.sumX;
    mDesiredP[1] = mBezierTrajectoryGen.sumZ;

    double d = sqrt(pow(mDesiredP[0],2)+pow(mDesiredP[1],2));
    double phi = acos(abs(mDesiredP[0])/ d);
    double psi = acos(pow(d,2)/(2*0.23*d));

    if (mDesiredP[0] < 0)
        mDesiredPosition[0] = 1.57 - phi + psi;
    else if(mDesiredP[0] == 0)
        mDesiredPosition[0] = psi;
    else
        mDesiredPosition[0] = phi + psi - 1.57;
    mDesiredPosition[1] = -acos((pow(d,2)-2*pow(0.23,2)) / (2*0.23*0.23));

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
        if (mTorque[index] > mTorqueLimit[index])
        {
            mTorque[index] = mTorqueLimit[index];
        }
        else if (mTorque[index] < -mTorqueLimit[index])
        {
            mTorque[index] = -mTorqueLimit[index];
        }
    }
    mCan->setTorque(mTorque);
}

void JointPDController::doHomeControl()
{
    for(int index = 0; index < MOTOR_NUM; index++)
    {
        mTorque[index] =
                Kp[index] * (mCubicTrajectoryGen[index].getPositionTrajectory(sharedMemory->localTime) - sharedMemory->motorPosition[index])
                + Kd[index] * (mCubicTrajectoryGen[index].getVelocityTrajectory(sharedMemory->localTime) - sharedMemory->motorVelocity[index]);
    }
    setControlInput();
}

void JointPDController::doPDControl()
{
    setTrajectory();
    computeControlInput();
    setControlInput();
}