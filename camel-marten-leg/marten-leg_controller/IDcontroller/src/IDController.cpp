//
// Created by jh on 22. 11. 3.
//

#include <IDcontroller/IDController.hpp>

extern pSHM sharedMemory;

IDController::IDController()
    : mDesiredPosition(0.0)
    , mDesiredVelocity(0.0)
    , mDesiredAcceleration(0.0)
    , Kp(50.0)
    , Kd(2.5)
    , mSign(1)
{
    mJacobian.setZero();
    mTorque.setZero();
    mForce.setZero();
    for (int motorIdx = 0; motorIdx < MOTOR_NUM; motorIdx++)
    {
        mTorqueLimit[motorIdx] = 3.5;
    }
}

void IDController::DoControl()
{
    setQuinticTrajectory();
    computeControlInput();
    SetControlInput();
}

void IDController::InitQuinticTrajectory()
{
    mQuinticTrajectoryGen.updateTrajectory(sharedMemory->desiredHipVerticalPosition, 0.2 + mSign * 0.1, sharedMemory->localTime, 0.5);
    mSign *= -1;
}

void IDController::SetControlInput()
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

void IDController::computeControlInput()
{
    Eigen::VectorXd calculatedForce = Eigen::VectorXd(2);
    double Fz;
    double dz_dth1;
    double dz_dth2;
    Fz = LUMPED_MASS * (mDesiredAcceleration + Kp * (mDesiredPosition - sharedMemory->hipVerticalPosition)
        + Kd * (mDesiredVelocity - sharedMemory->hipVerticalVelocity) - GRAVITY);
    calculatedForce << 0.0, Fz;
    dz_dth1 = -LINK2_LENGTH * sin(sharedMemory->motorPosition[0]) - LINK1_LENGTH * sin(sharedMemory->motorPosition[0] + sharedMemory->motorPosition[1]);
    dz_dth2 = -LINK1_LENGTH * sin(sharedMemory->motorPosition[0] + sharedMemory->motorPosition[1]);
    mJacobian << 0.0, 0.0, dz_dth1, dz_dth2;
    mTorque = mJacobian.transpose() * calculatedForce;
}

void IDController::setQuinticTrajectory()
{
    mDesiredPosition = mQuinticTrajectoryGen.getPositionTrajectory(sharedMemory->localTime);
    mDesiredVelocity = mQuinticTrajectoryGen.getVelocityTrajectory(sharedMemory->localTime);
    mDesiredAcceleration = mQuinticTrajectoryGen.getAccelerationTrajectory(sharedMemory->localTime);
    sharedMemory->desiredHipVerticalPosition = mDesiredPosition;
    sharedMemory->desiredHipVerticalVelocity = mDesiredVelocity;
}
