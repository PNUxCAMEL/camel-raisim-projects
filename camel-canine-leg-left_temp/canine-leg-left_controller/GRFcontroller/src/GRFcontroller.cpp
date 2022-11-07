//
// Created by jh on 22. 11. 7.
//

#include <GRFcontroller/GRFcontroller.hpp>

extern pSHM sharedMemory;

GRFcontroller::GRFcontroller()
: Kp(10.0)
, Ki(0.0)
, mIntegratedError(0.0)
{
    mJacobian.setZero();
    mTorque.setZero();
    for (int motorIdx = 0; motorIdx < MOTOR_NUM; motorIdx++)
    {
        mTorqueLimit[motorIdx] = 11.0;
    }
}

void GRFcontroller::DoControl()
{
    computeControlInput();
    setControlInput();
}

void GRFcontroller::computeControlInput()
{
    Eigen::VectorXd calculatedForce = Eigen::VectorXd(2);
    double dz_dth1;
    double dz_dth2;
    double errorGRF;
    double calculatedGRF;
    dz_dth1 = -0.23 * sin(sharedMemory->motorPosition[0]) - 0.23 * sin(sharedMemory->motorPosition[0] + sharedMemory->motorPosition[1]);
    dz_dth2 = -0.23 * sin(sharedMemory->motorPosition[0] + sharedMemory->motorPosition[1]);
    mJacobian << 0.0, 0.0, dz_dth1, dz_dth2;
//    errorGRF = sharedMemory->desiredGRF - sharedMemory->measuredGRF;
    errorGRF = sharedMemory->desiredGRF - sharedMemory->estimatedGRFMLP;
//    errorGRF = sharedMemory->desiredGRF - sharedMemory->estimatedGRFSMO;
    mIntegratedError += errorGRF * MPC_dT;
    calculatedGRF = sharedMemory->desiredGRF + 0.5*errorGRF + 2.0*mIntegratedError;
    calculatedForce << 0.0, calculatedGRF;
    mTorque = mJacobian.transpose() * calculatedForce;
}

void GRFcontroller::setControlInput()
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