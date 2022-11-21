//
// Created by jh on 22. 11. 7.
//

#include <MPCcontroller/MPCcontroller.hpp>
#include <iostream>
#include <unistd.h>

extern pSHM sharedMemory;

MPCcontroller::MPCcontroller()
    : mIteration(0)
    , mMaximumIteration(100)
    , mTerminateCondition(1e-7)
    , mDelta(1e-3)
    , mStepSize(500.0 / MPC_dT)
    , mInitialPosition(0.0)
    , mInitialVelocity(0.0)
{
    mJacobian.setZero();
    mTorque.setZero();
    mForce.setOnes();
    mForceTemp.setZero();
    mForce = mForce * LUMPED_MASS * GRAVITY * (-1.0);
    mA << 1.0, MPC_dT, 0.0, 1.0;
    mB << 0.0, MPC_dT / LUMPED_MASS;
    mC << 0.0, GRAVITY;
    mQ << 1.5, 0.0, 0.0, 1e-3;
    mR << 1e-3 * MPC_dT * MPC_dT;
    for (int motorIdx = 0; motorIdx < MOTOR_NUM; motorIdx++)
    {
        mTorqueLimit[motorIdx] = 3.5;
    }
}

void MPCcontroller::DoControl()
{
    resetMPCVariables();
    setSineTrajectory();
    solve();
    computeControlInput();
    SetControlInput();
}

void MPCcontroller::InitSineTrajectory()
{
    mSineTrajectoryGenerator.updateTrajectory(sharedMemory->hipVerticalPosition, sharedMemory->localTime, 0.05, 0.5);
}

void MPCcontroller::SetControlInput()
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

void MPCcontroller::setSineTrajectory()
{
    for (int i = 0; i < MPC_HORIZON; i++)
    {
        mTrajectorySequence(0, i) = mSineTrajectoryGenerator.getPositionTrajectory(sharedMemory->localTime + MPC_dT * i);
        mTrajectorySequence(1, i) = mSineTrajectoryGenerator.getVelocityTrajectory(sharedMemory->localTime + MPC_dT * i);
    }
    sharedMemory->desiredHipVerticalPosition = mTrajectorySequence(0, 0);
    sharedMemory->desiredHipVerticalVelocity = mTrajectorySequence(1, 0);
}

void MPCcontroller::computeControlInput()
{
    Eigen::VectorXd calculatedForce = Eigen::VectorXd(2);
    double Fz;
    double dz_dth1;
    double dz_dth2;
    Fz = mForce(0);
    calculatedForce << 0.0, Fz;
    dz_dth1 = -LINK2_LENGTH * sin(sharedMemory->motorPosition[0]) - LINK1_LENGTH * sin(sharedMemory->motorPosition[0] + sharedMemory->motorPosition[1]);
    dz_dth2 = -LINK1_LENGTH * sin(sharedMemory->motorPosition[0] + sharedMemory->motorPosition[1]);
    mJacobian << 0.0, 0.0, dz_dth1, dz_dth2;
    mTorque = mJacobian.transpose() * calculatedForce;
    sharedMemory->desiredGRF = Fz;
}

void MPCcontroller::solve()
{
    mInitialPosition = sharedMemory->hipVerticalPosition;
    mInitialVelocity = sharedMemory->hipVerticalVelocity;
    while (true)
    {
        mIteration++;
        updateMPCStates();
        computeGradient();
        updateForces();

        if (isTerminateCondition())
        {
//            std::cout << "iteration : " << mIteration << std::endl;
//            std::cout << "mTrajectorySequence : \n" << mTrajectorySequence << std::endl;
//            std::cout << "mNextStates : \n" << mNextStates << "\n\n" << std::endl;
            break;
        }
    }
}

void MPCcontroller::updateMPCStates()
{
    for (int i = 0; i < MPC_HORIZON; i++)
    {
        if (i == 0)
        {
            mNextStates(0, i) = mInitialPosition + MPC_dT * mInitialVelocity;
            mNextStates(1, i) = mInitialVelocity + MPC_dT * (mForce(i) / LUMPED_MASS + GRAVITY);
        }
        else if (i == 1)
        {
            mNextStates(0, i) = mNextStates(0, i - 1) + MPC_dT * mInitialVelocity + MPC_dT * MPC_dT * (mForce(i - 1) / LUMPED_MASS + GRAVITY);
            mNextStates(1, i) = mNextStates(1, i - 1) + MPC_dT * (mForce(i) / LUMPED_MASS + GRAVITY);
        }
        else
        {
            mNextStates(0, i) = mNextStates(0, i - 1) + MPC_dT * mNextStates(1, i - 2) + MPC_dT * MPC_dT * (mForce(i - 1) / LUMPED_MASS + GRAVITY);
            mNextStates(1, i) = mNextStates(1, i - 1) + MPC_dT * (mForce(i) / LUMPED_MASS + GRAVITY);
        }
    }
}

void MPCcontroller::updateMPCStatesTemp(const Eigen::VectorXd force)
{
    for (int i = 0; i < MPC_HORIZON; i++)
    {
        if (i == 0)
        {
            mNextStatesTemp(0, i) = mInitialPosition + MPC_dT * mInitialVelocity;
            mNextStatesTemp(1, i) = mInitialVelocity + MPC_dT * (force(i) / LUMPED_MASS + GRAVITY);
        }
        else if (i == 1)
        {
            mNextStatesTemp(0, i) = mNextStatesTemp(0, i - 1) + MPC_dT * mInitialVelocity + MPC_dT * MPC_dT * (force(i - 1) / LUMPED_MASS + GRAVITY);
            mNextStatesTemp(1, i) = mNextStatesTemp(1, i - 1) + MPC_dT * (force(i) / LUMPED_MASS + GRAVITY);
        }
        else
        {
            mNextStatesTemp(0, i) = mNextStatesTemp(0, i - 1) + MPC_dT * mNextStatesTemp(1, i - 2) + MPC_dT * MPC_dT * (force(i - 1) / LUMPED_MASS + GRAVITY);
            mNextStatesTemp(1, i) = mNextStatesTemp(1, i - 1) + MPC_dT * (force(i) / LUMPED_MASS + GRAVITY);
        }
    }
}

void MPCcontroller::computeGradient()
{
    double functionValue = objectiveFunction(mForce);
    for (int i = 0; i < MPC_HORIZON; i++)
    {
        mForceTemp = mForce;
        mForceTemp(i) += mDelta;
        mGradient(i) = (objectiveFunction(mForceTemp) - functionValue) / mDelta;
    }
    mRMSGradient = pow(mGradient.dot(mGradient), 0.5);
}

void MPCcontroller::updateForces()
{
    for (int i = 0; i < MPC_HORIZON; i++)
    {
        mForce(i) -= mStepSize * mGradient(i);
    }
}

void MPCcontroller::resetMPCVariables()
{
    mIteration = 0;
//    mForce.setOnes();
//    mForce = mForce * LUMPED_MASS * GRAVITY * (-1);
    mRMSGradient = 10;
}

bool MPCcontroller::isTerminateCondition()
{
    if (mIteration > mMaximumIteration)
    {
//        std::cout << "maximum iteration" << std::endl;
        return true;
    }
    else if (mRMSGradient < mTerminateCondition)
    {
//        std::cout << "terminate condition" << std::endl;
        return true;
    }
    else
    {
        return false;
    }
}

double MPCcontroller::objectiveFunction(const Eigen::VectorXd force)
{
    mObjFunctionValue = 0.0;
    updateMPCStatesTemp(force);

    for (int i = 0; i < MPC_HORIZON; i++)
    {
        /* It was slow.. bcz of the following reasons
         * primary reason : transpose and multiply of matrix
         * secondary reason : double call of matrix component
         */

        /* Previous Code
           mNextX(0) = mNextStatesTemp(0, i);
           mNextX(1) = mNextStatesTemp(1, i);
           mNextXDes(0) = mTrajectorySequence(0, i);
           mNextXDes(1) = mTrajectorySequence(1, i);
           mObjFunctionValue += ((mNextX - mNextXDes).transpose()*mQ*(mNextX - mNextXDes) + mForce(i)*mR*mForce(i))(0);
        */

        /* Improved Code*/
        tempValue1 = mNextStatesTemp(0, i) - mTrajectorySequence(0, i);
        tempValue2 = mNextStatesTemp(1, i) - mTrajectorySequence(1, i);
        mObjFunctionValue += tempValue1 * tempValue1 * mQ(0, 0);
        mObjFunctionValue += tempValue2 * tempValue2 * mQ(1, 1);
        mObjFunctionValue += mForce(i) * mR(0) * mForce(i);
    }
    return mObjFunctionValue;
}
