#include "controller/SingleLeggedMPCController.hpp"

extern pSHM sharedMemory;

SingleLeggedMPCController::SingleLeggedMPCController(Robot* robot, double DT, int MPCHorizon)
    : Controller(robot)
    , mDT(DT)
    , mMPCHorizon(MPCHorizon)
{
    mTorque.setZero();
    mPosition.setZero();
    mVelocity.setZero();
    updateState();
    mTrajectoryGenerator.updateTrajectory(mPosition[0], mRobot->GetWorldTime(), 0.1, 0.2);
    mCalculatedForce = 0.0;
    mdz_dth1 = 0.0;
    mdz_dth2 = 0.0;
    mTorqueLimit = 10.0;
    mLumpedMass = 2.009;
    mGravity = -9.81;

    mMaximumIteration = 100;
    mTerminateCondition = 1e-7;
    mDelta = 1e-3;
    mStepSize = 100.0 / mDT / mDT;
    mA << 1.0, mDT, 0.0, 1.0;
    mB << 0.0, mDT / mLumpedMass;
    mC << 0.0, mGravity;
    mQ << 1.5, 0.0, 0.0, 1e-3;
    mR << 1e-3 * mDT * mDT;
    mIteration = 0;
    mForce.setOnes();
    mForce = mForce * mLumpedMass * mGravity * (-1);
}

void SingleLeggedMPCController::DoControl()
{
    updateState();
    setTrajectory();
    solve();
    computeControlInput();
    setControlInput();
    resetMPCVariables();
    updateSHM();
}

void SingleLeggedMPCController::setTrajectory()
{
    double currentTime = mRobot->GetWorldTime();
    for (int i = 0; i < mMPCHorizon; i++)
    {
        mTrajectorySequence(0, i) = mTrajectoryGenerator.getPositionTrajectory(currentTime + mDT * i);
        mTrajectorySequence(1, i) = mTrajectoryGenerator.getVelocityTrajectory(currentTime + mDT * i);
    }
}

void SingleLeggedMPCController::updateState()
{
    mPosition = mRobot->GetQ();
    mVelocity = mRobot->GetQD();
    mInitialPosition = mPosition[0];
    mInitialVelocity = mVelocity[0];
}

void SingleLeggedMPCController::computeControlInput()
{
    mCalculatedForce = mForce(0);
    mdz_dth1 = -0.23 * sin(mPosition[1]) - 0.23 * sin(mPosition[1] + mPosition[2]);
    mdz_dth2 = -0.23 * sin(mPosition[1] + mPosition[2]);
    mTorque[1] = mdz_dth1 * mCalculatedForce;
    mTorque[2] = mdz_dth2 * mCalculatedForce;
}

void SingleLeggedMPCController::setControlInput()
{
    for (int i = 0; i < 3; i++)
    {
        if (mTorque[i] > mTorqueLimit)
        {
            mTorque[i] = mTorqueLimit;
        }
        else if (mTorque[i] < -mTorqueLimit)
        {
            mTorque[i] = -mTorqueLimit;
        }
    }
    mRobot->SetTau(mTorque);
}

void SingleLeggedMPCController::updateSHM()
{
    sharedMemory->localTime = mRobot->GetWorldTime();
    sharedMemory->positionZ = mPosition[0];
    sharedMemory->desiredPositionZ = mTrajectorySequence(0, 0);
    sharedMemory->velocityZ = mVelocity[0];
    sharedMemory->desiredVelocityZ = mTrajectorySequence(1, 0);
    sharedMemory->jointPosition[0] = mPosition[1];
    sharedMemory->jointPosition[1] = mPosition[2];
    sharedMemory->jointVelocity[0] = mVelocity[1];
    sharedMemory->jointVelocity[1] = mVelocity[2];
    sharedMemory->jointTorque[0] = mTorque[0];
    sharedMemory->jointTorque[1] = mTorque[1];
}

void SingleLeggedMPCController::solve()
{
    while (true)
    {
        mIteration++;
        updateMPCStates();
        computeGradient();
        updateForces();
        if (isTerminateCondition())
        {
//            std::cout<<"iteration : "<<mIteration<<std::endl;
//            std::cout<<"mNextStates : \n"<<mNextStates<<"\n\n"<<std::endl;
            break;
        }
    }
}

void SingleLeggedMPCController::updateMPCStates()
{
    for (int i = 0; i < mMPCHorizon; i++)
    {
        if (i == 0)
        {
            mNextStates(0, i) = mInitialPosition + mDT * mInitialVelocity;
            mNextStates(1, i) = mInitialVelocity + mDT * (mForce(i) / mLumpedMass + mGravity);
        }
        else if (i == 1)
        {
            mNextStates(0, i) = mNextStates(0, i - 1) + mDT * mInitialVelocity + mDT * mDT * (mForce(i - 1) / mLumpedMass + mGravity);
            mNextStates(1, i) = mInitialVelocity + mDT * (mForce(i) / mLumpedMass + mGravity);
        }
        else
        {
            mNextStates(0, i) = mNextStates(0, i - 1) + mDT * mNextStates(1, i - 2) + mDT * mDT * (mForce(i - 1) / mLumpedMass + mGravity);
            mNextStates(1, i) = mInitialVelocity + mDT * (mForce(i) / mLumpedMass + mGravity);
        }
    }
}

void SingleLeggedMPCController::updateMPCStatesTemp(Eigen::VectorXd const force)
{
    for (int i = 0; i < mMPCHorizon; i++)
    {
        if (i == 0)
        {
            mNextStatesTemp(0, i) = mInitialPosition + mDT * mInitialVelocity;
            mNextStatesTemp(1, i) = mInitialVelocity + mDT * (force(i) / mLumpedMass + mGravity);
        }
        else if (i == 1)
        {
            mNextStatesTemp(0, i) = mNextStatesTemp(0, i - 1) + mDT * mInitialVelocity + mDT * mDT * (force(i - 1) / mLumpedMass + mGravity);
            mNextStatesTemp(1, i) = mInitialVelocity + mDT * (force(i) / mLumpedMass + mGravity);
        }
        else
        {
            mNextStatesTemp(0, i) = mNextStates(0, i - 1) + mDT * mNextStatesTemp(1, i - 2) + mDT * mDT * (force(i - 1) / mLumpedMass + mGravity);
            mNextStatesTemp(1, i) = mInitialVelocity + mDT * (force(i) / mLumpedMass + mGravity);
        }
    }
}

void SingleLeggedMPCController::computeGradient()
{

    double functionValue = objectiveFunction(mForce);

    for (int i = 0; i < mMPCHorizon; i++)
    {
        mForceTemp = mForce;
        mForceTemp(i) += mDelta;
        mGradient[i] = (objectiveFunction(mForceTemp) - functionValue) / mDelta;
    }
    mRMSGradient = pow(mGradient.dot(mGradient), 0.5);
}

void SingleLeggedMPCController::updateForces()
{
    for (int i = 0; i < mMPCHorizon; i++)
    {
        mForce(i) -= mStepSize * mGradient(i);
    }
}

void SingleLeggedMPCController::resetMPCVariables()
{
    mIteration = 0;
    mForce.setOnes();
    mForce = mForce * mLumpedMass * mGravity * (-1);
    mRMSGradient = 10;
}

bool SingleLeggedMPCController::isTerminateCondition()
{
    if (mIteration > mMaximumIteration)
    {
//        std::cout<<"maximum iteration"<<std::endl;
        return true;
    }
    else if (mRMSGradient < mTerminateCondition)
    {
//        std::cout<<"terminate condition"<<std::endl;
        return true;
    }
    else
    {
        return false;
    }
}

double SingleLeggedMPCController::objectiveFunction(Eigen::VectorXd const force)
{
    mObjFunctionValue = 0.0;
    updateMPCStatesTemp(force);

    for (int i = 0; i < mMPCHorizon; i++)
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
