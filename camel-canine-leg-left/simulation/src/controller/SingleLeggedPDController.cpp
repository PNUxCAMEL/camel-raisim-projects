#include "controller/SingleLeggedPDController.hpp"

extern pSHM sharedMemory;


SingleLeggedPDController::SingleLeggedPDController(Robot* robot, double DT)
    : Controller(robot)
    , mDT(DT)
{
    setPDGain(200.0, 2.5);
    mTorque.setZero();
    mPositionError.setZero();
    mVelocityError.setZero();
    mDesiredJointPosition.setZero();
    mDesiredJointVelocity.setZero();
    mPosition.setZero();
    mVelocity.setZero();
    updateState();
    mTrajectoryGenerator.updateTrajectory(mPosition[0], 0.35, mRobot->GetWorldTime(), 1.0);
    mDesiredPosition = 0.0;
    mDesiredVelocity = 0.0;
    mTorqueLimit = 10.0;
}

void SingleLeggedPDController::DoControl()
{
    updateState();
    setTrajectory();
    solveIK();
    computeControlInput();
    setControlInput();
    updateSHM();
}

void SingleLeggedPDController::setTrajectory()
{
    mDesiredPosition = mTrajectoryGenerator.getPositionTrajectory(sharedMemory->localTime);
    mDesiredVelocity = mTrajectoryGenerator.getVelocityTrajectory(sharedMemory->localTime);
}

void SingleLeggedPDController::updateState()
{
    mPosition = mRobot->GetQ();
    mVelocity = mRobot->GetQD();
}

void SingleLeggedPDController::computeControlInput()
{
    for (int i = 1; i < 3; i++)
    {
        mPositionError[i - 1] = mDesiredJointPosition[i - 1] - mPosition[i];
        mVelocityError[i - 1] = mDesiredJointVelocity[i - 1] - mVelocity[i];
        mTorque[i] = mPGain * mPositionError[i - 1] + mDGain * mVelocityError[i - 1];
    }
}

void SingleLeggedPDController::setControlInput()
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

void SingleLeggedPDController::updateSHM()
{
    sharedMemory->localTime = mRobot->GetWorldTime();
    sharedMemory->positionZ = mPosition[0];
    sharedMemory->desiredPositionZ = mDesiredPosition;
    sharedMemory->velocityZ = mVelocity[0];
    sharedMemory->desiredVelocityZ = mDesiredVelocity;
    sharedMemory->jointPosition[0] = mPosition[1];
    sharedMemory->jointPosition[1] = mPosition[2];
    sharedMemory->jointVelocity[0] = mVelocity[1];
    sharedMemory->jointVelocity[1] = mVelocity[2];
    sharedMemory->jointTorque[0] = mTorque[0];
    sharedMemory->jointTorque[1] = mTorque[1];
}

void SingleLeggedPDController::setPDGain(double const PGain, double const DGain)
{
    mPGain = PGain;
    mDGain = DGain;
}

void SingleLeggedPDController::solveIK()
{
    mDesiredJointPosition[0] = acos(mDesiredPosition / 0.46);
    mDesiredJointPosition[1] = -2 * mDesiredJointPosition[0];
    mDesiredJointVelocity[0] = acos(mDesiredVelocity / 0.46);
    mDesiredJointVelocity[1] = -2 * mDesiredJointVelocity[0];
}
