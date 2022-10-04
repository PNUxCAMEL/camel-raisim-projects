#include "controller/SingleLeggedIDController.hpp"

extern pSHM sharedMemory;

SingleLeggedIDController::SingleLeggedIDController(Robot* robot, double DT)
    : Controller(robot)
    , mDT(DT)
{
    setPDGain(50.0, 2.5);
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
    mDesiredAcceleration = 0.0;
    mCalculatedForce = 0.0;
    mdz_dth1 = 0.0;
    mdz_dth2 = 0.0;
    mTorqueLimit = 10.0;
    mLumpedMass = 2.009;
    mGravity = -9.81;
}

void SingleLeggedIDController::DoControl()
{
    updateState();
    setTrajectory();
//    solveIK();
    computeControlInput();
    setControlInput();
    updateSHM();
}

void SingleLeggedIDController::setTrajectory()
{
    mDesiredPosition = mTrajectoryGenerator.getPositionTrajectory(sharedMemory->localTime);
    mDesiredVelocity = mTrajectoryGenerator.getVelocityTrajectory(sharedMemory->localTime);
    mDesiredAcceleration = mTrajectoryGenerator.getAccelerationTrajectory(sharedMemory->localTime);
}

void SingleLeggedIDController::updateState()
{
    mPosition = mRobot->GetQ();
    mVelocity = mRobot->GetQD();
}

void SingleLeggedIDController::computeControlInput()
{
    mCalculatedForce = mLumpedMass * (mDesiredAcceleration + mPGain * (mDesiredPosition - mPosition[0]) + mDGain * (mDesiredVelocity - mVelocity[0]) - mGravity);
    mdz_dth1 = -0.23 * sin(mPosition[1]) - 0.23 * sin(mPosition[1] + mPosition[2]);
    mdz_dth2 = -0.23 * sin(mPosition[1] + mPosition[2]);
    mTorque[1] = mdz_dth1 * mCalculatedForce;
    mTorque[2] = mdz_dth2 * mCalculatedForce;
}

void SingleLeggedIDController::setControlInput()
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

void SingleLeggedIDController::updateSHM()
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

void SingleLeggedIDController::setPDGain(double const PGain, double const DGain)
{
    mPGain = PGain;
    mDGain = DGain;
}

void SingleLeggedIDController::solveIK()
{
    mDesiredJointPosition[0] = acos(mDesiredPosition / 0.46);
    mDesiredJointPosition[1] = -2 * mDesiredJointPosition[0];
    mDesiredJointVelocity[0] = acos(mDesiredVelocity / 0.46);
    mDesiredJointVelocity[1] = -2 * mDesiredJointVelocity[0];
}