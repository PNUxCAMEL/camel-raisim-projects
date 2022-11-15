//
// Created by hs on 22. 6. 27.
//

#include <convexMPC/MPCController.hpp>

extern pSHM sharedMemory;

MPCController::MPCController(const uint8_t& horizon)
    : mHorizon(horizon)
    , mTorqueLimit(17.0)
    , ConvexMPCSolver(mHorizon)
    , SwingLegTrajectory(0.5)
    , mFirstRunTrot(true)
    , mPgain{30,30,30}
    , mDgain{2,2,2}
/*    , mPgain{20,50,50}
    , mDgain{1,0.5,0.5}*/
{
    mGRF->setZero();
    mTorque->setZero();
    mInitState.setZero();
    mSwingTorque->setZero();
}

void MPCController::InitUpTrajectory()
{
    double timeDuration = 2.0;
    mBaseTrajectory[0].updateTrajectory(sharedMemory->basePosition[0], 0.0,
                                        sharedMemory->localTime, timeDuration);
    mBaseTrajectory[1].updateTrajectory(sharedMemory->basePosition[1], 0.0,
                                        sharedMemory->localTime, timeDuration);
    mBaseTrajectory[2].updateTrajectory(sharedMemory->basePosition[2], 0.35,
                                        sharedMemory->localTime, timeDuration);
}

void MPCController::InitDownTrajectory()
{
    double timeDuration = 2.0;
    mBaseTrajectory[0].updateTrajectory(sharedMemory->basePosition[0], sharedMemory->basePosition[0],
                                        sharedMemory->localTime, timeDuration);
    mBaseTrajectory[1].updateTrajectory(sharedMemory->basePosition[1], sharedMemory->basePosition[1],
                                        sharedMemory->localTime, timeDuration);
    mBaseTrajectory[2].updateTrajectory(sharedMemory->basePosition[2], 0.0,
                                        sharedMemory->localTime, timeDuration);
}

void MPCController::InitSwingTrjactory()
{
    SwingLegTrajectory.UpdateTrajectory(sharedMemory->localTime);
}

void MPCController::DoControl()
{
    updateState();
    ConvexMPCSolver.SetTrajectory(mBaseTrajectory);
    ConvexMPCSolver.GetMetrices(mInitState, mFootPosition);
    ConvexMPCSolver.SolveQP();
    ConvexMPCSolver.GetGRF(mGRF);

    if (sharedMemory->gaitState != STAND)
    {
        if (mFirstRunTrot)
        {
            InitSwingTrjactory();
            mFirstRunTrot = false;
        }
    }
    setLegControl();
    computeControlInput();
    setControlInput();
}

void MPCController::updateState()
{
    for (int idx=0; idx<3; idx++)
    {
        mBasePosition[idx] = sharedMemory->basePosition[idx];
        mBaseVelocity[idx] = sharedMemory->baseVelocity[idx];
        mBaseEulerPosition[idx] = sharedMemory->baseEulerPosition[idx];
        mBaseEulerVelocity[idx] = sharedMemory->baseEulerVelocity[idx];
    }

    for (int leg=0; leg<4; leg++)
    {
        for (int mt=0; mt<3; mt++)
        {
            mFootPosition[leg][mt] = sharedMemory->footPosition[leg][mt];
            mMotorPosition[leg][mt] = sharedMemory->motorPosition[leg*3+mt];
            mMotorVelocity[leg][mt] = sharedMemory->motorVelocity[leg*3+mt];
        }
    }

    mInitState << mBaseEulerPosition[0], mBaseEulerPosition[1], mBaseEulerPosition[2],
            mBasePosition[0], mBasePosition[1], mBasePosition[2],
            mBaseEulerVelocity[0], mBaseEulerVelocity[1], mBaseEulerVelocity[2],
            mBaseVelocity[0], mBaseVelocity[1], mBaseVelocity[2], GRAVITY;
}

void MPCController::setLegControl()
{
    SwingLegTrajectory.GetPositionTrajectory(sharedMemory->localTime, mDesiredPosition);

    double d = sqrt(pow(mDesiredPosition[0],2)+pow(mDesiredPosition[1],2));
    double phi = acos(abs(mDesiredPosition[0])/ d);
    double psi = acos(pow(d,2)/(2*0.23*d));
    if (mDesiredPosition[0] < 0)
        mSwingJointPos[1] = 1.57 - phi + psi;
    else if(mDesiredPosition[0] == 0)
        mSwingJointPos[1] = psi;
    else
        mSwingJointPos[1] = phi + psi - 1.57;
    mSwingJointPos[2] = -acos((pow(d,2)-2*pow(0.23,2)) / (2*0.23*0.23));

    for (int i = 0; i < 4; i++)
    {
        if (sharedMemory->gaitTable[i] == 0)
        {
            for(int j=0; j<3; j++)
            {
                mSwingTorque[i][j] = mPgain[j] * (mSwingJointPos[j] - mMotorPosition[i][j])
                                   + mDgain[j] * (mSwingJointVel[j] - mMotorVelocity[i][j]);
            }
        }
        else
        {
            for(int j=0; j<3; j++)
            {
                mSwingTorque[i][j] = 0.f;
            }
        }
    }
}

void MPCController::computeControlInput()
{
    GetJacobian(mJacobian[0], mMotorPosition[0],1);
    GetJacobian(mJacobian[1], mMotorPosition[1],-1);
    GetJacobian(mJacobian[2], mMotorPosition[2],1);
    GetJacobian(mJacobian[3], mMotorPosition[3],-1);

    for(int idx=0; idx<4; idx++)
    {
        mJacobian[idx].transposeInPlace();
        mTorqueJacobian[idx] = mJacobian[idx]*mGRF[idx];
        mTorque[idx][0] = mTorqueJacobian[idx][0]+mSwingTorque[idx][0];
        mTorque[idx][1] = mTorqueJacobian[idx][1]+mSwingTorque[idx][1];
        mTorque[idx][2] = mTorqueJacobian[idx][2]+mSwingTorque[idx][2];
    }
}

void MPCController::setControlInput()
{
    for (int leg = 0; leg < 4; leg++)
    {
        for (int motor = 0; motor < 3; motor++)
        {
            if (mTorque[leg][motor] > mTorqueLimit)
            {
                mTorque[leg][motor] = mTorqueLimit;
            }
            else if (mTorque[leg][motor] < -mTorqueLimit)
            {
                mTorque[leg][motor] = -mTorqueLimit;
            }
            sharedMemory->motorDesiredTorque[leg*3+motor] = mTorque[leg][motor];
        }
    }
}