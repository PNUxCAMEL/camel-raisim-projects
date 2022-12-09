//
// Created by hs on 22. 10. 27.
//

#include <WBController/LowPDcontrol.hpp>
#include <iostream>

extern pSHM sharedMemory;

LowPDcontrol::LowPDcontrol()
    : mTorqueLimit(30)
    , SwingLegTrajectory(GAIT_PERIOD/2)
    , mFirstRunTrot(true)
    , mSwingPgain{30,30,30}
    , mSwingDgain{2,2,2}
    , mStandPgain{30,5,5}
    , mStandDgain{1,1,1}
//    , mSwingPgain{30,30,30}
//    , mSwingDgain{1,1,1}
//    , mStandPgain{5,5,5}
//    , mStandDgain{1,1,1}
{
    mTorque->setZero();
}

void LowPDcontrol::DoControl()
{
    updateState();

    if (sharedMemory->gaitState != STAND)
    {
        if (mFirstRunTrot)
        {
            InitSwingTrjactory();
            mFirstRunTrot = false;
        }
    }
    setLegControl();
    setControlInput();
}

void LowPDcontrol::InitSwingTrjactory()
{
    SwingLegTrajectory.UpdateTrajectory(sharedMemory->localTime);
}

void LowPDcontrol::updateState()
{
    for (int leg=0; leg<4; leg++)
    {
        for (int mt=0; mt<3; mt++)
        {
            mMotorPosition[leg][mt] = sharedMemory->motorPosition[leg*3+mt];
            mMotorVelocity[leg][mt] = sharedMemory->motorVelocity[leg*3+mt];
        }
    }
}

void LowPDcontrol::getJointPos(const double& x, const double& z, Vec3<double>& pos)
{
    double d = sqrt(pow(x,2)+pow(z,2));
    double phi = acos(abs(x)/ d);
    double psi = acos(pow(d,2)/(2*LEN_THI*d));

    if (x < 0)
        pos[1] = 1.57 - phi + psi;
    else if(x == 0)
        pos[1] = psi;
    else
        pos[1] = phi + psi - 1.57;
    pos[2] = -acos((pow(d,2)-2*pow(LEN_CAL,2)) / (2*LEN_CAL*LEN_CAL));
}

void LowPDcontrol::setLegControl()
{
    getJointPos(0.0, sharedMemory->baseDesiredPosition[2], mStandJointPos);

    for (int i = 0; i < 4; i++)
    {
        if (sharedMemory->gaitTable[i] == 0)
        {
            SwingLegTrajectory.GetPositionTrajectory(sharedMemory->localTime, mDesiredPosition, i);
            GetLegInvKinematics(mSwingJointPos, mDesiredPosition, i);

            for(int j=0; j<3; j++)
            {
                mLegTorque[i][j] = mSwingPgain[j] * (mSwingJointPos[j] - mMotorPosition[i][j])
                                   + mSwingDgain[j] * (mSwingJointVel[j] - mMotorVelocity[i][j]);
            }
        }
        else
        {
            for(int j=0; j<3; j++)
            {
                mLegTorque[i][j] = mStandPgain[j] * (mStandJointPos[j] - mMotorPosition[i][j])
                                   + mStandDgain[j] * (mStandJointVel[j] - mMotorVelocity[i][j]);
            }
        }
    }

    for(int idx=0; idx<4; idx++)
    {
        mTorque[idx][0] = mLegTorque[idx][0] + sharedMemory->mpcTorque[idx][0];
        mTorque[idx][1] = mLegTorque[idx][1] + sharedMemory->mpcTorque[idx][1];
        mTorque[idx][2] = mLegTorque[idx][2] + sharedMemory->mpcTorque[idx][2];
    }
}

void LowPDcontrol::setControlInput()
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