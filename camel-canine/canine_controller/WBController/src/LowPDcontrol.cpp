//
// Created by hs on 22. 10. 27.
//

#include <WBController/LowPDcontrol.hpp>
#include <iostream>

extern pSHM sharedMemory;

LowPDcontrol::LowPDcontrol()
    : mTorqueLimit(35)
    , SwingLegTrajectory(GAIT_PERIOD/2)
    , bIsFirstRunTrot(true)
    , bIsFirstRunSwing{true,true,true,true}
//    , mSwingPgain{50,50,50}
//    , mSwingDgain{2,2,2}
//    , mStandPgain{5,5,5}
//    , mStandDgain{1,1,1}
    , mSwingPgain{30,30,30}
    , mSwingDgain{1,1,1}
    , mStandPgain{5,5,5}
    , mStandDgain{1,1,1}
{
    mTorque->setZero();
    mShoulderPosition[0] <<  SHOULD_X_POS, -SHOULD_Y_POS, 0;
    mShoulderPosition[1] <<  SHOULD_X_POS,  SHOULD_Y_POS, 0;
    mShoulderPosition[2] << -SHOULD_X_POS, -SHOULD_Y_POS, 0;
    mShoulderPosition[3] << -SHOULD_X_POS,  SHOULD_Y_POS, 0;
    mHipPosition[0] <<  HIP_X_POS, -HIP_Y_POS, 0;
    mHipPosition[1] <<  HIP_X_POS,  HIP_Y_POS, 0;
    mHipPosition[2] << -HIP_X_POS, -HIP_Y_POS, 0;
    mHipPosition[3] << -HIP_X_POS,  HIP_Y_POS, 0;
}

void LowPDcontrol::DoControl()
{
    updateState();

    if (sharedMemory->gaitState != STAND)
    {
        if (bIsFirstRunTrot)
        {
            InitSwingTrjactory();
            bIsFirstRunTrot = false;
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
    for (int idx=0; idx<3; idx++)
    {
        mBasePosition[idx] = sharedMemory->basePosition[idx];
        mBaseVelocity[idx] = sharedMemory->baseVelocity[idx];
    }

    for (int idx=0; idx<4; idx++)
    {
        mBaseQuaternion[idx] = sharedMemory->baseQuartPosition[idx];
        mBodyFootPosition[idx] = sharedMemory->bodyFootPosition[idx];
        mGlobalFootPosition[idx] = sharedMemory->globalFootPosition[idx];
        for (int mt=0; mt<3; mt++)
        {
            mMotorPosition[idx][mt] = sharedMemory->motorPosition[idx*3+mt];
            mMotorVelocity[idx][mt] = sharedMemory->motorVelocity[idx*3+mt];

        }
    }
}

void LowPDcontrol::setLegControl()
{
    for (int leg = 0; leg < 4; leg++)
    {
        if (sharedMemory->gaitTable[leg] == 0)
        {
            if (bIsFirstRunSwing[leg] == true)
            {
                mDesiredFootPosition[leg] = mBasePosition + GetBaseRotationMatInverse(mBaseQuaternion)*mShoulderPosition[leg];
                mDesiredFootPosition[leg] += mBaseVelocity*GAIT_PERIOD/4 + 0.1*(mBaseVelocity-sharedMemory->desiredLinearVelocity);
                mDesiredFootPosition[leg] += (std::sqrt(-mBasePosition[2]/GRAVITY)/2)*GetSkew(mBaseVelocity)*sharedMemory->desiredAngularVelocity;
                SwingLegTrajectory.SetControlPoints(mGlobalFootPosition[leg], mDesiredFootPosition[leg], leg);
                bIsFirstRunSwing[leg] = false;
            }

            SwingLegTrajectory.GetPositionTrajectory(sharedMemory->localTime, mSwingFootPosition, leg); //global
            getJointPos(mSwingJointPos, mSwingFootPosition, leg, false);
            for(int mt=0; mt<3; mt++)
            {
                mLegTorque[leg][mt] = mSwingPgain[mt] * (mSwingJointPos[mt] - mMotorPosition[leg][mt])
                                   + mSwingDgain[mt] * (mSwingJointVel[mt] - mMotorVelocity[leg][mt]);
            }
        }
        else
        {
            if (bIsFirstRunSwing[leg] == false)
            {
                bIsFirstRunSwing[leg] = true;
            }
            getJointPos(mStandJointPos, mGlobalFootPosition[leg], leg, true);
            for(int mt=0; mt<3; mt++)
            {
                mLegTorque[leg][mt] = mStandPgain[mt] * (mStandJointPos[mt] - mMotorPosition[leg][mt])
                                   + mStandDgain[mt] * (mStandJointVel[mt] - mMotorVelocity[leg][mt]);
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

void LowPDcontrol::getJointPos(Vec3<double>& jointPos, Vec3<double> footPos, const int& leg, const bool stand)
{
    footPos = GetBaseRotationMat(mBaseQuaternion)*(footPos-mBasePosition)-mHipPosition[leg];
    if(stand)
    {
        footPos[0] = 0;
        switch (leg) {
            case 0:
            {
                footPos[1] = -0.107496;
                break;
            }
            case 1:
            {
                footPos[1] = 0.107496;
                break;
            }
            case 2:
            {
                footPos[1] = -0.107496;
                break;
            }
            case 3:
            {
                footPos[1] = 0.107496;
                break;
            }
            default:
            {
                break;
            }
        }
        footPos[2] = 0.35;
    }
    GetLegInvKinematics(jointPos, footPos, leg);
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