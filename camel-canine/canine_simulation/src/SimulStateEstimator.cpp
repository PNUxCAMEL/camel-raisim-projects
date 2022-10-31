//
// Created by hs on 22. 10. 27.
//

#include <canine_simulation/SimulStateEstimator.hpp>

extern pSHM sharedMemory;

SimulStateEstimator::SimulStateEstimator(raisim::ArticulatedSystem* robot)
        : mRobot(robot)
        , mPosition(raisim::VecDyn(19))
        , mVelocity(raisim::VecDyn(18))
{
    bIsFirstRun = true;
    bIsRightRearFirst = true;
    bIsLeftRearFirst = true;
}

void SimulStateEstimator::StateEstimatorFunction()
{
    mPosition = mRobot->getGeneralizedCoordinate();
    mVelocity = mRobot->getGeneralizedVelocity();

    getJointState();
    getRobotAngulerState();
    getRobotLinearState();
    getRobotFootPosition();
}


void SimulStateEstimator::getJointState()
{
    mPosition = mRobot->getGeneralizedCoordinate();
    mVelocity = mRobot->getGeneralizedVelocity();

    for (int idx=0; idx<MOTOR_NUM; idx++)
    {
        sharedMemory->motorPosition[idx] = mPosition[idx+7];
        sharedMemory->motorVelocity[idx] = mVelocity[idx+6];
    }
}

void SimulStateEstimator::getRobotAngulerState()
{
    for(int idx=0; idx<3; idx++)
    {
        sharedMemory->baseEulerVelocity[idx] = mVelocity[idx+3];
    }

    for(int idx=0; idx<4; idx++)
    {
        mQuaternion[idx] = mPosition[idx+3];
    }
    TransformQuat2Euler(mQuaternion, sharedMemory->baseEulerPosition);
}

void SimulStateEstimator::getRobotLinearState()
{
/*    for(int idx=0; idx<3; idx++)
    {
        sharedMemory->basePosition[idx] = mPosition[idx];
        sharedMemory->baseVelocity[idx] = mVelocity[idx];
    }*/

    if(bIsFirstRun == 1)
    {
        bIsFirstRun = 0;
        mWorldBodyPosition[0] = 0;
        mWorldBodyPosition[1] = 0;
        mWorldBodyPosition[2] = mTransMatRightRearGround2Base(2,3);
        mWorldBodyPosition[3] = 1;
        mWorldBodyPositionPrev = mWorldBodyPosition;
        mBodyVelocity << 0,0,0,1;
    }

    if(sharedMemory->gaitTable[2] == 1)
    {
        if(bIsRightRearFirst)
        {
            bIsRightRearFirst = false;
            bIsLeftRearFirst = true;
            mStartBodyPosision = mWorldBodyPosition;
            mBodyPositionPrev = mTransMatRightRearGround2Base * mWorldBodyPositionPrev;
        }
        mBodyPosition = mTransMatRightRearGround2Base * mStartBodyPosision;
        mWorldBodyPosition = mWorldBodyPosition + mBodyPosition - mBodyPositionPrev;
        mBodyVelocity = (mWorldBodyPosition - mWorldBodyPositionPrev)/0.005;
        mBodyPositionPrev = mBodyPosition;
        mWorldBodyPositionPrev = mWorldBodyPosition;

    }
    else
    {
        if(bIsLeftRearFirst)
        {
            bIsLeftRearFirst = false;
            bIsRightRearFirst = true;
            mStartBodyPosision = mWorldBodyPosition;
            mBodyPositionPrev = mTransMatLeftRearGround2Base * mWorldBodyPositionPrev;
        }
        mBodyPosition = mTransMatLeftRearGround2Base * mStartBodyPosision;
        mWorldBodyPosition = mWorldBodyPosition + mBodyPosition - mBodyPositionPrev;
        mBodyVelocity = (mWorldBodyPosition - mWorldBodyPositionPrev)/0.005;
        mBodyPositionPrev = mBodyPosition;
        mWorldBodyPositionPrev = mWorldBodyPosition;
    }

    mPosition[0] = mWorldBodyPosition[0];
    mPosition[1] = mWorldBodyPosition[1];
    mPosition[2] = mWorldBodyPosition[2];
    mVelocity[0] = mBodyVelocity[0];
    mVelocity[1] = mBodyVelocity[1];
    mVelocity[2] = mBodyVelocity[2];


    for(int idx=0; idx<3; idx++)
    {
        sharedMemory->basePosition[idx] = mPosition[idx];
        sharedMemory->baseVelocity[idx] = mVelocity[idx];
    }
}

void SimulStateEstimator::getRobotFootPosition()
{
/*    auto FRfootFrameIndex = mRobot->getFrameIdxByName("RF_FOOT");
    auto FLfootFrameIndex = mRobot->getFrameIdxByName("LF_FOOT");
    auto RRfootFrameIndex = mRobot->getFrameIdxByName("RH_FOOT");
    auto RLfootFrameIndex = mRobot->getFrameIdxByName("LH_FOOT");

    mRobot->getFramePosition(FRfootFrameIndex, mFootPosition[0]);
    mRobot->getFramePosition(FLfootFrameIndex, mFootPosition[1]);
    mRobot->getFramePosition(RRfootFrameIndex, mFootPosition[2]);
    mRobot->getFramePosition(RLfootFrameIndex, mFootPosition[3]);*/

///right
    mTransMatBase2RightRearHip<< 1, 0, 0, -0.175,
        0, std::cos(mPosition[13]), -std::sin(mPosition[13]), 0.055,
        0, std::sin(mPosition[13]), std::cos(mPosition[13]), 0,
        0, 0, 0, 1;
    mTransMatRightRearHip2Thigh << std::cos(mPosition[14]), 0, std::sin(mPosition[14]), -0.0705,
        0, 1, 0, -0.030496,
        -std::sin(mPosition[14]), 0, std::cos(mPosition[14]), 0,
        0, 0, 0, 1;
    mTransMatRightRearThigh2Knee << std::cos(mPosition[15]), 0, std::sin(mPosition[15]), 0,
        0, 1, 0, -0.077,
        -std::sin(mPosition[15]), 0, std::cos(mPosition[15]), -0.23,
        0, 0, 0, 1;

    mTempAngleForRightRearFoot = -(mPosition[15]+mPosition[14]);
    mTransMatRightRearKnee2Foot << std::cos(mTempAngleForRightRearFoot), 0, std::sin(mTempAngleForRightRearFoot), 0,
        0, 1, 0, 0,
        -std::sin(mTempAngleForRightRearFoot), 0, std::cos(mTempAngleForRightRearFoot), -0.23, //-0.23
        0, 0, 0, 1;

    mTransMatRightRearFoot2Ground << 1,0,0,0,
        0,1,0,0,
        0,0,1,-0.015,
        0,0,0,1;

    mTransMatRightRearHip2Base = mTransMatBase2RightRearHip.inverse();
    mTransMatRightRearThigh2Hip = mTransMatRightRearHip2Thigh.inverse();
    mTransMatRightRearKnee2Thigh = mTransMatRightRearThigh2Knee.inverse();
    mTransMatRightRearFoot2Knee = mTransMatRightRearKnee2Foot.inverse();
    mTransMatRightRearGround2Foot = mTransMatRightRearFoot2Ground.inverse();

    mTransMatBase2RightRearGround = mTransMatBase2RightRearHip * mTransMatRightRearHip2Thigh * mTransMatRightRearThigh2Knee * mTransMatRightRearKnee2Foot * mTransMatRightRearFoot2Ground;
    mTransMatRightRearGround2Base = mTransMatRightRearGround2Foot * mTransMatRightRearFoot2Knee * mTransMatRightRearKnee2Thigh * mTransMatRightRearThigh2Hip * mTransMatRightRearHip2Base;

    ///left
    mTransMatBase2LeftRearHip<< 1, 0, 0, -0.175,
        0, std::cos(mPosition[16]), -std::sin(mPosition[16]), 0.055,
        0, std::sin(mPosition[16]), std::cos(mPosition[16]), 0,
        0, 0, 0, 1;
    mTransMatLeftRearHip2Thigh<< std::cos(mPosition[17]), 0, std::sin(mPosition[17]), -0.0705,
        0, 1, 0, 0.030496,
        -std::sin(mPosition[17]), 0, std::cos(mPosition[17]), 0,
        0, 0, 0, 1;
    mTransMatLeftRearThigh2Knee<< std::cos(mPosition[18]), 0, std::sin(mPosition[18]), 0,
        0, 1, 0, 0.077,
        -std::sin(mPosition[18]), 0, std::cos(mPosition[18]), -0.23,
        0, 0, 0, 1;

    mTempAngleForLeftRearFoot = -(mPosition[18]+mPosition[17]);
    mTransMatLeftRearKnee2Foot << std::cos(mTempAngleForLeftRearFoot), 0, std::sin(mTempAngleForLeftRearFoot), 0,
        0, 1, 0, 0,
        -std::sin(mTempAngleForLeftRearFoot), 0, std::cos(mTempAngleForLeftRearFoot), -0.23, //-0.23
        0, 0, 0, 1;

    mTransMatLeftRearFoot2Ground << 1,0,0,0,
        0,1,0,0,
        0,0,1,-0.015,
        0,0,0,1;


    mTransMatLeftRearHip2Base = mTransMatBase2LeftRearHip.inverse();
    mTransMatLeftRearThigh2Hip = mTransMatLeftRearHip2Thigh.inverse();
    mTransMatLeftRearKnee2Thigh = mTransMatLeftRearThigh2Knee.inverse();
    mTransMatLeftRearFoot2Knee = mTransMatLeftRearKnee2Foot.inverse();
    mTransMatLeftRearGround2Foot = mTransMatLeftRearFoot2Ground.inverse();

    mTransMatBase2LeftRearGround = mTransMatBase2LeftRearHip * mTransMatLeftRearHip2Thigh * mTransMatLeftRearThigh2Knee * mTransMatLeftRearKnee2Foot * mTransMatLeftRearFoot2Ground;
    mTransMatLeftRearGround2Base = mTransMatLeftRearGround2Foot * mTransMatLeftRearFoot2Knee * mTransMatLeftRearKnee2Thigh * mTransMatLeftRearThigh2Hip * mTransMatLeftRearHip2Base;

    mRobot->getFramePosition(RLfootFrameIndex, mFootPosition[3]);

    for (int leg=0; leg<4; leg++)
    {
        for (int mt=0; mt<3; mt++)
        {
            sharedMemory->footPosition[leg][mt] = mFootPosition[leg][mt];
        }
    }
}