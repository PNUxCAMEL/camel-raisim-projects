//
// Created by hs on 22. 10. 14.
//

#include <canine_util/StateEstimator.hpp>

extern pSHM sharedMemory;

StateEstimator::StateEstimator(raisim::ArticulatedSystem* robot)
    : mRobot(robot)
    , mPosition(raisim::VecDyn(19))
    , mVelocity(raisim::VecDyn(18))
{
    bIsFirstRun = true;
    bIsRightRearFirst = true;
    bIsLeftRearFirst = true;
}

void StateEstimator::StateEstimatorFunction()
{
    if (sharedMemory->visualState == STATE_UPDATE_VISUAL)
    {
        if (sharedMemory->simulState == WITH_SIMULATION)
        {
            getJointStateReal();
            getRobotAngulerStateReal();

        }
        else
        {
            getJointStateSimul();
            getRobotAngulerStateSimul();
            getRobotFootPositionSimul();
        }
        calculateRobotLinearState();
    }
}

void StateEstimator::getJointStateReal()
{
    //TODO: Get real robot encoder value from can
}

void StateEstimator::getJointStateSimul()
{
    mPosition = mRobot->getGeneralizedCoordinate();
    mVelocity = mRobot->getGeneralizedVelocity();

    for (int idx=0; idx<MOTOR_NUM; idx++)
    {
        sharedMemory->motorPosition[idx] = mPosition[idx+7];
        sharedMemory->motorVelocity[idx] = mVelocity[idx+6];
    }
}

void StateEstimator::getRobotAngulerStateReal()
{
    //TODO: Get angular velocity, position from IMU
}

void StateEstimator::getRobotAngulerStateSimul()
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

void StateEstimator::getRobotFootPositionSimul() {
    //TODO:
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
//
    mTransMatRightRearHip2Base(0,0) = mTransMatBase2RightRearHip(0,0), mTransMatRightRearHip2Base(0,1) = mTransMatBase2RightRearHip(1,0), mTransMatRightRearHip2Base(0,2) = mTransMatBase2RightRearHip(2,0), mTransMatRightRearHip2Base(0,3) = -0.175;
    mTransMatRightRearHip2Base(1,0) = mTransMatBase2RightRearHip(0,1), mTransMatRightRearHip2Base(1,1) = mTransMatBase2RightRearHip(1,1), mTransMatRightRearHip2Base(1,2) = mTransMatBase2RightRearHip(2,1), mTransMatRightRearHip2Base(1,3) =  0.055*cos(mPosition[13]);
    mTransMatRightRearHip2Base(2,0) = mTransMatBase2RightRearHip(0,2), mTransMatRightRearHip2Base(2,1) = mTransMatBase2RightRearHip(1,2), mTransMatRightRearHip2Base(2,2) = mTransMatBase2RightRearHip(2,2), mTransMatRightRearHip2Base(2,3) = -0.055*sin(mPosition[13]);
    mTransMatRightRearHip2Base(3,0) = 0, mTransMatRightRearHip2Base(3,1) = 0, mTransMatRightRearHip2Base(3,2) = 0, mTransMatRightRearHip2Base(3,3) = 1;

    mTransMatRightRearThigh2Hip(0,0) = mTransMatRightRearHip2Thigh(0,0), mTransMatRightRearThigh2Hip(0,1) = mTransMatRightRearHip2Thigh(1,0), mTransMatRightRearThigh2Hip(0,2) = mTransMatRightRearHip2Thigh(2,0), mTransMatRightRearThigh2Hip(0,3) = -0.075*cos(mPosition[14]);
    mTransMatRightRearThigh2Hip(1,0) = mTransMatRightRearHip2Thigh(0,1), mTransMatRightRearThigh2Hip(1,1) = mTransMatRightRearHip2Thigh(1,1), mTransMatRightRearThigh2Hip(1,2) = mTransMatRightRearHip2Thigh(2,1), mTransMatRightRearThigh2Hip(1,3) =  0.030496;
    mTransMatRightRearThigh2Hip(2,0) = mTransMatRightRearHip2Thigh(0,2), mTransMatRightRearThigh2Hip(2,1) = mTransMatRightRearHip2Thigh(1,2), mTransMatRightRearThigh2Hip(2,2) = mTransMatRightRearHip2Thigh(2,2), mTransMatRightRearThigh2Hip(2,3) = -0.075*sin(mPosition[14]);
    mTransMatRightRearThigh2Hip(3,0) = 0, mTransMatRightRearThigh2Hip(3,1) = 0, mTransMatRightRearThigh2Hip(3,2) = 0, mTransMatRightRearThigh2Hip(3,3) = 1;

    mTransMatRightRearKnee2Thigh(0,0) = mTransMatRightRearThigh2Knee(0,0), mTransMatRightRearKnee2Thigh(0,1) = mTransMatRightRearThigh2Knee(1,0), mTransMatRightRearKnee2Thigh(0,2) = mTransMatRightRearThigh2Knee(2,0), mTransMatRightRearKnee2Thigh(0,3) = -0.23*sin(mPosition[15]);
    mTransMatRightRearKnee2Thigh(1,0) = mTransMatRightRearThigh2Knee(0,1), mTransMatRightRearKnee2Thigh(1,1) = mTransMatRightRearThigh2Knee(1,1), mTransMatRightRearKnee2Thigh(1,2) = mTransMatRightRearThigh2Knee(2,1), mTransMatRightRearKnee2Thigh(1,3) =  0.077;
    mTransMatRightRearKnee2Thigh(2,0) = mTransMatRightRearThigh2Knee(0,2), mTransMatRightRearKnee2Thigh(2,1) = mTransMatRightRearThigh2Knee(1,2), mTransMatRightRearKnee2Thigh(2,2) = mTransMatRightRearThigh2Knee(2,2), mTransMatRightRearKnee2Thigh(2,3) =  0.23*cos(mPosition[15]);
    mTransMatRightRearKnee2Thigh(3,0) = 0, mTransMatRightRearKnee2Thigh(3,1) = 0, mTransMatRightRearKnee2Thigh(3,2) = 0, mTransMatRightRearKnee2Thigh(3,3) = 1;

    mTransMatRightRearFoot2Knee(0,0) = mTransMatRightRearKnee2Foot(0,0), mTransMatRightRearFoot2Knee(0,1) = mTransMatRightRearKnee2Foot(1,0), mTransMatRightRearFoot2Knee(0,2) = mTransMatRightRearKnee2Foot(2,0), mTransMatRightRearFoot2Knee(0,3) = -0.23*sin(mTempAngleForRightRearFoot);
    mTransMatRightRearFoot2Knee(1,0) = mTransMatRightRearKnee2Foot(0,1), mTransMatRightRearFoot2Knee(1,1) = mTransMatRightRearKnee2Foot(1,1), mTransMatRightRearFoot2Knee(1,2) = mTransMatRightRearKnee2Foot(2,1), mTransMatRightRearFoot2Knee(1,3) =  0;
    mTransMatRightRearFoot2Knee(2,0) = mTransMatRightRearKnee2Foot(0,2), mTransMatRightRearFoot2Knee(2,1) = mTransMatRightRearKnee2Foot(1,2), mTransMatRightRearFoot2Knee(2,2) = mTransMatRightRearKnee2Foot(2,2), mTransMatRightRearFoot2Knee(2,3) =  0.23*cos(mTempAngleForRightRearFoot);
    mTransMatRightRearFoot2Knee(3,0) = 0, mTransMatRightRearFoot2Knee(3,1) = 0, mTransMatRightRearFoot2Knee(3,2) = 0, mTransMatRightRearFoot2Knee(3,3) = 1;

    mTransMatRightRearGround2Foot(0,0) = mTransMatRightRearFoot2Ground(0,0), mTransMatRightRearGround2Foot(0,1) = mTransMatRightRearFoot2Ground(1,0), mTransMatRightRearGround2Foot(0,2) = mTransMatRightRearFoot2Ground(2,0), mTransMatRightRearGround2Foot(0,3) = 0;
    mTransMatRightRearGround2Foot(1,0) = mTransMatRightRearFoot2Ground(0,1), mTransMatRightRearGround2Foot(1,1) = mTransMatRightRearFoot2Ground(1,1), mTransMatRightRearGround2Foot(1,2) = mTransMatRightRearFoot2Ground(2,1), mTransMatRightRearGround2Foot(1,3) = 0;
    mTransMatRightRearGround2Foot(2,0) = mTransMatRightRearFoot2Ground(0,2), mTransMatRightRearGround2Foot(2,1) = mTransMatRightRearFoot2Ground(1,2), mTransMatRightRearGround2Foot(2,2) = mTransMatRightRearFoot2Ground(2,2), mTransMatRightRearGround2Foot(2,3) = 0.015;
    mTransMatRightRearGround2Foot(3,0) = 0, mTransMatRightRearGround2Foot(3,1) = 0, mTransMatRightRearGround2Foot(3,2) = 0, mTransMatRightRearGround2Foot(3,3) = 1;
//
//    mTransMatRightRearHip2Base = mTransMatBase2RightRearHip.inverse();
//    mTransMatRightRearThigh2Hip = mTransMatRightRearHip2Thigh.inverse();
//    mTransMatRightRearKnee2Thigh = mTransMatRightRearThigh2Knee.inverse();
//    mTransMatRightRearFoot2Knee = mTransMatRightRearKnee2Foot.inverse();
//    mTransMatRightRearGround2Foot = mTransMatRightRearFoot2Ground.inverse();

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

}

void StateEstimator::calculateRobotLinearState()
{
    //TODO: Calculate global body position and velocity
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
