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
    bFirstRunOfRightRearFoot = 1;
    mWorldBodyPosition << robot->getGeneralizedCoordinate()[0], robot->getGeneralizedCoordinate()[1], robot->getGeneralizedCoordinate()[2], 1;
    mWorldBodyPositionPrev << robot->getGeneralizedCoordinate()[0], robot->getGeneralizedCoordinate()[1], robot->getGeneralizedCoordinate()[2], 1;

    ///right
    mTransMatBase2RightRearHip<< 1, 0, 0, -0.175,
                                            0, std::cos(robot->getGeneralizedCoordinate()[13]), -std::sin(robot->getGeneralizedCoordinate()[13]), 0.055,
                                            0, std::sin(robot->getGeneralizedCoordinate()[13]), std::cos(robot->getGeneralizedCoordinate()[13]), 0,
                                            0, 0, 0, 1;
    mTransMatRightRearHip2Thigh << std::cos(robot->getGeneralizedCoordinate()[14]), 0, std::sin(robot->getGeneralizedCoordinate()[14]), -0.0705,
                                            0, 1, 0, -0.030496,
                                            -std::sin(robot->getGeneralizedCoordinate()[14]), 0, std::cos(robot->getGeneralizedCoordinate()[14]), 0,
                                            0, 0, 0, 1;
    mTransMatRightRearThigh2Knee << std::cos(robot->getGeneralizedCoordinate()[15]), 0, std::sin(robot->getGeneralizedCoordinate()[15]), 0,
                                            0, 1, 0, -0.077,
                                            -std::sin(robot->getGeneralizedCoordinate()[15]), 0, std::cos(robot->getGeneralizedCoordinate()[15]), -0.23,
                                            0, 0, 0, 1;

    mTempAngleForRightRearFoot = -(robot->getGeneralizedCoordinate()[15]+robot->getGeneralizedCoordinate()[14]);
    mTransMatRightRearKnee2Foot << std::cos(mTempAngleForRightRearFoot), 0, std::sin(mTempAngleForRightRearFoot), 0,
                                            0, 1, 0, 0,
                                            -std::sin(mTempAngleForRightRearFoot), 0, std::cos(mTempAngleForRightRearFoot), -0.195, //-0.23
                                            0, 0, 0, 1;

    mTransMatRightRearHip2Base = mTransMatBase2RightRearHip.inverse();
    mTransMatRightRearThigh2Hip = mTransMatRightRearHip2Thigh.inverse();
    mTransMatRightRearKnee2Thigh = mTransMatRightRearThigh2Knee.inverse();
    mTransMatRightRearFoot2Knee = mTransMatRightRearKnee2Foot.inverse();

    mTransMatBase2RightRearFoot = mTransMatBase2RightRearHip * mTransMatRightRearHip2Thigh * mTransMatRightRearThigh2Knee * mTransMatRightRearKnee2Foot;
    mTransMatRightRearFoot2Base = mTransMatRightRearFoot2Knee * mTransMatRightRearKnee2Thigh * mTransMatRightRearThigh2Hip * mTransMatRightRearHip2Base;

    ///left
    mTransMatBase2LeftRearHip<< 1, 0, 0, -0.175,
        0, std::cos(robot->getGeneralizedCoordinate()[16]), -std::sin(robot->getGeneralizedCoordinate()[16]), 0.055,
        0, std::sin(robot->getGeneralizedCoordinate()[16]), std::cos(robot->getGeneralizedCoordinate()[16]), 0,
        0, 0, 0, 1;
    mTransMatLeftRearHip2Thigh<< std::cos(robot->getGeneralizedCoordinate()[17]), 0, std::sin(robot->getGeneralizedCoordinate()[17]), -0.0705,
        0, 1, 0, 0.030496,
        -std::sin(robot->getGeneralizedCoordinate()[17]), 0, std::cos(robot->getGeneralizedCoordinate()[17]), 0,
        0, 0, 0, 1;
    mTransMatLeftRearThigh2Knee<< std::cos(robot->getGeneralizedCoordinate()[18]), 0, std::sin(robot->getGeneralizedCoordinate()[18]), 0,
        0, 1, 0, 0.077,
        -std::sin(robot->getGeneralizedCoordinate()[18]), 0, std::cos(robot->getGeneralizedCoordinate()[18]), -0.23,
        0, 0, 0, 1;

    mTempAngleForLeftRearFoot = -(robot->getGeneralizedCoordinate()[18]+robot->getGeneralizedCoordinate()[17]);
    mTransMatLeftRearKnee2Foot << std::cos(mTempAngleForLeftRearFoot), 0, std::sin(mTempAngleForLeftRearFoot), 0,
        0, 1, 0, 0,
        -std::sin(mTempAngleForLeftRearFoot), 0, std::cos(mTempAngleForLeftRearFoot), -0.195, //-0.23
        0, 0, 0, 1;

    mTransMatLeftRearHip2Base = mTransMatBase2LeftRearHip.inverse();
    mTransMatLeftRearThigh2Hip = mTransMatLeftRearHip2Thigh.inverse();
    mTransMatLeftRearKnee2Thigh = mTransMatLeftRearThigh2Knee.inverse();
    mTransMatLeftRearFoot2Knee = mTransMatLeftRearKnee2Foot.inverse();

    mTransMatBase2LeftRearFoot = mTransMatBase2LeftRearHip * mTransMatLeftRearHip2Thigh * mTransMatLeftRearThigh2Knee * mTransMatLeftRearKnee2Foot;
    mTransMatLeftRearFoot2Base = mTransMatLeftRearFoot2Knee * mTransMatLeftRearKnee2Thigh * mTransMatLeftRearThigh2Hip * mTransMatLeftRearHip2Base;

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

    mTransMatRightRearHip2Base = mTransMatBase2RightRearHip.inverse();
    mTransMatRightRearThigh2Hip = mTransMatRightRearHip2Thigh.inverse();
    mTransMatRightRearKnee2Thigh = mTransMatRightRearThigh2Knee.inverse();
    mTransMatRightRearFoot2Knee = mTransMatRightRearKnee2Foot.inverse();

    mTransMatBase2RightRearFoot = mTransMatBase2RightRearHip * mTransMatRightRearHip2Thigh * mTransMatRightRearThigh2Knee * mTransMatRightRearKnee2Foot;
    mTransMatRightRearFoot2Base = mTransMatRightRearFoot2Knee * mTransMatRightRearKnee2Thigh * mTransMatRightRearThigh2Hip * mTransMatRightRearHip2Base;

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

//    mTransMatContact

    mTransMatLeftRearHip2Base = mTransMatBase2LeftRearHip.inverse();
    mTransMatLeftRearThigh2Hip = mTransMatLeftRearHip2Thigh.inverse();
    mTransMatLeftRearKnee2Thigh = mTransMatLeftRearThigh2Knee.inverse();
    mTransMatLeftRearFoot2Knee = mTransMatLeftRearKnee2Foot.inverse();

    mTransMatBase2LeftRearFoot = mTransMatBase2LeftRearHip * mTransMatLeftRearHip2Thigh * mTransMatLeftRearThigh2Knee * mTransMatLeftRearKnee2Foot;
    mTransMatLeftRearFoot2Base = mTransMatLeftRearFoot2Knee * mTransMatLeftRearKnee2Thigh * mTransMatLeftRearThigh2Hip * mTransMatLeftRearHip2Base;

}

void StateEstimator::calculateRobotLinearState()
{
    //TODO: Calculate global body position and velocity
    if(bFirstRunOfRightRearFoot == 1)
    {
        bFirstRunOfRightRearFoot = 0;
        mStartBodyPosision = mWorldBodyPosition;
        mBodyPositionPrev = mTransMatLeftRearFoot2Base * mStartBodyPosision;
    }
    mBodyPosition = mTransMatLeftRearFoot2Base * mStartBodyPosision;
    mWorldBodyPosition = mWorldBodyPosition + mBodyPosition - mBodyPositionPrev;
    mBodyPositionPrev = mBodyPosition;

    for(int idx=0; idx<3; idx++)
    {
        sharedMemory->basePosition[idx] = mPosition[idx];
        sharedMemory->baseVelocity[idx] = mVelocity[idx];
    }

    std::cout << mWorldBodyPosition[0] << std::endl;
}
