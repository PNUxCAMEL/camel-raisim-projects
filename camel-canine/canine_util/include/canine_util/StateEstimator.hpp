//
// Created by hs on 22. 10. 14.
//

#ifndef RAISIM_STATEESTIMATOR_HPP
#define RAISIM_STATEESTIMATOR_HPP

#include <raisim/World.hpp>

#include "RobotDescription.hpp"
#include "SharedMemory.hpp"
#include "RobotMath.hpp"

class StateEstimator{
public:
    StateEstimator(raisim::ArticulatedSystem* robot);

    void StateEstimatorFunction();
private:
    void getJointStateReal();
    void getJointStateSimul();
    void getRobotAngulerStateReal();
    void getRobotAngulerStateSimul();
    void getRobotFootPositionSimul();
    void calculateRobotLinearState();
private:
    raisim::ArticulatedSystem* mRobot;
    raisim::VecDyn mPosition;
    raisim::VecDyn mVelocity;
    raisim::Vec<3> mFootPosition[4];
    double mQuaternion[4];

    double mTempAngleForRightRearFoot;
    double mTempAngleForLeftRearFoot;


    Eigen::Matrix4d mTransMatBase2LeftRearGround;
    Eigen::Matrix4d mTransMatBase2LeftRearHip;
    Eigen::Matrix4d mTransMatBase2RightRearGround;
    Eigen::Matrix4d mTransMatBase2RightRearHip;

    Eigen::Matrix4d mTransMatLeftRearHip2Thigh;
    Eigen::Matrix4d mTransMatLeftRearThigh2Knee;
    Eigen::Matrix4d mTransMatLeftRearKnee2Foot;
    Eigen::Matrix4d mTransMatLeftRearFoot2Ground;
    Eigen::Matrix4d mTransMatLeftRearHip2Base;
    Eigen::Matrix4d mTransMatLeftRearThigh2Hip;
    Eigen::Matrix4d mTransMatLeftRearKnee2Thigh;
    Eigen::Matrix4d mTransMatLeftRearFoot2Knee;
    Eigen::Matrix4d mTransMatLeftRearGround2Foot;
    Eigen::Matrix4d mTransMatLeftRearGround2Base;


    Eigen::Matrix4d mTransMatRightRearHip2Thigh;
    Eigen::Matrix4d mTransMatRightRearThigh2Knee;
    Eigen::Matrix4d mTransMatRightRearKnee2Foot;
    Eigen::Matrix4d mTransMatRightRearFoot2Ground;
    Eigen::Matrix4d mTransMatRightRearHip2Base;
    Eigen::Matrix4d mTransMatRightRearThigh2Hip;
    Eigen::Matrix4d mTransMatRightRearKnee2Thigh;
    Eigen::Matrix4d mTransMatRightRearFoot2Knee;
    Eigen::Matrix4d mTransMatRightRearGround2Foot;
    Eigen::Matrix4d mTransMatRightRearGround2Base;

    Eigen::Vector4d mBodyPosition;
    Eigen::Vector4d mBodyPositionPrev;
    Eigen::Vector4d mRightRearFootPosition;
    Eigen::Vector4d mLeftRearFootPosition;
    Eigen::Vector4d mStartBodyPosision;
    Eigen::Vector4d mWorldBodyPosition;
    Eigen::Vector4d mWorldBodyPositionPrev;
    Eigen::Vector4d mBodyVelocity;

    bool bIsRightRearFirst; /// is it the first iteration of the right rear leg touching the floor.
    bool bIsLeftRearFirst; /// is it the first iteration of the left rear leg touching the floor.
    bool bIsFirstRun;



};

#endif //RAISIM_STATEESTIMATOR_HPP
